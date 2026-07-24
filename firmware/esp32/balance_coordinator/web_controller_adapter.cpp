/* SPDX-License-Identifier: GPL-3.0-only */
#include "web_controller_adapter.h"

#if GS_ENABLE_WEB_CONTROL

#include <mbedtls/base64.h>
#include <mbedtls/sha1.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace gs::balance {
namespace {

constexpr uint64_t kStatusPeriodUs = 100000u;
constexpr size_t kMaximumReadPerService = 64u;
constexpr size_t kMaximumWritePerService = 64u;
constexpr char kWebSocketGuid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

constexpr char kControllerPage[] = R"HTML(<!doctype html>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>GAUSSTOP dry-run controller</title>
<style>
body{font:16px system-ui;max-width:42rem;margin:auto;padding:1rem;background:#111;color:#eee}
button,input{font:inherit;margin:.35rem}.danger{background:#b22;color:white}
.row{display:flex;gap:.5rem;flex-wrap:wrap}.status{font-family:monospace;white-space:pre-wrap}
label{display:block;margin-top:1rem}input[type=range]{width:100%}
</style>
<h1>GAUSSTOP balance</h1>
<div class="row"><button id=a>Arm</button><button id=d>Disarm</button>
<button id=s>Stop</button><button class=danger id=e>Emergency stop</button></div>
<label>Velocity <output id=vo>0</output><input id=v type=range min=-1 max=1 step=.01 value=0></label>
<label>Yaw <output id=yo>0</output><input id=y type=range min=-1 max=1 step=.01 value=0></label>
<p id=c>connecting</p><div class=status id=t></div>
<script>
const q=id=>document.getElementById(id),ws=new WebSocket(`ws://${location.host}/control`);
const send=x=>ws.readyState===1&&ws.send(x),move=()=>send(`move,${q('v').value},${q('y').value}`);
ws.onopen=()=>q('c').textContent='connected';ws.onclose=()=>q('c').textContent='disconnected';
ws.onmessage=e=>q('t').textContent=e.data;
q('a').onclick=()=>send('arm');q('d').onclick=()=>send('disarm');
q('s').onclick=()=>send('stop');q('e').onclick=()=>send('estop');
for(const id of ['v','y']){q(id).oninput=()=>{q(id+'o').value=q(id).value;move()};
q(id).onchange=move;for(const e of ['pointerup','pointercancel','lostpointercapture'])
q(id).addEventListener(e,()=>{q('v').value=0;q('y').value=0;move()})}
addEventListener('blur',()=>{q('v').value=0;q('y').value=0;move()});
addEventListener('beforeunload',()=>send('stop'));
</script>)HTML";

const char *headerValue(const char *request, const char *name, char *output,
                        size_t capacity) {
  const char *start = std::strstr(request, name);
  if (start == nullptr) {
    return nullptr;
  }
  start += std::strlen(name);
  while (*start == ' ') {
    ++start;
  }
  const char *end = std::strstr(start, "\r\n");
  if (end == nullptr || static_cast<size_t>(end - start) >= capacity) {
    return nullptr;
  }
  const size_t length = static_cast<size_t>(end - start);
  std::memcpy(output, start, length);
  output[length] = '\0';
  return output;
}

} // namespace

WebControllerAdapter::WebControllerAdapter(WebCommandSource &command_source)
    : command_source_(command_source) {}

bool WebControllerAdapter::begin(const char *ssid, const char *password) {
  if (ssid == nullptr || ssid[0] == '\0') {
    return false;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  return true;
}

void WebControllerAdapter::service(uint64_t now_us, const char *state,
                                   float pitch_deg, float left, float right,
                                   bool imu_healthy, bool calibrated,
                                   bool motor_healthy, bool dry_run,
                                   uint32_t faults) {
  if (!server_started_ && WiFi.status() == WL_CONNECTED) {
    server_.begin();
    server_started_ = true;
  }
  if (!server_started_) {
    return;
  }
  acceptClient();
  writePendingResponse();
  if (responsePending()) {
    return;
  }
  readClient(now_us);
  sendStatus(now_us, state, pitch_deg, left, right, imu_healthy, calibrated,
             motor_healthy, dry_run, faults);
}

void WebControllerAdapter::acceptClient() {
  if (client_ && client_.connected()) {
    return;
  }
  if (client_) {
    closeClient();
  }
  client_ = server_.available();
}

void WebControllerAdapter::writePendingResponse() {
  if (!client_ || !client_.connected() || !responsePending()) {
    return;
  }
  const size_t available =
      std::min(static_cast<size_t>(client_.availableForWrite()),
               kMaximumWritePerService);
  if (available == 0u) {
    return;
  }
  if (response_header_offset_ < response_header_length_) {
    const size_t remaining = response_header_length_ - response_header_offset_;
    const size_t length = std::min(available, remaining);
    client_.write(reinterpret_cast<const uint8_t *>(
                      &response_header_[response_header_offset_]),
                  length);
    response_header_offset_ += length;
    return;
  }
  if (page_response_ && response_body_offset_ < sizeof(kControllerPage) - 1u) {
    const size_t remaining =
        sizeof(kControllerPage) - 1u - response_body_offset_;
    const size_t length = std::min(available, remaining);
    client_.write(reinterpret_cast<const uint8_t *>(
                      &kControllerPage[response_body_offset_]),
                  length);
    response_body_offset_ += length;
    return;
  }
  if (upgrade_response_) {
    upgraded_ = true;
    upgrade_response_ = false;
    response_header_length_ = 0u;
    response_header_offset_ = 0u;
    receive_length_ = 0u;
    return;
  }
  closeClient();
}

bool WebControllerAdapter::responsePending() const {
  return response_header_length_ != 0u || page_response_ || upgrade_response_;
}

void WebControllerAdapter::readClient(uint64_t now_us) {
  if (!client_ || !client_.connected()) {
    return;
  }
  size_t read_count = 0u;
  while (client_.available() > 0 && read_count < kMaximumReadPerService &&
         receive_length_ < sizeof(receive_buffer_) - 1u) {
    receive_buffer_[receive_length_++] = static_cast<char>(client_.read());
    ++read_count;
  }
  receive_buffer_[receive_length_] = '\0';
  if (!upgraded_ && std::strstr(receive_buffer_, "\r\n\r\n") != nullptr) {
    handleHttpRequest();
  }
  if (upgraded_) {
    handleWebSocketFrames(now_us);
  }
}

void WebControllerAdapter::handleHttpRequest() {
  if (std::strstr(receive_buffer_, "Upgrade: websocket") != nullptr &&
      upgradeWebSocket(receive_buffer_)) {
    receive_length_ = 0u;
    return;
  }
  servePage();
}

void WebControllerAdapter::handleWebSocketFrames(uint64_t now_us) {
  while (receive_length_ >= 6u) {
    const auto *bytes = reinterpret_cast<const uint8_t *>(receive_buffer_);
    const uint8_t payload_length = bytes[1] & 0x7fu;
    const bool masked = (bytes[1] & 0x80u) != 0u;
    if (!masked || payload_length > 125u) {
      closeClient();
      return;
    }
    const size_t frame_length = 6u + payload_length;
    if (receive_length_ < frame_length) {
      return;
    }
    if ((bytes[0] & 0x0fu) == 0x08u) {
      closeClient();
      return;
    }
    char command[126] = {};
    for (size_t index = 0u; index < payload_length; ++index) {
      command[index] =
          static_cast<char>(bytes[6u + index] ^ bytes[2u + (index % 4u)]);
    }
    handleTextCommand(command, now_us);
    std::memmove(receive_buffer_, &receive_buffer_[frame_length],
                 receive_length_ - frame_length);
    receive_length_ -= frame_length;
  }
}

void WebControllerAdapter::handleTextCommand(char *command, uint64_t now_us) {
  if (std::strcmp(command, "arm") == 0) {
    command_source_.arm(now_us);
    return;
  }
  if (std::strcmp(command, "disarm") == 0) {
    command_source_.disarm(now_us);
    return;
  }
  if (std::strcmp(command, "stop") == 0) {
    command_source_.stop(now_us);
    return;
  }
  if (std::strcmp(command, "estop") == 0) {
    command_source_.emergencyStop(now_us);
    return;
  }
  float velocity = 0.0f;
  float yaw = 0.0f;
  char trailing = '\0';
  if (std::sscanf(command, "move,%f,%f%c", &velocity, &yaw, &trailing) == 2 &&
      std::isfinite(velocity) && std::isfinite(yaw)) {
    command_source_.movement(std::clamp(velocity, -1.0f, 1.0f),
                             std::clamp(yaw, -1.0f, 1.0f), now_us);
  }
}

void WebControllerAdapter::sendStatus(uint64_t now_us, const char *state,
                                      float pitch_deg, float left, float right,
                                      bool imu_healthy, bool calibrated,
                                      bool motor_healthy, bool dry_run,
                                      uint32_t faults) {
  if (!upgraded_ || now_us < next_status_us_) {
    return;
  }
  char payload[126] = {};
  const int written =
      std::snprintf(payload, sizeof(payload),
                    "{\"state\":\"%s\",\"mpu\":%u,\"cal\":%u,\"pitch\":%.2f,"
                    "\"left\":%.1f,\"right\":%.1f,\"motor\":%u,\"dry\":%u,"
                    "\"faults\":%lu}",
                    state, imu_healthy ? 1u : 0u, calibrated ? 1u : 0u,
                    pitch_deg, left, right, motor_healthy ? 1u : 0u,
                    dry_run ? 1u : 0u, static_cast<unsigned long>(faults));
  if (written <= 0 || written > 125 ||
      client_.availableForWrite() < written + 2) {
    return;
  }
  const uint8_t header[] = {0x81u, static_cast<uint8_t>(written)};
  client_.write(header, sizeof(header));
  client_.write(reinterpret_cast<const uint8_t *>(payload),
                static_cast<size_t>(written));
  next_status_us_ = now_us + kStatusPeriodUs;
}

bool WebControllerAdapter::upgradeWebSocket(const char *request) {
  char key[64] = {};
  if (headerValue(request, "Sec-WebSocket-Key:", key, sizeof(key)) == nullptr) {
    return false;
  }
  char combined[128] = {};
  const int length =
      std::snprintf(combined, sizeof(combined), "%s%s", key, kWebSocketGuid);
  unsigned char digest[20] = {};
  if (length <= 0 ||
      mbedtls_sha1_ret(reinterpret_cast<const unsigned char *>(combined),
                       static_cast<size_t>(length), digest) != 0) {
    return false;
  }
  unsigned char accept[64] = {};
  size_t accept_length = 0u;
  if (mbedtls_base64_encode(accept, sizeof(accept), &accept_length, digest,
                            sizeof(digest)) != 0) {
    return false;
  }
  const int written =
      std::snprintf(response_header_, sizeof(response_header_),
                    "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                    "Connection: Upgrade\r\nSec-WebSocket-Accept: %.*s\r\n\r\n",
                    static_cast<int>(accept_length), accept);
  if (written <= 0 ||
      static_cast<size_t>(written) >= sizeof(response_header_)) {
    return false;
  }
  response_header_length_ = static_cast<size_t>(written);
  response_header_offset_ = 0u;
  response_body_offset_ = 0u;
  page_response_ = false;
  upgrade_response_ = true;
  return true;
}

void WebControllerAdapter::servePage() {
  const int written = std::snprintf(
      response_header_, sizeof(response_header_),
      "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
      "Cache-Control: no-store\r\nContent-Length: %u\r\n\r\n",
      static_cast<unsigned>(sizeof(kControllerPage) - 1u));
  if (written <= 0 ||
      static_cast<size_t>(written) >= sizeof(response_header_)) {
    closeClient();
    return;
  }
  response_header_length_ = static_cast<size_t>(written);
  response_header_offset_ = 0u;
  response_body_offset_ = 0u;
  page_response_ = true;
  upgrade_response_ = false;
}

void WebControllerAdapter::closeClient() {
  const bool had_client =
      static_cast<bool>(client_) || upgraded_ || receive_length_ != 0u;
  if (client_) {
    client_.stop();
  }
  if (had_client) {
    command_source_.disconnect();
  }
  receive_length_ = 0u;
  response_header_length_ = 0u;
  response_header_offset_ = 0u;
  response_body_offset_ = 0u;
  page_response_ = false;
  upgrade_response_ = false;
  upgraded_ = false;
}

} // namespace gs::balance

#endif

/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#if GS_ENABLE_WEB_CONTROL

#include "web_command_source.h"

#include <WiFi.h>

#include <cstddef>
#include <cstdint>

namespace gs::balance {

class WebControllerAdapter {
public:
  explicit WebControllerAdapter(WebCommandSource &command_source);

  bool begin(const char *ssid, const char *password);
  void service(uint64_t now_us, const char *state, float pitch_deg, float left,
               float right, bool imu_healthy, bool calibrated,
               bool motor_healthy, bool dry_run, uint32_t faults);

private:
  void acceptClient();
  void writePendingResponse();
  bool responsePending() const;
  void readClient(uint64_t now_us);
  void handleHttpRequest();
  void handleWebSocketFrames(uint64_t now_us);
  void handleTextCommand(char *command, uint64_t now_us);
  void sendStatus(uint64_t now_us, const char *state, float pitch_deg,
                  float left, float right, bool imu_healthy, bool calibrated,
                  bool motor_healthy, bool dry_run, uint32_t faults);
  bool upgradeWebSocket(const char *request);
  void servePage();
  void closeClient();

  WebCommandSource &command_source_;
  WiFiServer server_{80u};
  WiFiClient client_{};
  char receive_buffer_[1024] = {};
  size_t receive_length_ = 0u;
  char response_header_[256] = {};
  size_t response_header_length_ = 0u;
  size_t response_header_offset_ = 0u;
  size_t response_body_offset_ = 0u;
  uint64_t next_status_us_ = 0u;
  bool server_started_ = false;
  bool upgraded_ = false;
  bool page_response_ = false;
  bool upgrade_response_ = false;
};

} // namespace gs::balance

#endif

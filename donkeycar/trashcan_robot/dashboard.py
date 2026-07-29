from __future__ import annotations

import threading
from typing import Any

from flask import Flask, Response, jsonify, render_template_string

from .state import RobotState


DASHBOARD_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Trashcan Robot</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { margin: 0; background: #111; color: #eee; }
    header { padding: 14px 18px; background: #1b1b1b; position: sticky; top: 0; z-index: 2; }
    h1, h2 { margin: 0 0 10px; }
    main { padding: 16px; display: grid; gap: 16px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit,minmax(320px,1fr)); gap: 16px; }
    .card { background: #1c1c1c; border: 1px solid #3a3a3a; border-radius: 10px; padding: 12px; }
    .camera { width: 100%; min-height: 220px; object-fit: contain; background: #000; }
    iframe { width: 100%; min-height: 620px; border: 0; background: #fff; }
    pre { white-space: pre-wrap; overflow-wrap: anywhere; background: #090909; padding: 12px; }
    .tag { display: inline-block; margin: 4px; padding: 8px; border: 1px solid #555; border-radius: 6px; }
    .ok { color: #69db7c; } .bad { color: #ff8787; }
    a { color: #74c0fc; }
  </style>
</head>
<body>
<header>
  <h1>Trashcan Robot</h1>
  <span id="connection">Loading robot status...</span>
</header>
<main>
  <section class="grid">
    <div class="card">
      <h2>Forward camera</h2>
      <img class="camera" id="front-camera" src="/camera.jpg" alt="Forward camera">
    </div>
    <div class="card">
      <h2>Backup camera and AprilTags</h2>
      <img class="camera" id="backup-camera" src="/backup-camera.jpg" alt="Backup camera">
      <div id="tags">No tags detected.</div>
    </div>
  </section>
  <section class="card">
    <h2>Manual drive</h2>
    <p>
      The embedded Donkeycar controller uses the same zero-on-release and
      zero-on-websocket-disconnect safety path as the motor service.
      <a id="manual-link" target="_blank" rel="noopener">Open controls in a new tab</a>
    </p>
    <iframe id="manual-controller" title="Manual drive controller"></iframe>
  </section>
  <section class="card">
    <h2>Robot state</h2>
    <pre id="state"></pre>
  </section>
</main>
<script>
const manualUrl = `${location.protocol}//${location.hostname}:{{ manual_port }}/`;
document.getElementById("manual-controller").src = manualUrl;
document.getElementById("manual-link").href = manualUrl;

function refreshImage(id, path) {
  document.getElementById(id).src = `${path}?t=${Date.now()}`;
}

function renderTags(tags) {
  const target = document.getElementById("tags");
  if (!tags || tags.length === 0) {
    target.textContent = "No tags detected.";
    return;
  }
  target.innerHTML = tags.map(tag => {
    const distance = tag.distance_m === null ? "range uncalibrated" : `${tag.distance_m.toFixed(2)} m`;
    return `<span class="tag">ID ${tag.id}, ${tag.horizontal_position}, ` +
      `${tag.bearing_degrees.toFixed(1)}°, ${distance}</span>`;
  }).join("");
}

async function refresh() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error(`state HTTP ${response.status}`);
    const state = await response.json();
    document.getElementById("state").textContent = JSON.stringify(state, null, 2);
    const healthy = state.esp32_connected && (!state.faults || state.faults.length === 0);
    const connection = document.getElementById("connection");
    connection.className = healthy ? "ok" : "bad";
    connection.textContent = healthy ? "ESP32 connected, drive path healthy" : "Drive path not ready";
    renderTags(state.apriltags);
  } catch (error) {
    const connection = document.getElementById("connection");
    connection.className = "bad";
    connection.textContent = `Dashboard state error: ${error}`;
  }
  refreshImage("front-camera", "/camera.jpg");
  refreshImage("backup-camera", "/backup-camera.jpg");
}
refresh();
setInterval(refresh, 500);
</script>
</body>
</html>
"""


class DashboardServer:
    def __init__(
        self,
        state: RobotState,
        host: str,
        port: int,
        manual_port: int = 8887,
    ) -> None:
        self._state = state
        self._host = host
        self._port = port
        self._manual_port = int(manual_port)
        self._latest_jpeg: bytes | None = None
        self._latest_backup_jpeg: bytes | None = None
        self._image_lock = threading.Lock()
        self._app = Flask(__name__)
        self._thread: threading.Thread | None = None
        self._configure_routes()

    def _configure_routes(self) -> None:
        @self._app.get("/")
        def index() -> str:
            return render_template_string(
                DASHBOARD_HTML,
                manual_port=self._manual_port,
            )

        @self._app.get("/healthz")
        def healthz() -> Any:
            snapshot = self._state.snapshot()
            return jsonify(
                {
                    "dashboard": "ok",
                    "esp32_connected": snapshot["esp32_connected"],
                    "backup_camera_connected": snapshot["backup_camera"]["connected"],
                }
            )

        @self._app.get("/api/state")
        def api_state() -> Any:
            return jsonify(self._state.snapshot())

        @self._app.get("/camera.jpg")
        def camera() -> Response:
            with self._image_lock:
                image = self._latest_jpeg
            if image is None:
                return Response(status=204)
            return Response(image, mimetype="image/jpeg")

        @self._app.get("/backup-camera.jpg")
        def backup_camera() -> Response:
            with self._image_lock:
                image = self._latest_backup_jpeg
            if image is None:
                return Response(status=204)
            return Response(image, mimetype="image/jpeg")

    def update_camera(self, jpeg: bytes | None) -> None:
        if jpeg:
            with self._image_lock:
                self._latest_jpeg = jpeg

    def update_backup_camera(self, jpeg: bytes | None) -> None:
        if jpeg:
            with self._image_lock:
                self._latest_backup_jpeg = jpeg

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=lambda: self._app.run(
                host=self._host,
                port=self._port,
                threaded=True,
                use_reloader=False,
            ),
            daemon=True,
        )
        self._thread.start()

    def shutdown(self) -> None:
        return

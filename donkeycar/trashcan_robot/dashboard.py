from __future__ import annotations

import threading
from typing import Any

from flask import Flask, Response, jsonify

from .state import RobotState


class DashboardServer:
    def __init__(self, state: RobotState, host: str, port: int) -> None:
        self._state = state
        self._host = host
        self._port = port
        self._latest_jpeg: bytes | None = None
        self._app = Flask(__name__)
        self._thread: threading.Thread | None = None
        self._configure_routes()

    def _configure_routes(self) -> None:
        @self._app.get("/")
        def index() -> str:
            return """<!doctype html><html><head><title>Trashcan Robot</title>
<style>body{font-family:sans-serif;max-width:960px;margin:20px auto;background:#111;color:#eee}img{max-width:100%;border:1px solid #555}pre{background:#222;padding:16px}</style></head>
<body><h1>Trashcan Robot</h1><img id='camera' src='/camera.jpg'><pre id='state'></pre>
<script>setInterval(async()=>{document.getElementById('state').textContent=JSON.stringify(await (await fetch('/api/state')).json(),null,2);document.getElementById('camera').src='/camera.jpg?'+Date.now()},500)</script></body></html>"""

        @self._app.get("/api/state")
        def api_state() -> Any:
            return jsonify(self._state.snapshot())

        @self._app.get("/camera.jpg")
        def camera() -> Response:
            if self._latest_jpeg is None:
                return Response(status=204)
            return Response(self._latest_jpeg, mimetype="image/jpeg")

    def update_camera(self, jpeg: bytes | None) -> None:
        if jpeg:
            self._latest_jpeg = jpeg

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=lambda: self._app.run(host=self._host, port=self._port, threaded=True, use_reloader=False),
            daemon=True,
        )
        self._thread.start()

    def shutdown(self) -> None:
        return

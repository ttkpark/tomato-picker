"""외부 패키지 없이 stdlib http.server만으로 만든 실시간 인식 로그 뷰어(SSE).

PC 브라우저에서 http://<젯슨IP>:포트 로 접속하면 인식된 텍스트가
발화 즉시(수 초 지연) 스트리밍된다.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .log_hub import LogHub


def _page(has_video: bool) -> str:
    video_block = (
        '<img id="cam" src="/video" alt="카메라 영상">'
        if has_video else
        '<div id="novideo">카메라 미연결 — 영상 없음</div>'
    )
    return f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>토마토피커 대시보드</title>
<style>
  :root {{ color-scheme: light dark; }}
  body {{ font-family: -apple-system, "Malgun Gothic", sans-serif; margin: 0; padding: 1rem;
         background: Canvas; color: CanvasText; }}
  h1 {{ font-size: 1.1rem; opacity: 0.8; }}
  #count {{ font-size: 1.6rem; font-weight: 700; padding: 0.5rem 0.9rem; border-radius: 10px;
           background: color-mix(in srgb, #e74c3c 22%, Canvas); margin-bottom: 0.75rem;
           display: inline-block; }}
  #cam {{ width: 100%; max-width: 720px; border-radius: 10px; display: block;
          margin-bottom: 0.75rem; background: #000; }}
  #novideo {{ opacity: 0.6; margin-bottom: 0.75rem; }}
  #log {{ display: flex; flex-direction: column; gap: 0.4rem; }}
  .row {{ display: flex; gap: 0.75rem; padding: 0.5rem 0.75rem; border-radius: 8px;
         background: color-mix(in srgb, CanvasText 6%, Canvas); font-size: 0.95rem; }}
  .row.intent {{ background: color-mix(in srgb, #2ecc71 25%, Canvas); font-weight: 600; }}
  .row.count {{ background: color-mix(in srgb, #e74c3c 20%, Canvas); font-weight: 600; }}
  .row.heard {{ opacity: 0.55; font-style: italic; }}
  .row.error {{ background: color-mix(in srgb, #e74c3c 25%, Canvas); }}
  .ts {{ opacity: 0.55; white-space: nowrap; font-variant-numeric: tabular-nums; }}
  #status {{ font-size: 0.85rem; opacity: 0.6; margin-bottom: 0.75rem; }}
</style></head>
<body>
<h1>토마토피커 — 실시간 대시보드</h1>
<div id="count">🍅 토마토: —</div>
{video_block}
<div id="status">연결 중...</div>
<div id="log"></div>
<script>
  const log = document.getElementById('log');
  const status = document.getElementById('status');
  const countEl = document.getElementById('count');
  function addRow(ev) {{
    const row = document.createElement('div');
    row.className = 'row ' + (ev.kind || '');
    const ts = document.createElement('span');
    ts.className = 'ts';
    ts.textContent = ev.ts;
    const text = document.createElement('span');
    text.textContent = ev.text;
    row.append(ts, text);
    log.appendChild(row);
    while (log.children.length > 200) log.removeChild(log.firstChild);
    row.scrollIntoView({{block: 'end'}});
  }}
  function onEvent(ev) {{
    // 개수는 상단 배너만 갱신하고 로그 줄로는 안 쌓는다(도배 방지).
    if (ev.kind === 'count') {{ countEl.textContent = '🍅 토마토: ' + ev.count + '개'; return; }}
    addRow(ev);
  }}
  const src = new EventSource('/events');
  src.onopen = () => status.textContent = '연결됨';
  src.onerror = () => status.textContent = '연결 끊김 — 재연결 시도 중...';
  src.onmessage = (e) => onEvent(JSON.parse(e.data));
</script>
</body></html>"""


def _make_handler(log_hub: LogHub, vision=None) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:  # noqa: A002 - stdlib 시그니처
            pass  # 콘솔에 접속 로그 안 찍음(음성 인식 로그와 섞이면 지저분함)

        def do_GET(self) -> None:
            if self.path == "/":
                body = _page(vision is not None).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/video":
                if vision is None:
                    self.send_response(404)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        jpeg = vision.latest_jpeg()
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        time.sleep(0.1)  # ~10fps 상한
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return

            if self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                q, history = log_hub.subscribe()
                try:
                    for event in history:
                        self._write_event(event)
                    while True:
                        event = q.get()  # 새 이벤트 올 때까지 블로킹
                        self._write_event(event)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    log_hub.unsubscribe(q)
                return

            self.send_response(404)
            self.end_headers()

        def _write_event(self, event: dict) -> None:
            payload = f"data: {json.dumps(event, ensure_ascii=False)}\n\n"
            self.wfile.write(payload.encode("utf-8"))
            self.wfile.flush()

    return Handler


def start_log_server(log_hub: LogHub, port: int, vision=None) -> ThreadingHTTPServer:
    """백그라운드 스레드에서 서버를 띄우고 서버 객체를 반환(종료 시 shutdown() 호출용).

    vision(VisionStreamer)을 넘기면 /video MJPEG 엔드포인트가 활성화된다.
    """
    server = ThreadingHTTPServer(("0.0.0.0", port), _make_handler(log_hub, vision))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server

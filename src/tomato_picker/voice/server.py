"""외부 패키지 없이 stdlib http.server만으로 만든 실시간 인식 로그 뷰어(SSE).

PC 브라우저에서 http://<젯슨IP>:포트 로 접속하면 인식된 텍스트가
발화 즉시(수 초 지연) 스트리밍된다.
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .log_hub import LogHub

_PAGE = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<title>토마토피커 음성 인식 로그</title>
<style>
  :root { color-scheme: light dark; }
  body { font-family: -apple-system, "Malgun Gothic", sans-serif; margin: 0; padding: 1rem;
         background: Canvas; color: CanvasText; }
  h1 { font-size: 1.1rem; opacity: 0.8; }
  #log { display: flex; flex-direction: column; gap: 0.4rem; }
  .row { display: flex; gap: 0.75rem; padding: 0.5rem 0.75rem; border-radius: 8px;
         background: color-mix(in srgb, CanvasText 6%, Canvas); font-size: 0.95rem; }
  .row.intent { background: color-mix(in srgb, #2ecc71 25%, Canvas); font-weight: 600; }
  .row.heard { opacity: 0.55; font-style: italic; }
  .ts { opacity: 0.55; white-space: nowrap; font-variant-numeric: tabular-nums; }
  #status { font-size: 0.85rem; opacity: 0.6; margin-bottom: 0.75rem; }
</style></head>
<body>
<h1>토마토피커 — 실시간 음성 인식 로그</h1>
<div id="status">연결 중...</div>
<div id="log"></div>
<script>
  const log = document.getElementById('log');
  const status = document.getElementById('status');
  function addRow(ev) {
    const row = document.createElement('div');
    row.className = 'row ' + (ev.kind || '');
    const ts = document.createElement('span');
    ts.className = 'ts';
    ts.textContent = ev.ts;
    const text = document.createElement('span');
    text.textContent = ev.text;
    row.append(ts, text);
    log.appendChild(row);
    row.scrollIntoView({block: 'end'});
  }
  const src = new EventSource('/events');
  src.onopen = () => status.textContent = '연결됨 — 듣는 중';
  src.onerror = () => status.textContent = '연결 끊김 — 재연결 시도 중...';
  src.onmessage = (e) => addRow(JSON.parse(e.data));
</script>
</body></html>"""


def _make_handler(log_hub: LogHub) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:  # noqa: A002 - stdlib 시그니처
            pass  # 콘솔에 접속 로그 안 찍음(음성 인식 로그와 섞이면 지저분함)

        def do_GET(self) -> None:
            if self.path == "/":
                body = _PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
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


def start_log_server(log_hub: LogHub, port: int) -> ThreadingHTTPServer:
    """백그라운드 스레드에서 서버를 띄우고 서버 객체를 반환(종료 시 shutdown() 호출용)."""
    server = ThreadingHTTPServer(("0.0.0.0", port), _make_handler(log_hub))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server

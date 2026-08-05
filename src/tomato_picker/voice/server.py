"""외부 패키지 없이 stdlib http.server만으로 만든 실시간 인식 로그 뷰어(SSE).

PC 브라우저에서 http://<젯슨IP>:포트 로 접속하면 인식된 텍스트가
발화 즉시(수 초 지연) 스트리밍된다.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from ..config import BASE_DRIVE_SPEED
from .log_hub import LogHub

# 하드웨어가 하나도 없어도 대시보드는 떠야 한다(부스 데모에서 화면이 검은 것보다
# "장비 미연결"이라고 떠 있는 게 낫다). 그래서 이 모듈의 어떤 것도 하드웨어
# 객체를 요구하지 않고, 없는 건 페이지에 상태로 표시만 한다.


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
  html, body {{ height: 100%; }}
  body {{ font-family: -apple-system, "Malgun Gothic", sans-serif; margin: 0; padding: 1rem;
         box-sizing: border-box; height: 100vh; display: flex; flex-direction: column;
         background: Canvas; color: CanvasText; }}
  /* 상단(영상·개수)은 고정, 로그만 아래에서 스크롤 */
  #top {{ flex: 0 0 auto; }}
  h1 {{ font-size: 1.1rem; opacity: 0.8; margin: 0 0 0.5rem; }}
  #count {{ font-size: 1.6rem; font-weight: 700; padding: 0.5rem 0.9rem; border-radius: 10px;
           background: color-mix(in srgb, #2ecc71 22%, Canvas); margin-bottom: 0.4rem;
           display: inline-block; }}
  #positions {{ font-size: 0.95rem; opacity: 0.8; margin-bottom: 0.75rem;
               font-variant-numeric: tabular-nums; }}
  #cam {{ width: 100%; max-width: 720px; max-height: 50vh; object-fit: contain;
          border-radius: 10px; display: block; margin-bottom: 0.5rem; background: #000; }}
  #novideo {{ opacity: 0.7; margin-bottom: 0.75rem; padding: 1.5rem; text-align: center;
             border: 1px dashed color-mix(in srgb, CanvasText 30%, Canvas); border-radius: 10px;
             max-width: 720px; }}
  #hw {{ font-size: 0.9rem; margin-bottom: 0.6rem; display: flex; flex-wrap: wrap; gap: 0.4rem; }}
  #hw span {{ padding: 0.15rem 0.5rem; border-radius: 999px;
             background: color-mix(in srgb, CanvasText 8%, Canvas); }}
  #hw span.ok {{ background: color-mix(in srgb, #2ecc71 25%, Canvas); }}
  #hw span.down {{ background: color-mix(in srgb, #e74c3c 25%, Canvas); }}
  #log {{ flex: 1 1 auto; min-height: 0; overflow-y: auto;
          display: flex; flex-direction: column; gap: 0.4rem; }}
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
<div id="top">
<h1>토마토피커 — 실시간 대시보드 &nbsp;<a href="/control">🎮 수동 조작</a></h1>
<div id="hw"><span>장비 상태 확인 중...</span></div>
<div id="count">🍅 공중 토마토: —</div>
<div id="positions">위치: —</div>
{video_block}
<div id="status">연결 중...</div>
</div>
<div id="log"></div>
<script>
  const log = document.getElementById('log');
  const status = document.getElementById('status');
  const countEl = document.getElementById('count');
  const posEl = document.getElementById('positions');
  const hwEl = document.getElementById('hw');
  // 하드웨어 상태 배지 — {{팔:'ok'|'down', ...}}. 아무것도 안 붙어 있어도
  // 페이지는 그대로 뜨고 여기만 빨갛게 표시된다.
  function renderHw(items) {{
    hwEl.textContent = '';
    for (const [name, state] of Object.entries(items || {{}})) {{
      const el = document.createElement('span');
      el.className = state === 'ok' ? 'ok' : 'down';
      el.textContent = (state === 'ok' ? '● ' : '○ ') + name;
      hwEl.append(el);
    }}
  }}
  function addRow(ev) {{
    const row = document.createElement('div');
    row.className = 'row ' + (ev.kind || '');
    const ts = document.createElement('span');
    ts.className = 'ts';
    ts.textContent = ev.ts;
    const text = document.createElement('span');
    text.textContent = ev.text;
    row.append(ts, text);
    log.insertBefore(row, log.firstChild);  // 최신이 맨 위로
    while (log.children.length > 200) log.removeChild(log.lastChild);
  }}
  function onEvent(ev) {{
    if (ev.kind === 'hw') {{ renderHw(ev.items); return; }}
    // 개수·위치는 상단 배너만 갱신하고 로그 줄로는 안 쌓는다(도배 방지).
    if (ev.kind === 'count') {{
      countEl.textContent = '🍅 공중 토마토: ' + ev.count + '개';
      const ps = (ev.positions || []).map(p => '(' + p[0] + ',' + p[1] + ')').join(', ') || '—';
      posEl.textContent = '위치: ' + ps + '   ·   낙과: ' + (ev.fallen || 0) + '개';
      return;
    }}
    addRow(ev);
  }}
  const src = new EventSource('/events');
  src.onopen = () => status.textContent = '연결됨';
  src.onerror = () => status.textContent = '연결 끊김 — 재연결 시도 중...';
  src.onmessage = (e) => onEvent(JSON.parse(e.data));
</script>
</body></html>"""


def _control_page(preset_ids: list[int], default_speed: int) -> str:
    """수동 조작 화면 — 음성 없이 브라우저에서 바퀴/팔을 직접 움직인다.

    데모 현장에서 음성이 안 먹히거나(소음·마이크 문제) 자세를 잡아둬야 할 때
    쓰는 백업 조작반. 명령은 전부 POST /cmd 하나로 보낸다.
    """
    preset_buttons = "".join(
        f'<button onclick="cmd({{action:\'arm_preset\',preset:{i}}})">프리셋 {i}</button>'
        for i in preset_ids
    ) or '<span class="dim">저장된 프리셋 없음</span>'
    return f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>토마토피커 수동 조작</title>
<style>
  :root {{ color-scheme: light dark; }}
  body {{ font-family: -apple-system, "Malgun Gothic", sans-serif; margin: 0; padding: 1rem;
         background: Canvas; color: CanvasText; }}
  h1 {{ font-size: 1.1rem; opacity: 0.8; margin: 0 0 0.75rem; }}
  h2 {{ font-size: 0.95rem; opacity: 0.7; margin: 1.25rem 0 0.5rem; }}
  a {{ color: inherit; }}
  button {{ font: inherit; padding: 0.9rem 0.6rem; border-radius: 10px; cursor: pointer;
           border: 1px solid color-mix(in srgb, CanvasText 25%, Canvas);
           background: color-mix(in srgb, CanvasText 8%, Canvas); color: inherit; }}
  button:active {{ background: color-mix(in srgb, #2ecc71 35%, Canvas); }}
  button.stop {{ background: color-mix(in srgb, #e74c3c 30%, Canvas); font-weight: 700; }}
  /* 메카넘 방향키 — 십자 배치 그대로가 제일 직관적 */
  #pad {{ display: grid; grid-template-columns: repeat(3, minmax(84px, 110px));
         gap: 0.5rem; }}
  #arm {{ display: flex; flex-wrap: wrap; gap: 0.5rem; }}
  .dim {{ opacity: 0.6; }}
  #keys {{ font-size: 0.9rem; margin-bottom: 0.75rem; padding: 0.6rem 0.8rem; border-radius: 8px;
          background: color-mix(in srgb, CanvasText 6%, Canvas); line-height: 1.9; }}
  kbd {{ font: inherit; font-size: 0.85em; padding: 0.1em 0.45em; border-radius: 5px;
        border: 1px solid color-mix(in srgb, CanvasText 30%, Canvas);
        background: color-mix(in srgb, CanvasText 10%, Canvas); }}
  #held {{ font-weight: 700; }}
  label {{ display: block; margin: 0.5rem 0; font-size: 0.9rem; }}
  input[type=range] {{ width: min(320px, 90vw); vertical-align: middle; }}
  #out {{ margin-top: 1rem; padding: 0.6rem 0.8rem; border-radius: 8px; min-height: 1.2em;
         background: color-mix(in srgb, CanvasText 6%, Canvas); font-size: 0.9rem;
         white-space: pre-wrap; }}
</style></head>
<body>
<h1>토마토피커 — 수동 조작 <span class="dim">(<a href="/">대시보드로</a>)</span></h1>

<div id="keys">⌨ <b>키보드</b>: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> 또는 방향키 = 이동(누르는 동안 계속) ·
<kbd>Q</kbd><kbd>E</kbd> = 회전 · <kbd>Space</kbd> = 정지 · <kbd>1</kbd>~<kbd>4</kbd> = 프리셋 · <kbd>R</kbd> = 힘 빼기
<span id="held" class="dim"></span></div>

<label>속도 <input type="range" id="speed" min="40" max="255" value="{default_speed}"
  oninput="speedOut.textContent=this.value"> <b id="speedOut">{default_speed}</b> / 255</label>
<label>동작 시간 <input type="range" id="secs" min="0.2" max="3" step="0.1" value="0.6"
  oninput="secsOut.textContent=this.value"> <b id="secsOut">0.6</b> 초</label>

<h2>바퀴 (메카넘)</h2>
<div id="pad">
  <button onclick="drive(1,-1,0)">↖ 좌전</button>
  <button onclick="drive(1,0,0)">▲ 전진</button>
  <button onclick="drive(1,1,0)">↗ 우전</button>
  <button onclick="drive(0,-1,0)">◀ 좌이동</button>
  <button class="stop" onclick="cmd({{action:'stop'}})">■ 정지</button>
  <button onclick="drive(0,1,0)">▶ 우이동</button>
  <button onclick="drive(-1,-1,0)">↙ 좌후</button>
  <button onclick="drive(-1,0,0)">▼ 후진</button>
  <button onclick="drive(-1,1,0)">↘ 우후</button>
  <button onclick="drive(0,0,-1)">↺ 좌회전</button>
  <span></span>
  <button onclick="drive(0,0,1)">↻ 우회전</button>
</div>

<h2>로봇팔</h2>
<div id="arm">
  {preset_buttons}
  <button onclick="cmd({{action:'arm_demo'}})">전체 시퀀스</button>
  <button onclick="cmd({{action:'arm_relax'}})">힘 빼기</button>
</div>

<div id="out">명령 대기 중</div>
<script>
  const out = document.getElementById('out');
  let busy = false;
  async function cmd(body) {{
    // 시리얼 명령은 블로킹이라 겹쳐 보내면 포트가 깨진다 — 한 번에 하나만.
    if (busy) {{ out.textContent = '이전 명령 실행 중...'; return; }}
    busy = true;
    out.textContent = '실행 중: ' + JSON.stringify(body);
    try {{
      const r = await fetch('/cmd', {{method: 'POST', body: JSON.stringify(body)}});
      const j = await r.json();
      out.textContent = (j.ok ? '완료: ' : '실패: ') + (j.detail || JSON.stringify(body));
    }} catch (e) {{
      out.textContent = '요청 실패: ' + e;
    }} finally {{ busy = false; }}
  }}
  function drive(vx, vy, w) {{
    const s = +document.getElementById('speed').value;
    cmd({{action:'drive', vx: vx*s, vy: vy*s, w: w*s,
         seconds: +document.getElementById('secs').value}});
  }}

  // --- 키보드 홀드 주행 ---
  // 누르고 있는 키 집합을 유지하고, 100ms마다 현재 합성 속도를 서버로 보낸다.
  // 서버(JetsonBase.hold)는 연결을 열어둔 채 50ms마다 펌웨어에 재전송하므로
  // 키를 누르고 있는 동안 끊김 없이 굴러간다. 키를 떼면 (0,0,0)이 가고,
  // 브라우저가 죽어도 서버가 0.5초 뒤 자동 정지한다(이중 안전).
  const KEY_VEC = {{
    w: [1,0,0], arrowup: [1,0,0], s: [-1,0,0], arrowdown: [-1,0,0],
    a: [0,-1,0], arrowleft: [0,-1,0], d: [0,1,0], arrowright: [0,1,0],
    q: [0,0,-1], e: [0,0,1],
  }};
  const held = new Set();
  const heldEl = document.getElementById('held');
  let lastSent = null;

  function heldVector() {{
    let v = [0,0,0];
    for (const k of held) {{
      const d = KEY_VEC[k];
      if (d) {{ v[0]+=d[0]; v[1]+=d[1]; v[2]+=d[2]; }}
    }}
    // 대각선에서 두 축이 겹쳐 2배가 되지 않게 -1..1로 자른다.
    return v.map(x => Math.max(-1, Math.min(1, x)));
  }}

  async function sendHold() {{
    const s = +document.getElementById('speed').value;
    const [vx, vy, w] = heldVector().map(x => Math.round(x * s));
    const key = vx + ',' + vy + ',' + w;
    // 정지 상태가 계속되면 굳이 반복해 보내지 않는다(서버가 알아서 멈춰 있음).
    if (key === '0,0,0' && lastSent === '0,0,0') return;
    lastSent = key;
    heldEl.textContent = (vx||vy||w) ? `  ▶ vx=${{vx}} vy=${{vy}} w=${{w}}` : '';
    try {{
      await fetch('/cmd', {{method:'POST', body: JSON.stringify({{action:'hold', vx, vy, w}})}});
    }} catch (e) {{ /* 한 번 실패해도 다음 주기에 다시 보낸다 */ }}
  }}
  setInterval(sendHold, 100);

  addEventListener('keydown', (e) => {{
    const k = e.key.toLowerCase();
    if (k === ' ') {{ held.clear(); e.preventDefault(); cmd({{action:'stop'}}); return; }}
    if (k === 'r') {{ cmd({{action:'arm_relax'}}); return; }}
    if ('1234'.includes(k)) {{ cmd({{action:'arm_preset', preset:+k}}); return; }}
    if (KEY_VEC[k]) {{ held.add(k); e.preventDefault(); }}
  }});
  addEventListener('keyup', (e) => {{ held.delete(e.key.toLowerCase()); }});
  // 탭을 벗어나면 키를 뗀 걸로 본다 — 안 그러면 keyup을 놓쳐 계속 굴러간다.
  addEventListener('blur', () => held.clear());
</script>
</body></html>"""


def _handle_command(body: dict, arm, base) -> tuple[bool, str]:
    """수동 조작 명령 하나를 실행. (성공여부, 사람이 읽을 설명)을 돌려준다.

    장비가 없으면(Mock/None) 에러 대신 그 사실을 문자열로 알려준다 —
    조작 화면은 하드웨어가 없어도 떠 있어야 하기 때문.
    """
    action = body.get("action")
    try:
        if action == "drive":
            if base is None:
                return False, "바퀴 미연결"
            seconds = max(0.1, min(3.0, float(body.get("seconds", 0.6))))
            vx, vy, w = (int(body.get(k, 0)) for k in ("vx", "vy", "w"))
            base.drive(seconds, vx=vx, vy=vy, w=w)
            return True, f"바퀴 vx={vx} vy={vy} w={w}, {seconds}초"
        if action == "hold":
            # 키보드 홀드 주행 — 브라우저가 키를 누르고 있는 동안 반복 호출한다.
            # 논블로킹이라 즉시 돌아온다(로그도 안 남긴다 — 초당 여러 번 온다).
            if base is None:
                return False, "바퀴 미연결"
            vx, vy, w = (int(body.get(k, 0)) for k in ("vx", "vy", "w"))
            base.hold(vx=vx, vy=vy, w=w)
            return True, f"홀드 vx={vx} vy={vy} w={w}"
        if action == "stop":
            if base is None:
                return False, "바퀴 미연결"
            base.hold(0, 0, 0)  # 홀드 세션이 돌고 있으면 그걸 먼저 멈춘다
            base.stop()
            return True, "정지"
        if action == "arm_preset":
            if arm is None:
                return False, "팔 미연결"
            preset = int(body.get("preset", 1))
            arm.play_preset(preset)
            return True, f"프리셋 {preset} 재생"
        if action == "arm_demo":
            if arm is None:
                return False, "팔 미연결"
            arm.demo_move()
            return True, "전체 시퀀스 재생"
        if action == "arm_relax":
            if arm is None:
                return False, "팔 미연결"
            arm.relax()
            return True, "팔 토크 해제"
    except Exception as exc:  # noqa: BLE001 - 조작 실패가 서버를 죽이면 안 됨
        return False, f"{action} 실패: {exc}"
    return False, f"알 수 없는 명령: {action}"


def _make_handler(log_hub: LogHub, vision=None, hardware: dict | None = None) -> type[BaseHTTPRequestHandler]:
    # hardware는 {"arm": ..., "base": ...} 형태의 **가변** 딕셔너리다. 서버를
    # 하드웨어보다 먼저 띄우는 게 원칙이라(voice_mode 참고) 핸들러 생성 시점엔
    # 아직 팔·바퀴가 없다 — 요청이 올 때마다 이 딕셔너리를 다시 읽는다.
    hw = hardware if hardware is not None else {}
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

            if self.path == "/control":
                presets = list(getattr(hw.get("arm"), "preset_ids", []) or [])
                body = _control_page(presets, BASE_DRIVE_SPEED).encode("utf-8")
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
                        # 비전 프로세스가 아직 프레임을 안 썼거나 죽어도 스트림
                        # 핸들러가 예외로 죽지 않게 — 대기했다 다시 본다.
                        try:
                            jpeg = vision.latest_jpeg()
                        except Exception:  # noqa: BLE001 - 영상 없어도 페이지는 살아야 함
                            jpeg = None
                        if not jpeg:
                            time.sleep(0.5)
                            continue
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

        def do_POST(self) -> None:
            if self.path != "/cmd":
                self.send_response(404)
                self.end_headers()
                return
            length = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                ok, detail = False, "JSON 파싱 실패"
            else:
                ok, detail = _handle_command(body, hw.get("arm"), hw.get("base"))
                # 홀드는 초당 여러 번 오므로 로그에 안 쌓는다(실패했을 때만 알림).
                if body.get("action") != "hold" or not ok:
                    log_hub.publish({
                        "ts": time.strftime("%H:%M:%S"),
                        "kind": "intent" if ok else "error",
                        "text": f"[수동조작] {detail}",
                    })
            payload = json.dumps({"ok": ok, "detail": detail}, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _write_event(self, event: dict) -> None:
            payload = f"data: {json.dumps(event, ensure_ascii=False)}\n\n"
            self.wfile.write(payload.encode("utf-8"))
            self.wfile.flush()

    return Handler


def start_log_server(
    log_hub: LogHub, port: int, vision=None, hardware: dict | None = None
) -> ThreadingHTTPServer:
    """백그라운드 스레드에서 서버를 띄우고 서버 객체를 반환(종료 시 shutdown() 호출용).

    vision(VisionStreamer)을 넘기면 /video MJPEG 엔드포인트가 활성화된다.
    hardware는 {"arm":..., "base":...} 가변 딕셔너리 — 서버를 먼저 띄우고
    나중에 채워도 /control과 POST /cmd가 그때부터 동작한다.
    """
    server = ThreadingHTTPServer(("0.0.0.0", port), _make_handler(log_hub, vision, hardware))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def _make_redirect_handler(target_port: int) -> type[BaseHTTPRequestHandler]:
    class RedirectHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:  # noqa: A002 - stdlib 시그니처
            pass

        def do_GET(self) -> None:
            # Host 헤더의 호스트명만 살리고 포트를 대시보드 포트로 바꾼다
            # (젯슨 IP가 DHCP로 바뀌어도 하드코딩 없이 따라간다).
            host = (self.headers.get("Host") or "").split(":")[0] or "localhost"
            self.send_response(302)
            self.send_header("Location", f"http://{host}:{target_port}/")
            self.send_header("Content-Length", "0")
            self.end_headers()

        do_HEAD = do_GET

    return RedirectHandler


def start_redirect_server(listen_port: int, target_port: int) -> ThreadingHTTPServer | None:
    """listen_port로 들어온 요청을 target_port 대시보드로 302 리다이렉트.

    데모 때 "192.168.0.8" 만 쳐도 대시보드가 열리게 하는 용도. 80처럼 1024
    미만 포트는 CAP_NET_BIND_SERVICE가 없으면 바인드가 거부되는데, 그때
    서비스 전체가 죽으면 안 되므로 실패는 로그만 남기고 None을 돌려준다.
    """
    if not listen_port or listen_port == target_port:
        return None
    try:
        server = ThreadingHTTPServer(("0.0.0.0", listen_port), _make_redirect_handler(target_port))
    except OSError as exc:
        print(f"[voice] {listen_port}번 포트 리다이렉트 실패({exc}) — {target_port}번은 정상 동작")
        return None
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[voice] {listen_port}번 → {target_port}번 리다이렉트 활성화")
    return server

"""외부 패키지 없이 stdlib http.server만으로 만든 실시간 인식 로그 뷰어(SSE).

PC 브라우저에서 http://<젯슨IP>:포트 로 접속하면 인식된 텍스트가
발화 즉시(수 초 지연) 스트리밍된다.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from ..config import BASE_DRIVE_SPEED, CAMERA_HEIGHT
from .log_hub import LogHub


def _status_payload(arm, base) -> dict:
    """조작 화면이 폴링하는 상태. 어떤 조회가 실패해도 페이지는 떠야 하므로
    항목마다 개별로 감싸고, 없는 장비는 None으로 내려보낸다."""
    def _safe(fn, fallback):
        try:
            return fn()
        except Exception as exc:  # noqa: BLE001 - 상태 조회 실패가 화면을 죽이면 안 됨
            return {**fallback, "error": str(exc)}

    arm_status = (
        _safe(arm.status, {"presets": {"slots": [], "anchors": []}})
        if arm is not None and hasattr(arm, "status")
        else {"presets": {"slots": [], "anchors": []}, "error": "팔 미연결"}
    )
    base_status = (
        _safe(base.link_stats, {"connected": False})
        if base is not None and hasattr(base, "link_stats")
        else {"connected": False, "error": "바퀴 미연결"}
    )
    return {"arm": arm_status, "base": base_status}

# 하드웨어가 하나도 없어도 대시보드는 떠야 한다(부스 데모에서 화면이 검은 것보다
# "장비 미연결"이라고 떠 있는 게 낫다). 그래서 이 모듈의 어떤 것도 하드웨어
# 객체를 요구하지 않고, 없는 건 페이지에 상태로 표시만 한다.


def _page(has_video: bool, has_floor: bool = False) -> str:
    video_block = (
        '<img id="cam" src="/video" alt="카메라 영상">'
        if has_video else
        '<div id="novideo">카메라 미연결 — 영상 없음</div>'
    )
    if has_floor:
        # 바닥 카메라(CSI, 라인트레이싱) — 무대 카메라 아래 작게. 스트림이 아직
        # 없으면 SharedFrameSource가 placeholder를 내보내므로 깨진 이미지는 없다.
        video_block += (
            '<div id="floorlabel">🛞 바닥 카메라 (라인트레이싱) '
            '<span id="line">라인 검출 대기 중...</span></div>'
            '<img id="floorcam" src="/video2" alt="바닥 카메라 영상">'
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
  #floorlabel {{ font-size: 0.85rem; opacity: 0.7; margin: 0.25rem 0 0.2rem; }}
  #line {{ padding: 0.1rem 0.5rem; border-radius: 999px; font-variant-numeric: tabular-nums;
          background: color-mix(in srgb, CanvasText 10%, Canvas); }}
  #line.ok {{ background: color-mix(in srgb, #2ecc71 30%, Canvas); }}
  #line.lost {{ background: color-mix(in srgb, #e74c3c 28%, Canvas); }}
  #line.mark {{ background: color-mix(in srgb, #f39c12 40%, Canvas); font-weight: 700; }}
  #floorcam {{ width: 100%; max-width: 400px; border-radius: 10px; display: block;
              margin-bottom: 0.5rem; background: #000; }}
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
  const lineEl = document.getElementById('line');
  function onEvent(ev) {{
    if (ev.kind === 'hw') {{ renderHw(ev.items); return; }}
    // 바닥 카메라 라인 검출 — 배지만 갈아끼우고 로그로는 안 쌓는다.
    if (ev.kind === 'line') {{
      if (!lineEl) return;
      if (ev.mark) {{
        lineEl.className = 'mark';
        lineEl.textContent = '■ 정지마크 감지';
      }} else if (ev.found) {{
        lineEl.className = 'ok';
        const off = Math.round(ev.offset_px);
        const dir = off === 0 ? '중앙' : (off > 0 ? '오른쪽' : '왼쪽');
        lineEl.textContent = '● 라인 ' + dir + ' ' + Math.abs(off) + 'px'
          + (ev.angle_deg === null || ev.angle_deg === undefined ? '' : '  ∠' + ev.angle_deg + '°');
      }} else {{
        lineEl.className = 'lost';
        lineEl.textContent = '○ 라인 없음';
      }}
      return;
    }}
    // 개수·위치는 상단 배너만 갱신하고 로그 줄로는 안 쌓는다(도배 방지).
    if (ev.kind === 'count') {{
      countEl.textContent = '🍅 공중 토마토: ' + ev.count + '개';
      const ps = (ev.positions || []).map(p => '(' + p[0] + ',' + p[1] + ')').join(', ') || '—';
      posEl.textContent = '위치: ' + ps + '   ·   낙과: ' + (ev.fallen || 0) + '개';
      return;
    }}
    addRow(ev);
  }}
  // MJPEG(<img> multipart)는 스트림이 끊겨도 **스스로 재연결하지 않고 마지막
  // 프레임에서 얼어붙는다**. tomato-voice를 재시작하면 열려 있던 탭은 기동 직후의
  // "vision starting..." placeholder를 계속 붙들고 있게 된다(2026-08-09 실사고 —
  // 서버는 멀쩡한데 화면만 죽어 보였다). 연결이 정상 종료되면 error 이벤트도 안
  // 오므로, **SSE 재연결**을 신호로 삼는다(EventSource는 알아서 다시 붙는다).
  const camImgs = ['cam', 'floorcam'].map(id => document.getElementById(id)).filter(Boolean);
  const camSrc = new Map(camImgs.map(img => [img, img.getAttribute('src')]));
  function reloadCams() {{
    // 캐시버스터가 없으면 같은 URL이라 브라우저가 재요청을 안 한다.
    for (const img of camImgs) img.src = camSrc.get(img) + '?t=' + Date.now();
  }}
  for (const img of camImgs) img.addEventListener('error', reloadCams);
  // 탭을 한참 접어뒀다 돌아오면 스트림이 끊겨 있는 경우가 많다.
  document.addEventListener('visibilitychange', () => {{ if (!document.hidden) reloadCams(); }});

  const src = new EventSource('/events');
  let sseOpened = false;  // 첫 open에서 다시 물면 방금 뜬 영상만 껌뻑인다.
  src.onopen = () => {{
    status.textContent = '연결됨';
    if (sseOpened) reloadCams();
    sseOpened = true;
  }};
  src.onerror = () => status.textContent = '연결 끊김 — 재연결 시도 중...';
  src.onmessage = (e) => onEvent(JSON.parse(e.data));
</script>
</body></html>"""


def _control_page(default_speed: int, frame_height: int) -> str:
    """수동 조작 화면 — 음성 없이 브라우저에서 바퀴/팔/프리셋을 직접 다룬다.

    데모 현장에서 음성이 안 먹히거나(소음·마이크 문제) 자세를 새로 잡아둬야 할 때
    쓰는 조작반. 명령은 전부 POST /cmd 하나로 보내고, 화면 상태는 GET /status를
    1초마다 폴링해 갱신한다(프리셋 목록·리더암·링크 품질이 서버 쪽 진실).

    브레이스 이스케이프 지옥을 피하려고 f-string이 아니라 치환 방식으로 만든다.
    """
    return (
        _CONTROL_HTML
        .replace("__SPEED__", str(default_speed))
        .replace("__FRAME_H__", str(frame_height))
    )


_CONTROL_HTML = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>토마토피커 수동 조작</title>
<style>
  :root { color-scheme: light dark; }
  body { font-family: -apple-system, "Malgun Gothic", sans-serif; margin: 0; padding: 1rem;
         background: Canvas; color: CanvasText; }
  h1 { font-size: 1.1rem; opacity: 0.8; margin: 0 0 0.75rem; }
  h2 { font-size: 0.95rem; opacity: 0.7; margin: 1.4rem 0 0.5rem; }
  a { color: inherit; }
  button { font: inherit; padding: 0.7rem 0.6rem; border-radius: 10px; cursor: pointer;
           border: 1px solid color-mix(in srgb, CanvasText 25%, Canvas);
           background: color-mix(in srgb, CanvasText 8%, Canvas); color: inherit; }
  button:active { background: color-mix(in srgb, #2ecc71 35%, Canvas); }
  button.stop { background: color-mix(in srgb, #e74c3c 30%, Canvas); font-weight: 700; }
  button.on { background: color-mix(in srgb, #2ecc71 40%, Canvas); font-weight: 700; }
  button.small { padding: 0.3rem 0.5rem; font-size: 0.85rem; border-radius: 7px; }
  #pad { display: grid; grid-template-columns: repeat(3, minmax(84px, 110px)); gap: 0.5rem; }
  .row-flex { display: flex; flex-wrap: wrap; gap: 0.5rem; align-items: center; }
  .dim { opacity: 0.6; }
  .card { padding: 0.6rem 0.8rem; border-radius: 10px;
          background: color-mix(in srgb, CanvasText 6%, Canvas); }
  #keys { font-size: 0.9rem; margin-bottom: 0.75rem; line-height: 1.9; }
  kbd { font: inherit; font-size: 0.85em; padding: 0.1em 0.45em; border-radius: 5px;
        border: 1px solid color-mix(in srgb, CanvasText 30%, Canvas);
        background: color-mix(in srgb, CanvasText 10%, Canvas); }
  #held { font-weight: 700; }
  label { display: block; margin: 0.5rem 0; font-size: 0.9rem; }
  input[type=range] { width: min(320px, 90vw); vertical-align: middle; }
  input[type=text], input[type=number] { font: inherit; padding: 0.35rem 0.5rem; border-radius: 7px;
        border: 1px solid color-mix(in srgb, CanvasText 25%, Canvas);
        background: Canvas; color: inherit; }
  select { font: inherit; padding: 0.35rem; border-radius: 7px; background: Canvas; color: inherit;
        border: 1px solid color-mix(in srgb, CanvasText 25%, Canvas); }
  /* 프리셋 0~9 그리드 */
  #presets { display: grid; gap: 0.5rem;
             grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); max-width: 900px; }
  .slot { border-radius: 10px; padding: 0.5rem 0.6rem; display: flex; flex-direction: column;
          gap: 0.35rem; border: 1px solid color-mix(in srgb, CanvasText 20%, Canvas);
          background: color-mix(in srgb, CanvasText 5%, Canvas); }
  .slot.empty { opacity: 0.55; border-style: dashed; }
  .slot .no { font-weight: 700; font-size: 1.05rem; }
  .slot .nm { font-size: 0.85rem; opacity: 0.75; min-height: 1.1em; }
  .tag { font-size: 0.75rem; padding: 0.1rem 0.45rem; border-radius: 999px;
         background: color-mix(in srgb, #3498db 35%, Canvas); }
  #mode { font-weight: 700; }
  #link { font-size: 0.85rem; font-variant-numeric: tabular-nums; }
  #out { margin-top: 1rem; padding: 0.6rem 0.8rem; border-radius: 8px; min-height: 1.2em;
         background: color-mix(in srgb, CanvasText 6%, Canvas); font-size: 0.9rem;
         white-space: pre-wrap; }
</style></head>
<body>
<h1>토마토피커 — 수동 조작 <span class="dim">(<a href="/">대시보드로</a>)</span></h1>

<div class="card" id="link">링크 상태 확인 중...</div>

<div class="card" id="keys">⌨ <b>키보드</b>: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> 또는 방향키 = 이동(누르는 동안 계속) ·
<kbd>Q</kbd><kbd>E</kbd> = 회전 · <kbd>Space</kbd> = 정지 · <kbd>0</kbd>~<kbd>9</kbd> = 프리셋(현재 모드로) · <kbd>R</kbd> = 힘 빼기
<span id="held" class="dim"></span></div>

<label>속도 <input type="range" id="speed" min="40" max="255" value="__SPEED__"
  oninput="speedOut.textContent=this.value"> <b id="speedOut">__SPEED__</b> / 255</label>
<label>동작 시간 <input type="range" id="secs" min="0.2" max="3" step="0.1" value="0.6"
  oninput="secsOut.textContent=this.value"> <b id="secsOut">0.6</b> 초</label>

<h2>바퀴 (메카넘)</h2>
<div id="pad">
  <button onclick="drive(1,-1,0)">↖ 좌전</button>
  <button onclick="drive(1,0,0)">▲ 전진</button>
  <button onclick="drive(1,1,0)">↗ 우전</button>
  <button onclick="drive(0,-1,0)">◀ 좌이동</button>
  <button class="stop" onclick="cmd({action:'stop'})">■ 정지</button>
  <button onclick="drive(0,1,0)">▶ 우이동</button>
  <button onclick="drive(-1,-1,0)">↙ 좌후</button>
  <button onclick="drive(-1,0,0)">▼ 후진</button>
  <button onclick="drive(-1,1,0)">↘ 우후</button>
  <button onclick="drive(0,0,-1)">↺ 좌회전</button>
  <span></span>
  <button onclick="drive(0,0,1)">↻ 우회전</button>
</div>

<h2>모터 튜닝 <span class="dim">— 소음 ↔ 속도 (재플래시 없이 즉시 반영)</span></h2>
<div class="card">
  <div class="row-flex" style="margin-bottom:0.5rem">
    <button onclick="preset(50,2000)">🔊 힘 최대 · 50Hz "우우웅"</button>
    <button onclick="preset(400,2400)">400Hz</button>
    <button onclick="preset(700,2600)">700Hz (균형)</button>
    <button onclick="preset(1100,2900)">1100Hz</button>
    <button onclick="preset(1526,3200)">🔇 1526Hz "찌잉" (상한)</button>
  </div>
  <label>PWM 주파수 <input type="range" id="tHz" min="24" max="1526" step="1" value="700"
    oninput="tHzOut.textContent=this.value"> <b id="tHzOut">700</b> Hz
    <span class="dim">낮을수록 저음·힘셈 / 높을수록 고음·힘약함</span></label>
  <label>듀티 상한 <input type="range" id="tPwm" min="800" max="4095" step="25" value="2600"
    oninput="tPwmOut.textContent=this.value"> <b id="tPwmOut">2600</b> / 4095
    <span class="dim">주파수 올려 잃은 속도를 여기서 되찾는다 (⚠ 전류도 같이 커짐)</span></label>
  <label>가속 <input type="range" id="tAcc" min="1" max="40" value="6"
    oninput="tAccOut.textContent=this.value"> <b id="tAccOut">6</b>
    <span class="dim">클수록 즉답 / 작을수록 인러시가 낮아 젯슨 리셋에 안전</span></label>
  <div class="row-flex">
    <button onclick="applyTune()">적용</button>
    <span id="tNow" class="dim"></span>
  </div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    PCA9685는 <b>1526Hz가 물리적 상한</b>이라 가청대역 위로는 못 올린다 — "무음"은 선택지가
    아니고 저역 울림과 고역 휘파람 중 하나를 고르는 것. 마음에 드는 값을 찾으면
    <code>config.py</code>의 <code>BASE_PWM_HZ</code> / <code>BASE_MAX_PWM</code>에 적어두면
    다음 부팅부터 기본값이 된다. 젯슨이 리셋되면 듀티 상한부터 내릴 것.
  </p>
</div>

<h2>리더암 (자세 만들기)</h2>
<div class="card">
  <div class="row-flex">
    <select id="leaderPort"><option value="">포트 자동선택</option></select>
    <button id="btnLeader" onclick="toggleLeader()">리더암 연결</button>
    <button id="btnMirror" onclick="toggleMirror()">미러링 시작</button>
    <span id="leaderInfo" class="dim"></span>
  </div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    미러링 ON → 리더를 손으로 움직이면 팔로워가 따라온다. 원하는 자세에서
    아래 <b>등록 모드</b>로 바꾸고 슬롯을 누르면 그 자세가 저장된다.
    리더 없이도 <b>힘 빼기</b> 후 손으로 자세를 잡아 저장할 수 있다.
  </p>
</div>

<h2>프리셋 슬롯 0~9</h2>
<div class="card" style="margin-bottom:0.6rem">
  <div class="row-flex">
    <span>모드:</span>
    <button id="btnPlay" onclick="setMode('play')">▶ 재생</button>
    <button id="btnSave" onclick="setMode('save')">● 등록(저장)</button>
    <span id="mode" class="dim"></span>
  </div>
</div>
<div id="presets"></div>

<h2>시퀀스 · 힘 빼기</h2>
<div class="row-flex">
  <input type="text" id="seq" value="1,2,3,4" size="12">
  <button onclick="runSeq()">순서대로 재생</button>
  <button onclick="cmd({action:'arm_demo'})">전체 시퀀스(1→4)</button>
  <button onclick="cmd({action:'arm_relax'})">힘 빼기</button>
</div>

<h2>높이 보간 — 앵커 사이 자세 계산</h2>
<div class="card">
  <div id="anchorList" class="dim" style="margin-bottom:0.5rem">앵커 없음</div>
  <label>목표 높이 y <input type="range" id="blendY" min="0" max="__FRAME_H__" value="360"
    oninput="blendOut.textContent=this.value"> <b id="blendOut">360</b> px</label>
  <div class="row-flex">
    <button onclick="cmd({action:'arm_blend', y:+document.getElementById('blendY').value})">이 높이로 이동</button>
    <button onclick="cmd({action:'arm_blend', from_vision:true})">🍅 감지된 토마토 높이로 이동</button>
  </div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    y는 카메라 화면의 세로 픽셀(위=0, 아래=__FRAME_H__). 앵커를 2개 이상 등록하면
    그 사이는 관절값 선형보간으로 채워지고, 바깥은 <b>외삽하지 않고</b> 끝 앵커로 고정된다.
  </p>
</div>

<div id="out">명령 대기 중</div>
<script>
  const out = document.getElementById('out');
  let busy = false;
  let mode = 'play';
  let state = null;
  let tuneInit = false;   // 튜닝 슬라이더를 서버값으로 맞추는 건 첫 로드 1회만

  async function cmd(body) {
    // 시리얼 명령은 블로킹이라 겹쳐 보내면 포트가 깨진다 — 한 번에 하나만.
    if (busy) { out.textContent = '이전 명령 실행 중...'; return; }
    busy = true;
    out.textContent = '실행 중: ' + JSON.stringify(body);
    try {
      const r = await fetch('/cmd', {method: 'POST', body: JSON.stringify(body)});
      const j = await r.json();
      out.textContent = (j.ok ? '완료: ' : '실패: ') + (j.detail || JSON.stringify(body));
    } catch (e) {
      out.textContent = '요청 실패: ' + e;
    } finally { busy = false; refresh(); }
  }

  function drive(vx, vy, w) {
    const s = +document.getElementById('speed').value;
    cmd({action:'drive', vx: vx*s, vy: vy*s, w: w*s,
         seconds: +document.getElementById('secs').value});
  }

  // --- 모드(재생/등록) ---
  function setMode(m) {
    mode = m;
    document.getElementById('btnPlay').className = m === 'play' ? 'on' : '';
    document.getElementById('btnSave').className = m === 'save' ? 'on' : '';
    document.getElementById('mode').textContent =
      m === 'play' ? '슬롯을 누르면 그 자세로 이동합니다.'
                   : '슬롯을 누르면 지금 팔 자세를 그 슬롯에 덮어씁니다.';
    render();
  }

  function slotClick(n) {
    if (mode === 'save') {
      if (!confirm(n + '번 슬롯에 지금 자세를 저장할까요?' +
                   (slotFilled(n) ? '\\n(기존 자세는 덮어써집니다)' : ''))) return;
      cmd({action:'arm_save', slot:n});
    } else {
      cmd({action:'arm_preset', preset:n});
    }
  }
  function slotFilled(n) {
    const s = state && state.arm && state.arm.presets.slots.find(x => x.slot === n);
    return !!(s && s.filled);
  }

  function rename(n) {
    const cur = (state.arm.presets.slots.find(x => x.slot === n) || {}).name || '';
    const name = prompt(n + '번 슬롯 이름 (예: 접근, 집기)', cur);
    if (name === null) return;
    cmd({action:'arm_rename', slot:n, name});
  }
  function anchor(n) {
    const s = state.arm.presets.slots.find(x => x.slot === n) || {};
    if (s.anchor) { if (confirm(n + '번의 높이 앵커를 해제할까요?')) cmd({action:'arm_anchor_clear', slot:n}); return; }
    const label = prompt('앵커 라벨 (상 / 중 / 하 등)', '중');
    if (!label) return;
    const y = prompt('이 자세가 가리키는 화면 높이 y (0=맨위, __FRAME_H__=맨아래)', '360');
    if (y === null) return;
    cmd({action:'arm_anchor', slot:n, label, y:+y});
  }
  function del(n) {
    if (!confirm(n + '번 슬롯을 비울까요?')) return;
    cmd({action:'arm_delete', slot:n});
  }
  function runSeq() {
    const list = document.getElementById('seq').value.split(',')
      .map(s => parseInt(s.trim(), 10)).filter(n => !isNaN(n));
    if (!list.length) { out.textContent = '슬롯 번호를 쉼표로 입력하세요 (예: 1,2,3,4)'; return; }
    cmd({action:'arm_sequence', presets:list});
  }

  // --- 모터 튜닝 ---
  function preset(hz, pwm) {
    document.getElementById('tHz').value = hz;  tHzOut.textContent = hz;
    document.getElementById('tPwm').value = pwm; tPwmOut.textContent = pwm;
    applyTune();
  }
  function applyTune() {
    cmd({action:'base_tune',
         hz:      +document.getElementById('tHz').value,
         max_pwm: +document.getElementById('tPwm').value,
         accel:   +document.getElementById('tAcc').value});
  }

  // --- 리더암 ---
  function toggleLeader() {
    if (state && state.arm && state.arm.leader_connected) cmd({action:'leader_disconnect'});
    else cmd({action:'leader_connect', port: document.getElementById('leaderPort').value || null});
  }
  function toggleMirror() {
    const on = state && state.arm && state.arm.mirroring;
    cmd({action:'mirror', on: !on});
  }

  // --- 상태 렌더링 ---
  function render() {
    if (!state) return;
    const arm = state.arm || {};
    const base = state.base || {};

    // 링크 상태
    const ok = base.connected;
    const bits = [(ok ? '● 바퀴 링크 정상' : '○ 바퀴 링크 끊김')];
    if (base.port) bits.push(base.port);
    if (base.hb_age !== null && base.hb_age !== undefined) bits.push('hb ' + base.hb_age + 's 전');
    bits.push('수신 ' + (base.fw_rx || 0));
    if (base.fw_bad) bits.push('⚠ 깨진프레임 ' + base.fw_bad);
    if (base.nak) bits.push('⚠ nak ' + base.nak);
    // 하트비트가 끊겨 보드를 되살린 횟수 — 0이 아니면 전원/펌웨어를 의심할 것.
    if (base.hb_resets) bits.push('⚠ 링크복구 ' + base.hb_resets + '회');
    if (base.error) bits.push(base.error);
    const link = document.getElementById('link');
    link.textContent = bits.join('  ·  ');
    link.style.background = ok ? 'color-mix(in srgb, #2ecc71 18%, Canvas)'
                               : 'color-mix(in srgb, #e74c3c 18%, Canvas)';

    // 모터 튜닝 — 현재 적용값 표시. 첫 로드 때만 슬라이더를 서버값으로 맞춘다
    // (그 뒤엔 사용자가 끌고 있는 값을 폴링이 덮어쓰면 안 되므로 건드리지 않는다).
    const t = base.tuning;
    if (t) {
      // acks = 펌웨어가 실제로 되돌려준 "ok F/P/R ..." — 값이 먹혔다는 증거.
      // 이게 안 뜨면 요청만 갔고 보드는 예전 값 그대로다.
      document.getElementById('tNow').textContent =
        '요청값: ' + t.hz + 'Hz · 듀티 ' + t.max_pwm + '/4095 · 가속 ' + t.accel + ' / 감속 ' + t.decel
        + (base.acks && base.acks.length ? '   ← 보드 확인: ' + base.acks.join(' | ') : '   ← 보드 확인 없음');
      if (!tuneInit) {
        tuneInit = true;
        for (const [id, v] of [['tHz', t.hz], ['tPwm', t.max_pwm], ['tAcc', t.accel]]) {
          document.getElementById(id).value = v;
          document.getElementById(id + 'Out').textContent = v;
        }
      }
    }

    // 리더암
    const bl = document.getElementById('btnLeader');
    bl.textContent = arm.leader_connected ? '리더암 연결 해제' : '리더암 연결';
    bl.className = arm.leader_connected ? 'on' : '';
    const bm = document.getElementById('btnMirror');
    bm.textContent = arm.mirroring ? '미러링 중지' : '미러링 시작';
    bm.className = arm.mirroring ? 'on' : '';
    document.getElementById('leaderInfo').textContent =
      (arm.leader_port ? '리더 ' + arm.leader_port + ' · ' : '') +
      (arm.follower_port ? '팔로워 ' + arm.follower_port : '') +
      (arm.mirror_error ? '  ⚠ ' + arm.mirror_error : '');
    const sel = document.getElementById('leaderPort');
    const want = ['', ...(arm.leader_candidates || [])].join('|');
    if (sel.dataset.opts !== want) {
      sel.dataset.opts = want;
      const keep = sel.value;
      sel.innerHTML = '<option value="">포트 자동선택</option>';
      for (const p of (arm.leader_candidates || [])) {
        const o = document.createElement('option'); o.value = p; o.textContent = p; sel.append(o);
      }
      sel.value = keep;
    }

    // 프리셋 슬롯
    const grid = document.getElementById('presets');
    grid.textContent = '';
    for (const s of ((arm.presets || {}).slots || [])) {
      const el = document.createElement('div');
      el.className = 'slot' + (s.filled ? '' : ' empty');
      const head = document.createElement('div');
      head.className = 'row-flex';
      const no = document.createElement('span');
      no.className = 'no'; no.textContent = s.slot;
      head.append(no);
      if (s.anchor) {
        const t = document.createElement('span');
        t.className = 'tag';
        t.textContent = s.anchor.label + ' y=' + Math.round(s.anchor.y);
        head.append(t);
      }
      const nm = document.createElement('div');
      nm.className = 'nm'; nm.textContent = s.name || (s.filled ? '' : '비어 있음');
      const main = document.createElement('button');
      main.textContent = mode === 'save' ? '● 여기에 저장' : '▶ 재생';
      if (mode === 'play' && !s.filled) main.disabled = true;
      main.onclick = () => slotClick(s.slot);
      const tools = document.createElement('div');
      tools.className = 'row-flex';
      for (const [label, fn] of [['이름', rename], ['앵커', anchor], ['비우기', del]]) {
        const b = document.createElement('button');
        b.className = 'small'; b.textContent = label;
        b.disabled = !s.filled && label !== '이름';
        b.onclick = () => fn(s.slot);
        tools.append(b);
      }
      el.append(head, nm, main, tools);
      grid.append(el);
    }

    // 앵커 목록
    const anchors = (arm.presets || {}).anchors || [];
    document.getElementById('anchorList').textContent = anchors.length
      ? '앵커(위→아래): ' + anchors.map(a => a.label + '=슬롯' + a.slot + '(y' + Math.round(a.y) + ')').join('  →  ')
      : '앵커 없음 — 슬롯의 [앵커] 버튼으로 상/중/하를 지정하세요.';
  }

  async function refresh() {
    try {
      const r = await fetch('/status');
      state = await r.json();
      render();
    } catch (e) { /* 다음 주기에 다시 */ }
  }
  setInterval(refresh, 1000);
  setMode('play');
  refresh();

  // --- 키보드 홀드 주행 ---
  // 누르고 있는 키 집합을 유지하고, 100ms마다 현재 합성 속도를 서버로 보낸다.
  // 서버(MotorLink)는 연결을 열어둔 채 20ms마다 펌웨어에 재전송하므로 키를
  // 누르고 있는 동안 끊김 없이 굴러간다. 키를 떼면 (0,0,0)이 가고, 브라우저가
  // 죽어도 서버가 0.5초 뒤 자동 정지한다(그마저 못 가면 펌웨어 데드맨).
  const KEY_VEC = {
    w: [1,0,0], arrowup: [1,0,0], s: [-1,0,0], arrowdown: [-1,0,0],
    a: [0,-1,0], arrowleft: [0,-1,0], d: [0,1,0], arrowright: [0,1,0],
    q: [0,0,-1], e: [0,0,1],
  };
  const held = new Set();
  const heldEl = document.getElementById('held');
  let lastSent = null;

  function heldVector() {
    let v = [0,0,0];
    for (const k of held) {
      const d = KEY_VEC[k];
      if (d) { v[0]+=d[0]; v[1]+=d[1]; v[2]+=d[2]; }
    }
    // 대각선에서 두 축이 겹쳐 2배가 되지 않게 -1..1로 자른다.
    return v.map(x => Math.max(-1, Math.min(1, x)));
  }

  async function sendHold() {
    const s = +document.getElementById('speed').value;
    const [vx, vy, w] = heldVector().map(x => Math.round(x * s));
    const key = vx + ',' + vy + ',' + w;
    // 정지 상태가 계속되면 굳이 반복해 보내지 않는다(서버가 알아서 멈춰 있음).
    if (key === '0,0,0' && lastSent === '0,0,0') return;
    lastSent = key;
    heldEl.textContent = (vx||vy||w) ? ('  ▶ vx=' + vx + ' vy=' + vy + ' w=' + w) : '';
    try {
      await fetch('/cmd', {method:'POST', body: JSON.stringify({action:'hold', vx, vy, w})});
    } catch (e) { /* 한 번 실패해도 다음 주기에 다시 보낸다 */ }
  }
  setInterval(sendHold, 100);

  addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    const k = e.key.toLowerCase();
    if (k === ' ') { held.clear(); e.preventDefault(); cmd({action:'stop'}); return; }
    if (k === 'r') { cmd({action:'arm_relax'}); return; }
    if ('0123456789'.includes(k)) { slotClick(+k); return; }
    if (KEY_VEC[k]) { held.add(k); e.preventDefault(); }
  });
  addEventListener('keyup', (e) => { held.delete(e.key.toLowerCase()); });
  // 탭을 벗어나면 키를 뗀 걸로 본다 — 안 그러면 keyup을 놓쳐 계속 굴러간다.
  addEventListener('blur', () => held.clear());
</script>
</body></html>"""

def _lowest_tomato_y(vision) -> float | None:
    """비전이 본 공중 토마토 중 **가장 아래(y가 큰) 것**의 y좌표.

    여러 개가 보일 때 무엇을 딸지 정하는 규칙 — 아래쪽부터 따는 게
    수확 순서로 자연스럽고(위 열매를 건드려 떨어뜨리지 않는다), 낙과선
    바로 위 열매가 팔 사거리에도 가장 가깝다.
    """
    if vision is None or not hasattr(vision, "latest_status"):
        return None
    positions = vision.latest_status().get("positions") or []
    ys = [float(p[1]) for p in positions if len(p) >= 2]
    return max(ys) if ys else None


def _handle_command(body: dict, arm, base, vision=None) -> tuple[bool, str]:
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
        if action == "base_tune":
            # PWM 주파수·듀티상한·가감속 실시간 변경(재플래시 불필요).
            if base is None or not hasattr(base, "tune"):
                return False, "바퀴 미연결"
            t = base.tune(
                hz=body.get("hz"), max_pwm=body.get("max_pwm"),
                accel=body.get("accel"), decel=body.get("decel"),
            )
            return True, (f"튜닝 적용: {t['hz']}Hz · 듀티상한 {t['max_pwm']}/4095"
                          f" · 가속 {t['accel']}/감속 {t['decel']}")
        if arm is None:
            # 아래 명령은 전부 팔이 필요하다 — 한 곳에서 걸러낸다.
            if str(action).startswith(("arm_", "leader_", "mirror")):
                return False, "팔 미연결"
        elif action == "arm_preset":
            preset = int(body.get("preset", 1))
            arm.play_preset(preset)
            return True, f"프리셋 {preset} 재생"
        elif action == "arm_sequence":
            presets = [int(p) for p in (body.get("presets") or [])]
            if not presets:
                return False, "재생할 슬롯이 비어 있습니다"
            arm.play_sequence(presets)
            return True, "시퀀스 재생: " + " → ".join(str(p) for p in presets)
        elif action == "arm_blend":
            # 높이 앵커 사이 자세를 계산해 재생. from_vision이면 비전이 본
            # 가장 아래 토마토의 y를 그대로 쓴다.
            if body.get("from_vision"):
                y = _lowest_tomato_y(vision)
                if y is None:
                    return False, "비전이 잡은 토마토가 없습니다"
                source = f"비전 y={y:.0f}"
            else:
                y = float(body.get("y", 0))
                source = f"수동 y={y:.0f}"
            desc = arm.play_blended(y)
            return True, f"높이 보간 재생 ({source} → {desc})"
        elif action == "arm_save":
            slot = int(body.get("slot", 0))
            name = body.get("name")
            pose = arm.save_preset(slot, name)
            joints = ", ".join(f"{k.split('.')[0]}={v:.0f}" for k, v in sorted(pose.items()))
            return True, f"슬롯 {slot} 저장: {joints}"
        elif action == "arm_rename":
            slot = int(body.get("slot", 0))
            arm.presets.set_name(slot, str(body.get("name", "")))
            return True, f"슬롯 {slot} 이름 변경"
        elif action == "arm_delete":
            slot = int(body.get("slot", 0))
            arm.delete_preset(slot)
            return True, f"슬롯 {slot} 비움"
        elif action == "arm_anchor":
            slot = int(body.get("slot", 0))
            label = str(body.get("label", "")) or "앵커"
            y = float(body.get("y", 0))
            arm.set_anchor(slot, label, y)
            return True, f"슬롯 {slot} = 높이앵커 '{label}' (y={y:.0f})"
        elif action == "arm_anchor_clear":
            slot = int(body.get("slot", 0))
            arm.clear_anchor(slot)
            return True, f"슬롯 {slot} 앵커 해제"
        elif action == "arm_demo":
            arm.demo_move()
            return True, "전체 시퀀스 재생"
        elif action == "arm_relax":
            arm.relax()
            return True, "팔 토크 해제"
        elif action == "leader_connect":
            port = arm.connect_leader(body.get("port") or None)
            return True, f"리더암 연결됨: {port}"
        elif action == "leader_disconnect":
            arm.disconnect_leader()
            return True, "리더암 연결 해제"
        elif action == "mirror":
            if body.get("on"):
                arm.start_mirror()
                return True, "미러링 시작 — 리더를 움직이면 팔로워가 따라옵니다"
            arm.stop_mirror()
            return True, "미러링 중지 (자세 유지)"
    except Exception as exc:  # noqa: BLE001 - 조작 실패가 서버를 죽이면 안 됨
        return False, f"{action} 실패: {exc}"
    return False, f"알 수 없는 명령: {action}"


def _make_handler(
    log_hub: LogHub, vision=None, hardware: dict | None = None, floor=None
) -> type[BaseHTTPRequestHandler]:
    # hardware는 {"arm": ..., "base": ...} 형태의 **가변** 딕셔너리다. 서버를
    # 하드웨어보다 먼저 띄우는 게 원칙이라(voice_mode 참고) 핸들러 생성 시점엔
    # 아직 팔·바퀴가 없다 — 요청이 올 때마다 이 딕셔너리를 다시 읽는다.
    hw = hardware if hardware is not None else {}
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:  # noqa: A002 - stdlib 시그니처
            pass  # 콘솔에 접속 로그 안 찍음(음성 인식 로그와 섞이면 지저분함)

        def do_GET(self) -> None:
            if self.path == "/":
                body = _page(vision is not None, floor is not None).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/status":
                # 조작 화면이 1초마다 폴링하는 상태 스냅샷. 하드웨어가 없거나
                # 상태 조회가 실패해도 화면은 떠 있어야 하므로 전부 감싼다.
                self._write_json(_status_payload(hw.get("arm"), hw.get("base")))
                return

            if self.path == "/control":
                body = _control_page(BASE_DRIVE_SPEED, CAMERA_HEIGHT).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/video":
                self._stream_mjpeg(vision)
                return

            if self.path == "/video2":
                # 바닥 카메라(CSI) — line-cam.service가 /dev/shm에 쓰는 프레임.
                self._stream_mjpeg(floor)
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
                ok, detail = _handle_command(body, hw.get("arm"), hw.get("base"), vision)
                # 홀드는 초당 여러 번 오므로 로그에 안 쌓는다(실패했을 때만 알림).
                if body.get("action") != "hold" or not ok:
                    log_hub.publish({
                        "ts": time.strftime("%H:%M:%S"),
                        "kind": "intent" if ok else "error",
                        "text": f"[수동조작] {detail}",
                    })
            self._write_json({"ok": ok, "detail": detail})

        def _stream_mjpeg(self, source) -> None:
            """MJPEG 스트림 — source.latest_jpeg()를 ~10fps로 흘려보낸다.

            소스 프로세스가 아직 프레임을 안 썼거나 죽어도 핸들러가 예외로
            죽지 않게 — 대기했다 다시 본다.
            """
            if source is None:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    try:
                        jpeg = source.latest_jpeg()
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

        def _write_json(self, obj: dict) -> None:
            payload = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def _write_event(self, event: dict) -> None:
            payload = f"data: {json.dumps(event, ensure_ascii=False)}\n\n"
            self.wfile.write(payload.encode("utf-8"))
            self.wfile.flush()

    return Handler


def start_log_server(
    log_hub: LogHub, port: int, vision=None, hardware: dict | None = None, floor=None
) -> ThreadingHTTPServer:
    """백그라운드 스레드에서 서버를 띄우고 서버 객체를 반환(종료 시 shutdown() 호출용).

    vision(VisionStreamer)을 넘기면 /video MJPEG 엔드포인트가 활성화된다.
    floor(latest_jpeg()를 가진 소스)를 넘기면 /video2(바닥 카메라)도 켜진다.
    hardware는 {"arm":..., "base":...} 가변 딕셔너리 — 서버를 먼저 띄우고
    나중에 채워도 /control과 POST /cmd가 그때부터 동작한다.
    """
    server = ThreadingHTTPServer(
        ("0.0.0.0", port), _make_handler(log_hub, vision, hardware, floor)
    )
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

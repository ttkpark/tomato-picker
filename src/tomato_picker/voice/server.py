"""외부 패키지 없이 stdlib http.server만으로 만든 실시간 인식 로그 뷰어(SSE).

PC 브라우저에서 http://<젯슨IP>:포트 로 접속하면 인식된 텍스트가
발화 즉시(수 초 지연) 스트리밍된다.
"""

from __future__ import annotations

import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from ..config import BASE_DRIVE_SPEED, CAMERA_HEIGHT
from . import pages
from .intents import match_intent
from .linkmon import LinkSampler
from .log_hub import LogHub
from .wake import GATE as WAKE
from .words import STORE as WORDS


def _status_payload(arm, base, line=None, seq=None) -> dict:
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
    line_status = (
        _safe(line.status, {"mode": "idle"})
        if line is not None
        else {"mode": "off", "error": "라인 주행 비활성"}
    )
    seq_status = (
        _safe(seq.status, {"sequences": {}})
        if seq is not None
        else {"sequences": {}, "error": "시퀀스 비활성"}
    )
    # _safe의 폴백은 dict여야 한다(에러를 병합해 돌려주므로) — 목록은 감싸서 넘긴다.
    voice_status = _safe(lambda: {"items": WORDS.catalog(), "wake": WAKE.status()},
                         {"items": [], "wake": {}})
    return {"arm": arm_status, "base": base_status, "line": line_status, "seq": seq_status,
            "voice": voice_status}

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
         box-sizing: border-box; height: 100vh; height: 100dvh;
         display: flex; flex-direction: column; gap: 0.5rem;
         background: Canvas; color: CanvasText; }}
  /* --- 영상과 로그는 **둘 다 늘 보여야 한다** ---
     예전엔 제목·배지·개수·영상·바닥영상이 전부 한 덩어리(#top)에 있고 그 덩어리가
     flex:0 0 auto(축소 불가)였다. 카메라(50vh) + 바닥카메라(높이 상한 없음)를 더하면
     화면보다 커져서 #log가 0px로 밀려났다 — "카메라가 뜨면 채팅이 안 보인다"의 정체.
     창이 가려지면 MJPEG이 멈춰 <img> 높이가 0이 되니 그제서야 로그가 나타났다.
     이제 얇은 머리(#top)만 고정이고, 영상과 로그는 각자 줄어드는 두 칸(#panes)이다.
     좁은 화면에서는 위아래로 쌓되 **최소 지분**을 보장하고, 넓으면 좌우로 나눈다. */
  #top {{ flex: 0 0 auto; }}
  #panes {{ flex: 1 1 auto; min-height: 0; display: flex; gap: 1rem; }}
  /* 넓은 화면: 좌우로 나눈다(영상 왼쪽, 로그 오른쪽). 좁은 화면 규칙은 이 파일
     **맨 아래** @media에 있다 — 같은 특이도면 뒤에 온 규칙이 이기므로 순서가 곧 의미다. */
  #media {{ flex: 0 1 720px; min-height: 0; overflow-y: auto; }}
  h1 {{ font-size: 1.1rem; opacity: 0.8; margin: 0 0 0.5rem; }}
  #count {{ font-size: 1.6rem; font-weight: 700; padding: 0.5rem 0.9rem; border-radius: 10px;
           background: color-mix(in srgb, #2ecc71 22%, Canvas); margin-bottom: 0.4rem;
           display: inline-block; }}
  #positions {{ font-size: 0.95rem; opacity: 0.8; margin-bottom: 0.75rem;
               font-variant-numeric: tabular-nums; }}
  #cam {{ width: 100%; max-width: 720px; max-height: 46vh; object-fit: contain;
          border-radius: 10px; display: block; margin-bottom: 0.5rem; background: #000; }}
  #floorlabel {{ font-size: 0.85rem; opacity: 0.7; margin: 0.25rem 0 0.2rem; }}
  #line {{ padding: 0.1rem 0.5rem; border-radius: 999px; font-variant-numeric: tabular-nums;
          background: color-mix(in srgb, CanvasText 10%, Canvas); }}
  #line.ok {{ background: color-mix(in srgb, #2ecc71 30%, Canvas); }}
  #line.lost {{ background: color-mix(in srgb, #e74c3c 28%, Canvas); }}
  #line.mark {{ background: color-mix(in srgb, #f39c12 40%, Canvas); font-weight: 700; }}
  /* ⚠ 높이 상한이 **반드시** 있어야 한다. 없던 시절 400px 폭 4:3 영상이 300px를
     차지해 카메라와 합쳐 화면을 넘겼다(위 #panes 주석 참고). */
  #floorcam {{ width: 100%; max-width: 400px; max-height: 26vh; object-fit: contain;
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
  #status {{ font-size: 0.85rem; opacity: 0.6; margin: 0.25rem 0 0; }}
  /* 호출어 배지 — 지금 명령을 받는 상태인지 한눈에. */
  #wake {{ font-size: 1rem; font-weight: 700; padding: 0.3rem 0.8rem; border-radius: 999px;
          display: inline-block; margin-bottom: 0.5rem;
          background: color-mix(in srgb, CanvasText 10%, Canvas); }}
  #wake.on {{ background: color-mix(in srgb, #2ecc71 30%, Canvas); }}

  /* ===== 좁은 화면(휴대폰·작은 창): 위아래로 쌓되 로그 몫을 먼저 떼어 둔다 =====
     ⚠ 이 블록은 반드시 **위 기본 규칙들 뒤**에 있어야 한다. 앞에 뒀더니 같은
     특이도의 기본 #cam/#floorcam 규칙이 이겨서 아무 효과가 없었다(2026-08-13).
     ⚠ 지분은 %가 아니라 **grow 비율 + basis 0**으로 준다. 퍼센트 flex-basis는
     부모 높이가 확정일 때만 뜻이 있는데 이 중첩에선 아니라, 영상칸이 88px까지
     쪼그라들었다. basis 0이면 확정이라 55:45가 그대로 나온다. */
  @media (max-width: 899px) {{
    #panes {{ flex-direction: column; }}
    #media {{ flex: 55 1 0; min-height: 0; overflow-y: auto; }}
    #log {{ flex: 45 1 0; min-height: 0; }}
    /* 이미지는 칸 안에서 줄어들 수 있어야 한다 — 안 그러면 칸 밖으로 삐져나가
       "카메라가 잘려 보인다". object-fit:contain이라 비율은 유지된다. */
    #cam {{ max-height: 34vh; }}
    #floorcam {{ max-height: 14vh; }}
    /* 머리글이 두 줄로 늘어나 본문을 잡아먹으므로 조금 눌러 둔다. */
    h1 {{ font-size: 1rem; margin-bottom: 0.35rem; }}
    #count {{ font-size: 1.3rem; padding: 0.35rem 0.7rem; }}
    #positions {{ font-size: 0.85rem; margin-bottom: 0.4rem; }}
  }}
</style></head>
<body>
<div id="top">
<h1>토마토피커 — 실시간 대시보드 &nbsp;<a href="/control">🎮 수동 조작</a>&nbsp;<a href="/settings">⚙ 시스템 설정</a></h1>
<div id="hw"><span>장비 상태 확인 중...</span></div>
<div id="wake">🎤 음성 상태 확인 중...</div>
<div id="count">🍅 공중 토마토: —</div>
<div id="positions">위치: —</div>
</div>
<div id="panes">
  <div id="media">
{video_block}
    <div id="status">연결 중...</div>
  </div>
  <div id="log"></div>
</div>
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
      // 세 상태: ok(●) / 진행 중(◌ — 대기·연결 중) / down(○).
      // 진행 중을 down과 같이 빨갛게 칠하면 "고장"으로 오해된다.
      const busy = (state === '연결 중' || state === '대기' || state === '확인 중');
      el.className = state === 'ok' ? 'ok' : (busy ? '' : 'down');
      el.textContent = (state === 'ok' ? '● ' : (busy ? '◌ ' : '○ ')) + name
                     + (busy ? ' (' + state + ')' : '');
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
  const wakeEl = document.getElementById('wake');
  function onEvent(ev) {{
    if (ev.kind === 'hw') {{ renderHw(ev.items); return; }}
    // 호출어 상태 — 배지만 갈아끼우고 로그로는 안 쌓는다(1초마다 오므로).
    if (ev.kind === 'wake') {{
      wakeEl.textContent = ev.text;
      wakeEl.className = ev.open ? 'on' : '';
      return;
    }}
    // 바닥 카메라 라인 검출 — 배지만 갈아끼우고 로그로는 안 쌓는다.
    if (ev.kind === 'line') {{
      if (!lineEl) return;
      // 테이프는 진행방향과 나란한 **가로 띠**다 — 세로 위치(dy)가 무대와의
      // 거리 오차, 기울기가 로봇 요(yaw) 오차. 자세한 기하는 line_follow.py.
      const parts = [];
      if (ev.found) {{
        const dy = Math.round(ev.offset_y_px);
        parts.push('● 테이프 ' + (dy === 0 ? '기준' : (dy > 0 ? '아래 ' : '위 ') + Math.abs(dy) + 'px'));
        if (ev.angle_deg !== null && ev.angle_deg !== undefined) parts.push('∠' + ev.angle_deg + '°');
      }} else {{
        parts.push('○ 테이프 없음');
      }}
      if (ev.marker_name) parts.push(
        (ev.marker_role === 'end' ? '🚩 ' : ev.marker_role === 'mid' ? '📍 ' : '❔ ') + ev.marker_name);
      if (ev.end_side) parts.push('■ 코스 끝(' + (ev.end_side === 'left' ? '좌' : '우') + ')');
      lineEl.className = ev.end_side ? 'mark' : (ev.found ? 'ok' : 'lost');
      lineEl.textContent = parts.join('  ·  ');
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


def _handle_line_command(body: dict, line) -> tuple[bool, str] | None:
    """라인 주행 명령. 이 모듈이 처리할 게 아니면 None."""
    action = body.get("action")
    if not str(action).startswith("line_"):
        return None
    if line is None:
        return False, "라인 주행 비활성(바닥 카메라/바퀴 확인)"
    side = body.get("side", "right")
    speed = body.get("speed")
    if action == "line_goto_end":
        return True, line.goto_end(side, speed)
    if action == "line_travel":
        return True, line.travel(side, float(body.get("seconds", 1.0)), speed)
    if action == "line_jog":
        return True, line.jog(side, float(body.get("seconds", 0.25)), speed)
    if action == "line_align":
        return True, line.align(speed)
    if action == "line_station":
        return True, line.goto_station(int(body.get("index", 0)), speed)
    if action == "line_next":
        return True, line.next_station(side, speed)
    if action == "line_goto_color":
        # 색 구분은 기본 꺼짐(배경색을 통제하기 전까지 hue를 못 믿는다).
        # 켜지 않은 상태에서 누르면 "왜 안 되는지"를 그대로 알려준다.
        if not line.status().get("color_name") and not body.get("force"):
            return False, ("색 구분이 꺼져 있습니다 — 배경색을 통제한 뒤 "
                           "line-follow.service에 LF_COLOR=1을 넣고 재시작하세요. "
                           "지금은 [시간이동]이나 [끝까지]를 쓰세요.")
        return True, line.goto_color(side, body.get("name") or None, speed)
    if action == "line_stop":
        line.cancel("사용자 정지")
        return True, "라인 주행 정지"
    if action == "line_reset_origin":
        return True, line.reset_origin()
    if action == "line_set_target":
        return True, line.set_target_y()
    if action == "line_flip":
        return True, line.flip(str(body.get("what", "dy_sign")))
    if action == "line_align_mark":
        # 지점 정렬 — 마커(주황/노랑)를 화면 중앙에. 도착 후 자동으로도 돈다.
        return True, line.align_mark(body.get("speed"))
    if action == "line_set_station":
        # "지금 여기가 N번 지점" — 마커 세기가 방향 부호에 의존해 어긋났을 때
        # 번호를 되찾는 확실한 길. 부호가 맞든 틀리든 항상 통한다.
        return True, line.set_station(int(body.get("index", 0)))
    if action == "line_params":
        return True, line.set_params(
            speed=body.get("speed"), pulse_on=body.get("pulse_on"),
            pulse_period=body.get("pulse_period"),
            align_dither=body.get("align_dither"),
            # ⚠ smooth_speed가 빠져 있었다(2026-08-13 발견). 대시보드는 보내는데
            #   여기서 안 넘겨서 **[저속 주행 속도] 슬라이더가 조용히 무시**됐다 —
            #   "적용을 눌러도 속도가 그대로"의 원인. 목록에 빠지면 티가 안 나므로
            #   set_params의 limits와 여기를 항상 같이 볼 것.
            smooth_speed=body.get("smooth_speed"),
            travel_kick=body.get("travel_kick"),
            travel_wiggle=body.get("travel_wiggle"),
            # 정렬 완료 목표 범위(px·도).
            align_tol_x=body.get("align_tol_x"),
            align_dy_below=body.get("align_dy_below"),
            align_dy_above=body.get("align_dy_above"),
            align_yaw_tol=body.get("align_yaw_tol"),
            # 정렬 펄스 크기(게걸음) — 속도 슬라이더가 안 닿는 축.
            corr_min=body.get("corr_min"),
            corr_max=body.get("corr_max"),
        )
    return False, f"알 수 없는 라인 명령: {action}"


# 대시보드에서 재시작을 허용하는 서비스. ⚠ controller-drive는 뺐다 —
# tomato-voice와 ttyUSB0을 다투므로(독점) 둘이 같이 뜨면 나중 쪽이 죽는다.
_SERVICES = ("tomato-voice", "line-follow", "line-cam", "tomato-vision")

# 시퀀스 러너 핸들. _handle_command는 인자가 이미 많아 여기 담아 쓴다
# (핸들러 생성 시 hardware 딕셔너리에서 꺼내 넣는다).
_SEQ: dict = {}


def _handle_command(body: dict, arm, base, vision=None, line=None) -> tuple[bool, str]:
    """수동 조작 명령 하나를 실행. (성공여부, 사람이 읽을 설명)을 돌려준다.

    장비가 없으면(Mock/None) 에러 대신 그 사실을 문자열로 알려준다 —
    조작 화면은 하드웨어가 없어도 떠 있어야 하기 때문.
    """
    action = body.get("action")
    try:
        # --- 서비스 관리 (하드웨어 객체 불필요) ---
        # 팔이 안 움직일 때(토크 Overload로 굳는 등) 현장에서 ssh 없이 되살리는
        # 손잡이. 팔·바퀴 연결은 tomato-voice가 들고 있으므로 그걸 재시작하는 게
        # 재연결이다. ⚠ 이름은 화이트리스트만 — 임의 명령 실행 통로가 되면 안 된다.
        if action == "service_restart":
            name = str(body.get("name", ""))
            if name not in _SERVICES:
                return False, f"허용되지 않은 서비스: {name}"
            # 분리 실행 + 1초 지연: tomato-voice가 자기 자신을 재시작할 때
            # HTTP 응답이 브라우저에 도착할 시간을 준다(systemd Restart=always라
            # 죽어도 다시 올라온다).
            # ⚠ sudo를 쓰면 안 된다 — 유닛의 CapabilityBoundingSet이 자식의 권한
            #   상승을 전부 잘라 "unable to change to root gid"로 실패한다
            #   (2026-08-12 실사고: 버튼이 조용히 아무것도 안 했다). 대신 polkit
            #   규칙(deploy/50-tomato-services.rules)이 server 계정에 이 유닛들의
            #   restart만 허용한다 — systemctl은 D-Bus 경유라 권한 상승이 필요 없다.
            subprocess.Popen(
                ["sh", "-c", f"sleep 1 && systemctl restart {name}"],
                start_new_session=True,
            )
            note = (" — 화면이 몇 초 끊겼다 돌아옵니다" if name == "tomato-voice" else "")
            return True, f"{name} 재시작 중...{note}"
        if action == "service_status":
            outs = []
            for svc in _SERVICES:
                r = subprocess.run(["systemctl", "is-active", svc],
                                   capture_output=True, text=True, timeout=5)
                outs.append(f"{svc}: {(r.stdout or r.stderr).strip() or '?'}")
            return True, " · ".join(outs)

        # --- 음성 명령어 사전 (하드웨어 불필요) ---
        # 저장은 즉시 반영된다 — 인텐트 매칭이 매번 STORE에서 읽으므로 재시작이 필요 없다.
        if action == "voice_words_save":
            return True, WORDS.set(str(body.get("key", "")), str(body.get("text", "")))
        if action == "voice_words_reset":
            key = body.get("key")
            return True, WORDS.reset(None if key in (None, "", "all") else str(key))
        if action == "voice_wake":
            return True, WAKE.configure(
                enabled=body.get("enabled"),
                window_sec=body.get("window_sec"))
        if action == "voice_test":
            # 말하지 않고 문장을 넣어 어떤 명령으로 읽히는지 본다 — 낱말을 고친 뒤
            # 마이크에 대고 시험하려면 사람이 계속 말해야 하고, 그러면 로봇이 움직인다.
            text = str(body.get("text", ""))
            got = match_intent(text)
            if got is None:
                return True, f"'{text}' → (매칭 없음)"
            slots = f"  {got.slots}" if got.slots else ""
            return True, f"'{text}' → {got.label}{slots}"

        # --- 시퀀스 (팔 + 지점 이동 혼합 대본) ---
        if action in ("seq_start", "seq_stop", "seq_save"):
            seq = _SEQ.get("runner")
            if seq is None:
                return False, "시퀀스 러너가 없습니다"
            if action == "seq_start":
                return True, seq.start(str(body.get("key", "A")))
            if action == "seq_stop":
                return True, seq.stop()
            return True, seq.set_sequence(str(body.get("key", "A")), str(body.get("text", "")))

        handled = _handle_line_command(body, line)
        if handled is not None:
            return handled
        # 수동 조작이 들어오면 자동주행은 취소한다 — 두 주체가 같은 바퀴를
        # 서로 다른 방향으로 몰면 아무도 예측 못 하는 움직임이 나온다.
        if line is not None and action in ("drive", "hold", "stop"):
            if action != "hold" or any(int(body.get(k, 0)) for k in ("vx", "vy", "w")):
                line.cancel("수동 조작으로 전환")
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
                speed_scale=body.get("speed_scale"),
            )
            return True, (f"튜닝 적용: 전체 속도 {t['speed_scale']}% · {t['hz']}Hz"
                          f" · 듀티상한 {t['max_pwm']}/4095"
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
        elif action == "arm_cal_start":
            return True, arm.start_calibration()
        elif action == "arm_cal_finish":
            return True, arm.finish_calibration()
        elif action == "arm_cal_cancel":
            return True, arm.cancel_calibration()
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
    log_hub: LogHub, vision=None, hardware: dict | None = None, floor=None, sampler=None
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
                self._write_json(_status_payload(hw.get("arm"), hw.get("base"),
                                                 hw.get("line"), hw.get("seq")))
                return

            if self.path in ("/control", "/settings", "/diag"):
                html = (pages.settings_page() if self.path == "/settings"
                        else pages.diag_page() if self.path == "/diag"
                        else pages.control_page(BASE_DRIVE_SPEED, CAMERA_HEIGHT))
                body = html.encode("utf-8")
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

            if self.path == "/diag-events":
                # 링크 계측 스트림. /status(1초 폴링)로는 80ms 펄스가 통째로
                # 사라지므로, 서버가 100Hz로 뜬 파형을 0.1초마다 묶어 밀어준다.
                # since를 들고 있어서 브라우저가 잠깐 멈췄다 와도 구간이 안 빈다.
                if sampler is None:
                    self.send_response(503)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                since = -1.0
                try:
                    while True:
                        payload = sampler.snapshot(since)
                        since = payload["t"]
                        self._write_event(payload)
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass   # 탭을 닫았다 — 정상 종료
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
            # 러너는 서버보다 늦게 만들어진다 — 요청 시점에 최신 핸들을 집는다.
            _SEQ["runner"] = hw.get("seq")
            length = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                ok, detail = False, "JSON 파싱 실패"
            else:
                ok, detail = _handle_command(body, hw.get("arm"), hw.get("base"), vision,
                                             hw.get("line"))
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
    # 링크 계측 샘플러(/diag). 같은 가변 딕셔너리를 들여다보게 해서, 바퀴가
    # 나중에 붙어도(또는 재연결돼도) 알아서 따라간다.
    hw = hardware if hardware is not None else {}
    sampler = LinkSampler(lambda: hw.get("base"))
    server = ThreadingHTTPServer(
        ("0.0.0.0", port), _make_handler(log_hub, vision, hw, floor, sampler)
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

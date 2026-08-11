"""대시보드의 HTML 페이지 템플릿.

server.py에서 분리한 이유 — 조작 화면이 커지면서 한 파일 안에서 다루기
어려워졌고, **조작(운전)** 과 **설정(영점·튜닝)** 을 화면 단위로 갈라야 했다.

  /control   운전 화면 — 데모 중에 실제로 누르는 것만
  /settings  시스템 설정 — 영점/축·부호/모터 튜닝/팔 캘리브레이션처럼
             "한 번 잡아두고 잘 안 건드리는" 것
  /diag      링크 실시간 계측 — 지령 파형·펄스폭·수신율. 1초 폴링으로는
             80ms 펄스가 안 보여서 이 화면만 SSE(10Hz 푸시)로 따로 뺐다.

f-string을 안 쓴다(브레이스 이스케이프 지옥). 치환 자리표시자는 __NAME__ 형식.
"""

from __future__ import annotations

_CSS = """
  :root { color-scheme: light dark; }
  body { font-family: -apple-system, "Malgun Gothic", sans-serif; margin: 0; padding: 1rem;
         background: Canvas; color: CanvasText; }
  h1 { font-size: 1.1rem; opacity: 0.85; margin: 0 0 0.75rem; }
  h2 { font-size: 0.95rem; opacity: 0.7; margin: 1.4rem 0 0.5rem; }
  a { color: inherit; }
  nav { display: flex; gap: 0.5rem; flex-wrap: wrap; margin-bottom: 1rem; }
  nav a { padding: 0.35rem 0.7rem; border-radius: 999px; text-decoration: none;
          background: color-mix(in srgb, CanvasText 8%, Canvas); font-size: 0.9rem; }
  nav a.here { background: color-mix(in srgb, #2ecc71 30%, Canvas); font-weight: 700; }
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
  .card { padding: 0.6rem 0.8rem; border-radius: 10px; margin-bottom: 0.5rem;
          background: color-mix(in srgb, CanvasText 6%, Canvas); }
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
  #link, #lineState, #calState, #tNow { font-size: 0.9rem; font-variant-numeric: tabular-nums; }
  /* /diag 계측 화면 — 숫자가 1초에 10번 바뀌므로 tabular-nums가 필수다.
     아니면 자릿수가 바뀔 때마다 폭이 흔들려 읽을 수가 없다. */
  .mgrid { display: grid; gap: 0.5rem; margin-bottom: 0.6rem;
           grid-template-columns: repeat(auto-fill, minmax(160px, 1fr)); max-width: 1000px; }
  .m { border-radius: 10px; padding: 0.5rem 0.7rem; font-variant-numeric: tabular-nums;
       background: color-mix(in srgb, CanvasText 6%, Canvas);
       border: 1px solid color-mix(in srgb, CanvasText 12%, Canvas); }
  .m .lbl { font-size: 0.75rem; opacity: 0.65; }
  .m .val { font-size: 1.35rem; font-weight: 700; line-height: 1.25; }
  .m .sub { font-size: 0.75rem; opacity: 0.6; min-height: 1.1em; }
  .m.ok  { background: color-mix(in srgb, #2ecc71 16%, Canvas); }
  .m.warn{ background: color-mix(in srgb, #f1c40f 22%, Canvas); }
  .m.bad { background: color-mix(in srgb, #e74c3c 22%, Canvas); }
  #wave { width: 100%; max-width: 1000px; height: 190px; border-radius: 10px;
          background: color-mix(in srgb, CanvasText 6%, Canvas); }
  #pulses { font-variant-numeric: tabular-nums; font-size: 0.85rem; line-height: 1.7;
            max-width: 1000px; }
  .pw { display: inline-block; min-width: 3.6em; text-align: right; padding: 0 0.35rem;
        margin: 0 0.15rem 0.15rem 0; border-radius: 5px;
        background: color-mix(in srgb, CanvasText 10%, Canvas); }
  .legend span { margin-right: 0.9rem; font-size: 0.8rem; }
  .sw { display: inline-block; width: 0.75rem; height: 0.75rem; border-radius: 3px;
        vertical-align: -1px; margin-right: 0.25rem; }
  #out { margin-top: 1rem; padding: 0.6rem 0.8rem; border-radius: 8px; min-height: 1.2em;
         background: color-mix(in srgb, CanvasText 6%, Canvas); font-size: 0.9rem;
         white-space: pre-wrap; }
"""

# 두 페이지가 공유하는 JS — 명령 전송과 상태 폴링.
_COMMON_JS = """
  const out = document.getElementById('out');
  let busy = false;
  let state = null;
  async function cmd(body) {
    // 시리얼 명령은 겹쳐 보내면 포트가 깨진다 — 한 번에 하나만.
    if (busy) { out.textContent = '이전 명령 실행 중...'; return; }
    busy = true;
    out.textContent = '실행 중: ' + JSON.stringify(body);
    try {
      const r = await fetch('/cmd', {method: 'POST', body: JSON.stringify(body)});
      const j = await r.json();
      out.textContent = (j.ok ? '완료: ' : '실패: ') + (j.detail || JSON.stringify(body));
    } catch (e) { out.textContent = '요청 실패: ' + e; }
    finally { busy = false; refresh(); }
  }
  async function refresh() {
    try { state = await (await fetch('/status')).json(); render(); }
    catch (e) { /* 다음 주기에 다시 */ }
  }
  setInterval(refresh, 1000);
"""

_NAV = """
<nav>
  <a href="/">📊 대시보드</a>
  <a href="/control" class="__C__">🎮 수동 조작</a>
  <a href="/settings" class="__S__">⚙ 시스템 설정</a>
  <a href="/diag" class="__D__">🔌 링크 계측</a>
</nav>
"""


def _shell(title: str, body: str, script: str, here: str, common: bool = True) -> str:
    """페이지 껍데기.

    common=False면 1초 /status 폴링(_COMMON_JS)을 빼고 페이지 스크립트만 넣는다.
    /diag는 SSE로 10Hz를 받으므로 폴링을 겹치면 낭비이고, 그 폴링 자체가
    이 화면이 재려는 젯슨 부하에 섞여 들어간다(관측이 대상을 흔든다).
    """
    nav = _NAV.replace("__C__", "here" if here == "control" else "") \
              .replace("__S__", "here" if here == "settings" else "") \
              .replace("__D__", "here" if here == "diag" else "")
    boot = f"{_COMMON_JS}{script}\nrefresh();" if common else script
    return ("<!doctype html>\n<html lang=\"ko\"><head><meta charset=\"utf-8\">\n"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
            f"<title>{title}</title>\n<style>{_CSS}</style></head>\n<body>\n"
            f"<h1>{title}</h1>\n{nav}\n{body}\n<div id=\"out\">명령 대기 중</div>\n"
            f"<script>{boot}</script>\n</body></html>")


# ======================================================================
# /control — 운전 화면
# ======================================================================

_CONTROL_BODY = """
<div class="card" id="link">링크 상태 확인 중...</div>

<div class="card" id="keys">⌨ <b>키보드</b>: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> 또는 방향키 = 이동(누르는 동안 계속) ·
<kbd>Q</kbd><kbd>E</kbd> = 회전 · <kbd>Space</kbd> = 정지 · <kbd>0</kbd>~<kbd>9</kbd> = 프리셋(현재 모드로) · <kbd>R</kbd> = 힘 빼기
<span id="held" class="dim"></span></div>

<h2>라인 주행 <span class="dim">— 테이프를 유지하며 무대 앞을 오간다</span></h2>
<div class="card">
  <div id="lineState" style="margin-bottom:0.6rem">라인 상태 확인 중...</div>
  <div class="row-flex" style="margin-bottom:0.5rem">
    <button onclick="cmd({action:'line_goto_end',side:'left'})">◀◀ 왼쪽 끝까지</button>
    <button class="stop" onclick="cmd({action:'line_stop'})">■ 정지</button>
    <button onclick="cmd({action:'line_goto_end',side:'right'})">오른쪽 끝까지 ▶▶</button>
  </div>
  <div class="row-flex" style="margin-bottom:0.5rem">
    <span>지점:</span><span id="stationBtns"></span>
    <button onclick="cmd({action:'line_next',side:'left'})">◀ 이전 지점</button>
    <button onclick="cmd({action:'line_next',side:'right'})">다음 지점 ▶</button>
  </div>
  <!-- 번호가 실제 위치와 어긋났을 때의 탈출구. 마커 세기는 진행 방향에
       의존하는데, 손으로 옮기거나 방향 부호가 반대면 그 전제가 깨진다.
       이건 부호와 무관하게 항상 통한다. -->
  <div class="row-flex" style="margin-bottom:0.5rem">
    <span class="dim">번호가 틀렸다면 → 지금 위치를:</span><span id="setStationBtns"></span>
  </div>
  <div class="row-flex" style="margin-bottom:0.5rem">
    <button onclick="cmd({action:'line_jog',side:'left'})">◀ 톡</button>
    <button onclick="cmd({action:'line_jog',side:'right'})">톡 ▶</button>
    <span class="dim">|</span>
    <button onclick="cmd({action:'line_align'})">🎯 정렬(톡톡)</button>
    <span class="dim">제자리에서 톡톡 쳐가며 기준선·평행에 맞춘다</span>
  </div>
  <div class="row-flex">
    <button onclick="lineTravel('left')">◀ 시간이동</button>
    <input type="number" id="lineSecs" value="1.5" step="0.1" min="0.1" max="20" style="width:5rem">초
    <button onclick="lineTravel('right')">시간이동 ▶</button>
  </div>
</div>

<h2>바퀴 수동 <span class="dim">— 라인과 무관하게 직접</span></h2>
<label>속도 <input type="range" id="speed" min="40" max="255" value="__SPEED__"
  oninput="speedOut.textContent=this.value"> <b id="speedOut">__SPEED__</b> / 255</label>
<label>동작 시간 <input type="range" id="secs" min="0.2" max="3" step="0.1" value="0.6"
  oninput="secsOut.textContent=this.value"> <b id="secsOut">0.6</b> 초</label>
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

<h2>리더암 <span class="dim">— 자세 만들기</span></h2>
<div class="card">
  <div class="row-flex">
    <select id="leaderPort"><option value="">포트 자동선택</option></select>
    <button id="btnLeader" onclick="toggleLeader()">리더암 연결</button>
    <button id="btnMirror" onclick="toggleMirror()">미러링 시작</button>
    <span id="leaderInfo" class="dim"></span>
  </div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    미러링 ON → 리더를 손으로 움직이면 팔로워가 따라온다. 원하는 자세에서 아래
    <b>등록 모드</b>로 바꾸고 슬롯을 누르면 그 자세가 저장된다. 리더 없이도
    <b>힘 빼기</b> 후 손으로 자세를 잡아 저장할 수 있다.
  </p>
</div>

<h2>프리셋 슬롯 0~9</h2>
<div class="card">
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

<h2>높이 보간 <span class="dim">— 앵커 사이 자세 계산</span></h2>
<div class="card">
  <div id="anchorList" class="dim" style="margin-bottom:0.5rem">앵커 없음</div>
  <label>목표 높이 y <input type="range" id="blendY" min="0" max="__FRAME_H__" value="360"
    oninput="blendOut.textContent=this.value"> <b id="blendOut">360</b> px</label>
  <div class="row-flex">
    <button onclick="cmd({action:'arm_blend', y:+document.getElementById('blendY').value})">이 높이로 이동</button>
    <button onclick="cmd({action:'arm_blend', from_vision:true})">🍅 감지된 토마토 높이로 이동</button>
  </div>
</div>
"""

_CONTROL_JS = """
  let mode = 'play';
  function lineTravel(side) {
    cmd({action:'line_travel', side, seconds: +document.getElementById('lineSecs').value});
  }
  function drive(vx, vy, w) {
    const s = +document.getElementById('speed').value;
    cmd({action:'drive', vx: vx*s, vy: vy*s, w: w*s,
         seconds: +document.getElementById('secs').value});
  }
  function setMode(m) {
    mode = m;
    document.getElementById('btnPlay').className = m === 'play' ? 'on' : '';
    document.getElementById('btnSave').className = m === 'save' ? 'on' : '';
    document.getElementById('mode').textContent =
      m === 'play' ? '슬롯을 누르면 그 자세로 이동합니다.'
                   : '슬롯을 누르면 지금 팔 자세를 그 슬롯에 덮어씁니다.';
    render();
  }
  function slotFilled(n) {
    const s = state && state.arm && (state.arm.presets.slots || []).find(x => x.slot === n);
    return !!(s && s.filled);
  }
  function slotClick(n) {
    if (mode === 'save') {
      if (!confirm(n + '번 슬롯에 지금 자세를 저장할까요?' +
                   (slotFilled(n) ? '\\n(기존 자세는 덮어써집니다)' : ''))) return;
      cmd({action:'arm_save', slot:n});
    } else { cmd({action:'arm_preset', preset:n}); }
  }
  function rename(n) {
    const cur = ((state.arm.presets.slots || []).find(x => x.slot === n) || {}).name || '';
    const name = prompt(n + '번 슬롯 이름 (예: 접근, 집기)', cur);
    if (name === null) return;
    cmd({action:'arm_rename', slot:n, name});
  }
  function anchor(n) {
    const s = (state.arm.presets.slots || []).find(x => x.slot === n) || {};
    if (s.anchor) { if (confirm(n + '번의 높이 앵커를 해제할까요?')) cmd({action:'arm_anchor_clear', slot:n}); return; }
    const label = prompt('앵커 라벨 (상 / 중 / 하 등)', '중');
    if (!label) return;
    const y = prompt('이 자세가 가리키는 화면 높이 y', '360');
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
  function toggleLeader() {
    if (state && state.arm && state.arm.leader_connected) cmd({action:'leader_disconnect'});
    else cmd({action:'leader_connect', port: document.getElementById('leaderPort').value || null});
  }
  function toggleMirror() {
    cmd({action:'mirror', on: !(state && state.arm && state.arm.mirroring)});
  }

  function render() {
    if (!state) return;
    const arm = state.arm || {}, base = state.base || {}, ln = state.line || {};

    const ok = base.connected;
    const bits = [ok ? '● 바퀴 링크 정상' : '○ 바퀴 링크 끊김'];
    if (base.hb_age !== null && base.hb_age !== undefined) bits.push('hb ' + base.hb_age + 's 전');
    bits.push('수신 ' + (base.fw_rx || 0));
    if (base.fw_bad) bits.push('⚠ 깨진프레임 ' + base.fw_bad);
    if (base.hb_resets) bits.push('⚠ 링크복구 ' + base.hb_resets + '회');
    if (base.error) bits.push(base.error);
    const link = document.getElementById('link');
    link.textContent = bits.join('  ·  ');
    link.style.background = ok ? 'color-mix(in srgb, #2ecc71 18%, Canvas)'
                               : 'color-mix(in srgb, #e74c3c 18%, Canvas)';

    const el = document.getElementById('lineState');
    if (!ln || ln.mode === 'off') { el.textContent = ln.error || '라인 주행 비활성'; }
    else {
      const p = [ln.mode === 'idle' ? '⏸ 대기' : '▶ ' + ln.mode, ln.detail || ''];
      if (ln.found) {
        const dy = Math.round(ln.offset_y_px);
        p.push('테이프 ' + (dy > 0 ? '아래 ' : '위 ') + Math.abs(dy) + 'px'
               + (ln.angle_deg == null ? '' : ' ∠' + ln.angle_deg + '°'));
      } else p.push('⚠ 테이프 없음' + (ln.found_reason ? ' (' + ln.found_reason + ')' : ''));
      p.push(ln.position_px != null
             ? '변위 ' + Math.round(ln.position_px) + 'px'
               + (ln.position_pct != null ? ' (' + ln.position_pct + '%)' : '')
             : '변위 미기준 — 끝단을 한 번 지나가세요');
      if (ln.color_name) p.push((ln.color_role === 'end' ? '🚩 ' : '📍 ') + ln.color_name);
      if (ln.end_side) p.push('■ 끝(' + (ln.end_side === 'left' ? '좌' : '우') + ')');
      p.push((ln.station != null ? '📍 ' + ln.station_label
                                 : '📍 지점 미확인 — 주황 끝지점을 한 번 지나가세요')
             + (ln.between ? ' (이동 중)' : ''));
      if (ln.phase) p.push(ln.phase === 'coarse' ? '🏃 접근(길게)' : '👆 정밀(톡톡)');
      const seen = (ln.markers || []).map(m => m.name + '(h' + Math.round(m.hue) + ')');
      if (seen.length) p.push('보이는 마커 ' + seen.join(', '));
      p.push(ln.pulsing ? '펄스 ' + ln.pulse_on + 's/' + ln.pulse_period + 's' : '연속');
      el.textContent = p.filter(Boolean).join('  ·  ');
      el.style.background = ln.mode !== 'idle' ? 'color-mix(in srgb, #2ecc71 22%, Canvas)'
                          : (ln.found ? '' : 'color-mix(in srgb, #e74c3c 15%, Canvas)');
    }

    // 지점 버튼 — 라벨은 서버가 준다(코스 구성이 바뀌면 여기 안 고쳐도 된다)
    const sb = document.getElementById('stationBtns');
    const labels = ln.station_labels || [];
    if (sb && sb.dataset.n !== String(labels.length)) {
      sb.dataset.n = String(labels.length);
      sb.textContent = '';
      labels.forEach((lab, i) => {
        const b = document.createElement('button');
        b.className = 'small'; b.textContent = i + '. ' + lab;
        b.onclick = () => cmd({action:'line_station', index:i});
        sb.append(b);
      });
    }
    // "지금 위치를 N번으로 선언" 버튼 — 이동이 아니라 **번호 정정**이다.
    const ssb = document.getElementById('setStationBtns');
    if (ssb && ssb.dataset.n !== String(labels.length)) {
      ssb.dataset.n = String(labels.length);
      ssb.textContent = '';
      labels.forEach((lab, i) => {
        const b = document.createElement('button');
        b.className = 'small'; b.textContent = i + '로 지정';
        b.title = lab + ' — 이동하지 않고 번호만 바로잡습니다';
        b.onclick = () => cmd({action:'line_set_station', index:i});
        ssb.append(b);
      });
    }
    if (sb) for (let i = 0; i < sb.children.length; i++)
      sb.children[i].className = 'small' + (ln.station === i ? ' on' : '');

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

    const grid = document.getElementById('presets');
    grid.textContent = '';
    for (const s of ((arm.presets || {}).slots || [])) {
      const box = document.createElement('div');
      box.className = 'slot' + (s.filled ? '' : ' empty');
      const head = document.createElement('div'); head.className = 'row-flex';
      const no = document.createElement('span'); no.className = 'no'; no.textContent = s.slot;
      head.append(no);
      if (s.anchor) {
        const t = document.createElement('span'); t.className = 'tag';
        t.textContent = s.anchor.label + ' y=' + Math.round(s.anchor.y); head.append(t);
      }
      const nm = document.createElement('div');
      nm.className = 'nm'; nm.textContent = s.name || (s.filled ? '' : '비어 있음');
      const main = document.createElement('button');
      main.textContent = mode === 'save' ? '● 여기에 저장' : '▶ 재생';
      if (mode === 'play' && !s.filled) main.disabled = true;
      main.onclick = () => slotClick(s.slot);
      const tools = document.createElement('div'); tools.className = 'row-flex';
      for (const [label, fn] of [['이름', rename], ['앵커', anchor], ['비우기', del]]) {
        const b = document.createElement('button');
        b.className = 'small'; b.textContent = label;
        b.disabled = !s.filled && label !== '이름';
        b.onclick = () => fn(s.slot);
        tools.append(b);
      }
      box.append(head, nm, main, tools);
      grid.append(box);
    }
    const anchors = (arm.presets || {}).anchors || [];
    document.getElementById('anchorList').textContent = anchors.length
      ? '앵커(위→아래): ' + anchors.map(a => a.label + '=슬롯' + a.slot + '(y' + Math.round(a.y) + ')').join('  →  ')
      : '앵커 없음 — 슬롯의 [앵커] 버튼으로 상/중/하를 지정하세요.';
  }

  // --- 키보드 홀드 주행 ---
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
    for (const k of held) { const d = KEY_VEC[k]; if (d) { v[0]+=d[0]; v[1]+=d[1]; v[2]+=d[2]; } }
    return v.map(x => Math.max(-1, Math.min(1, x)));
  }
  async function sendHold() {
    const s = +document.getElementById('speed').value;
    const [vx, vy, w] = heldVector().map(x => Math.round(x * s));
    const key = vx + ',' + vy + ',' + w;
    if (key === '0,0,0' && lastSent === '0,0,0') return;
    lastSent = key;
    heldEl.textContent = (vx||vy||w) ? ('  ▶ vx=' + vx + ' vy=' + vy + ' w=' + w) : '';
    try { await fetch('/cmd', {method:'POST', body: JSON.stringify({action:'hold', vx, vy, w})}); }
    catch (e) { /* 다음 주기에 다시 */ }
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
  addEventListener('blur', () => held.clear());
  setMode('play');
"""


def control_page(default_speed: int, frame_height: int) -> str:
    body = (_CONTROL_BODY.replace("__SPEED__", str(default_speed))
            .replace("__FRAME_H__", str(frame_height)))
    return _shell("토마토피커 — 수동 조작", body, _CONTROL_JS, "control")


# ======================================================================
# /settings — 영점·튜닝 (한 번 잡아두고 잘 안 건드리는 것)
# ======================================================================

_SETTINGS_BODY = """
<h2>라인 영점 <span class="dim">— 코스를 새로 깔 때마다</span></h2>
<div class="card">
  <div id="lineState" style="margin-bottom:0.6rem">라인 상태 확인 중...</div>
  <div class="row-flex">
    <button onclick="cmd({action:'line_set_target'})">현재 위치를 기준선으로</button>
    <button onclick="cmd({action:'line_reset_origin'})">변위 0으로</button>
  </div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    로봇을 <b>무대와 올바른 거리</b>에 놓고 [기준선으로]를 누르면 그 거리를 유지하며 주행합니다.
    변위는 양끝 검은 테이프를 지날 때 자동으로 다시 잡힙니다.
  </p>
</div>

<h2>주행 방식 <span class="dim">— 펄스(깔짝깔짝) 조절</span></h2>
<div class="card">
  <label>펄스 속도 <input type="range" id="pSpeed" min="40" max="255" value="150"
    oninput="pSpeedOut.textContent=this.value"> <b id="pSpeedOut">150</b> / 255
    <span class="dim">정지마찰을 확실히 넘도록 빠르게</span></label>
  <label>펄스 ON <input type="range" id="pOn" min="0.02" max="0.6" step="0.01" value="0.08"
    oninput="pOnOut.textContent=this.value"> <b id="pOnOut">0.12</b> 초
    <span class="dim">한 번에 가는 거리</span></label>
  <label>펄스 주기 <input type="range" id="pPeriod" min="0.04" max="1.0" step="0.01" value="0.20"
    oninput="pPeriodOut.textContent=this.value"> <b id="pPeriodOut">0.32</b> 초
  <!-- 메카넘 롤러는 정지 상태에서 옆으로 안 미끄러진다. 바퀴가 굴러야 횡방향
       정지마찰이 깨지므로, 정렬 펄스마다 전후로 살짝 흔들어준다(부호 반전 → 상쇄). -->
  <!-- 저속 연속 주행 속도. 펄스와 값이 다르다 — 펄스는 순간적으로 세게 밀어
       정지마찰을 넘겨야 하지만, 연속 주행은 계속 굴러가므로 낮아도 된다. -->
  <label>저속 주행 속도 <input type="range" id="pSmooth" min="30" max="255" value="105"
    oninput="pSmoothOut.textContent=this.value"> <b id="pSmoothOut">105</b> / 255
    <span class="dim">— 저속 연속 주행일 때 쓰는 속도</span>
  <label>정렬 흔들기 <input type="range" id="pDither" min="0" max="255" step="5" value="150"
    oninput="pDitherOut.textContent=this.value"> <b id="pDitherOut">150</b> / 255
    <span class="dim">— 0이면 없음. 게걸음이 안 먹히면 올리세요(롤러 정지마찰)</span>
    <span class="dim">ON과 같게 두면 <b>연속 주행</b>이 된다</span></label>
  <div class="row-flex">
    <button onclick="applyPulse()">적용</button>
    <button class="small" onclick="preset(150,0.08,0.20)">톡톡(기본)</button>
    <button class="small" onclick="preset(150,0.05,0.16)">더 잘게</button>
    <button class="small" onclick="preset(170,0.04,0.14)">아주 잘게</button>
    <button class="small" onclick="preset(90,0.30,0.30)">연속 저속</button>
    <span id="pNow" class="dim"></span>
  </div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    저속으로 계속 밀면 바퀴가 정지마찰을 겨우 넘나들며 <b>붙었다 풀렸다</b> 해서 갑자기 쭉 미끄러집니다.
    짧고 빠른 펄스를 반복하면 한 걸음 거리가 일정해져 버튼을 톡톡 눌러 맞추는 조작이 됩니다.
  </p>
</div>

<h2>라인 축·부호 <span class="dim">— 손으로 밀어보며 확인</span></h2>
<div class="card">
  <div id="axisState" class="dim" style="margin-bottom:0.5rem"></div>
  <div class="row-flex">
    <button class="small" onclick="cmd({action:'line_flip',what:'dy_axis'})">보정축 vx↔vy</button>
    <button class="small" onclick="cmd({action:'line_flip',what:'dy_sign'})">거리 부호 ±</button>
    <button class="small" onclick="cmd({action:'line_flip',what:'yaw_enable'})">회전 보정 켜기/끄기</button>
    <button class="small" onclick="cmd({action:'line_flip',what:'yaw_sign'})">회전 부호 ±</button>
    <button class="small" onclick="cmd({action:'line_flip',what:'travel_sign'})">진행 좌우 ±</button>
    <button class="small" onclick="cmd({action:'line_flip',what:'smooth'})">주행 방식: 저속연속 ↔ 톡톡</button>
    <button class="small" onclick="cmd({action:'line_flip',what:'no_strafe'})">게걸음 금지 켜기/끄기</button>
    <button class="small" onclick="cmd({action:'line_flip',what:'odom_sign'})">수동이동 방향 ±</button>
  </div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    로봇을 손으로 밀어 <b>테이프에서 멀어졌을 때</b> 위 "보정" 값이 <b>되돌아가는 방향</b>이면 맞습니다.
  </p>
</div>

<h2>모터 튜닝 <span class="dim">— 소음 ↔ 속도</span></h2>
<div class="card">
  <div class="row-flex" style="margin-bottom:0.5rem">
    <button class="small" onclick="tpreset(50,2000)">🔊 50Hz "우우웅"(힘 최대)</button>
    <button class="small" onclick="tpreset(400,2400)">400Hz</button>
    <button class="small" onclick="tpreset(700,2600)">700Hz(균형)</button>
    <button class="small" onclick="tpreset(1100,2900)">1100Hz</button>
    <button class="small" onclick="tpreset(1526,3200)">🔇 1526Hz "찌잉"(상한)</button>
  </div>
  <label>PWM 주파수 <input type="range" id="tHz" min="24" max="1526" value="700"
    oninput="tHzOut.textContent=this.value"> <b id="tHzOut">700</b> Hz</label>
  <label>듀티 상한 <input type="range" id="tPwm" min="800" max="4095" step="25" value="2600"
    oninput="tPwmOut.textContent=this.value"> <b id="tPwmOut">2600</b> / 4095
    <span class="dim">⚠ 키우면 전류도 커진다 — 젯슨이 리셋되면 여기부터 내릴 것</span></label>
  <label>가속(슬루) <input type="range" id="tAcc" min="1" max="40" value="6"
    oninput="tAccOut.textContent=this.value"> <b id="tAccOut">6</b></label>
  <div class="row-flex"><button onclick="applyTune()">적용</button>
    <span id="tNow" class="dim"></span></div>
</div>

<h2>팔 범위 캘리브레이션 <span class="dim">— 관절 가동범위 다시 잡기</span></h2>
<div class="card">
  <div class="row-flex" style="margin-bottom:0.5rem">
    <button onclick="cmd({action:'arm_cal_start'})">① 중앙에서 시작</button>
    <button onclick="cmd({action:'arm_cal_finish'})">② 완료 &amp; 저장</button>
    <button class="small" onclick="cmd({action:'arm_cal_cancel'})">취소</button>
  </div>
  <div id="calState" class="dim">캘리브레이션 대기</div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    ① 팔을 <b>가동범위 한가운데</b>로 손으로 옮긴 뒤 누르세요(그 자세가 원점이 됩니다).
    ② 토크가 풀린 팔의 <b>모든 관절을 끝에서 끝까지</b> 움직이면 아래에 폭이 쌓입니다.
    ③ 다 움직였으면 저장하세요. <code>wrist_roll</code>은 한 바퀴(0~4095) 고정입니다.
    <br>⚠ 저장하면 <b>기존 프리셋 숫자는 다른 자세를 가리키게 됩니다</b> — 다시 교시하세요.
  </p>
</div>
"""

_SETTINGS_JS = """
  let tuneInit = false, pulseInit = false;
  function applyPulse() {
    cmd({action:'line_params',
         speed: +document.getElementById('pSpeed').value,
         pulse_on: +document.getElementById('pOn').value,
         pulse_period: +document.getElementById('pPeriod').value,
         align_dither: +document.getElementById('pDither').value,
         smooth_speed: +document.getElementById('pSmooth').value});
  }
  function preset(sp, on, per) {
    setRange('pSpeed', sp); setRange('pOn', on); setRange('pPeriod', per); applyPulse();
  }
  function setRange(id, v) {
    document.getElementById(id).value = v;
    document.getElementById(id + 'Out').textContent = v;
  }
  function applyTune() {
    cmd({action:'base_tune', hz: +document.getElementById('tHz').value,
         max_pwm: +document.getElementById('tPwm').value,
         accel: +document.getElementById('tAcc').value});
  }
  function tpreset(hz, pwm) { setRange('tHz', hz); setRange('tPwm', pwm); applyTune(); }

  function render() {
    if (!state) return;
    const arm = state.arm || {}, base = state.base || {}, ln = state.line || {};

    const el = document.getElementById('lineState');
    if (ln.found) {
      el.textContent = '테이프 ' + (ln.offset_y_px > 0 ? '아래 ' : '위 ')
        + Math.abs(Math.round(ln.offset_y_px)) + 'px  ·  band_y=' + Math.round(ln.band_y)
        + (ln.angle_deg == null ? '' : '  ·  ∠' + ln.angle_deg + '°')
        + (ln.position_px != null ? '  ·  변위 ' + Math.round(ln.position_px) + 'px' : '');
    } else el.textContent = '⚠ 테이프 없음 — 카메라가 띠를 보게 한 뒤 누르세요'
        + (ln.found_reason ? '  (' + ln.found_reason + ')' : '');

    document.getElementById('axisState').textContent =
      '보정축 ' + ln.dy_axis + '(부호 ' + ln.dy_sign + ')  ·  진행축 ' + ln.travel_axis
      + '(부호 ' + ln.travel_sign + ')  ·  회전 ' + (ln.yaw_gain ? '켜짐(부호 ' + ln.yaw_sign + ')' : '꺼짐')
      + '  ·  지금 보정 ' + (ln.dy_axis === 'vy' ? ln.would_vy : ln.would_vx)
      + ' w=' + ln.would_w
      + '  ·  주행 ' + (ln.smooth ? '저속연속(vy·w 비례보정)' : '펄스(톡톡)')
      + '  ·  게걸음 ' + (ln.smooth ? '보정에 사용'
                                    : (ln.no_strafe ? '금지(벗어나면 정렬)' : '허용'))
      + '  ·  진행방향 ' + (ln.last_dir > 0 ? '오른쪽' : (ln.last_dir < 0 ? '왼쪽' : '미확인'))
      + '(' + (ln.dir_source || '?') + ')';

    if (!pulseInit && ln.speed != null) {
      pulseInit = true;
      setRange('pSpeed', Math.round(ln.speed));
      setRange('pOn', ln.pulse_on); setRange('pPeriod', ln.pulse_period);
      if (ln.align_dither != null) setRange('pDither', Math.round(ln.align_dither));
      if (ln.smooth_speed != null) setRange('pSmooth', Math.round(ln.smooth_speed));
    }
    document.getElementById('pNow').textContent = ln.speed == null ? '' :
      ('현재: 속도 ' + Math.round(ln.speed) + ' · '
       + (ln.pulsing ? '펄스 ' + ln.pulse_on + 's / ' + ln.pulse_period + 's' : '연속 주행')
       + '  ·  정렬 흔들기 ' + Math.round(ln.align_dither || 0)
       + '  ·  주행 ' + (ln.smooth ? '저속연속 ' + Math.round(ln.smooth_speed || 0)
                                   : '펄스(톡톡)'));

    const t = base.tuning;
    if (t) {
      document.getElementById('tNow').textContent =
        '요청값 ' + t.hz + 'Hz · 듀티 ' + t.max_pwm + ' · 가속 ' + t.accel
        + (base.acks && base.acks.length ? '   ← 보드 확인: ' + base.acks.join(' | ') : '   ← 보드 확인 없음');
      if (!tuneInit) {
        tuneInit = true;
        setRange('tHz', t.hz); setRange('tPwm', t.max_pwm); setRange('tAcc', t.accel);
      }
    }

    const calEl = document.getElementById('calState');
    const cal = arm.calibration;
    if (!cal || !cal.active) {
      calEl.textContent = '캘리브레이션 대기 — ①을 누르면 지금 자세가 원점이 됩니다';
      calEl.style.background = '';
    } else {
      calEl.textContent = '● 기록 중 — ' + (cal.joints || []).map(j =>
        j.name + ': ' + (j.full_turn ? '한바퀴 고정'
          : j.min + '~' + j.max + ' (폭 ' + j.span + (j.span < 100 ? ' ⚠아직' : '') + ')')
      ).join('  ·  ') + (cal.error ? '  ⚠' + cal.error : '');
      calEl.style.background = 'color-mix(in srgb, #f39c12 25%, Canvas)';
    }
  }
"""


def settings_page() -> str:
    return _shell("토마토피커 — 시스템 설정", _SETTINGS_BODY, _SETTINGS_JS, "settings")


# ======================================================================
# /diag — 링크 실시간 계측 (SSE 10Hz)
# ======================================================================
#
# 이 화면만 폴링이 아니라 SSE인 이유: 1초 폴링으로는 80ms 펄스가 통째로
# 사라진다. 서버(linkmon.LinkSampler)가 100Hz로 뜬 파형을 0.1초마다 묶어
# 밀어준다 — 브라우저가 느려도 파형에 구멍이 나지 않는다.

_DIAG_BODY = """
<div class="card" id="conn">스트림 연결 중...</div>
<div class="mgrid" id="grid"></div>
<div class="card dim" id="acks"></div>

<h2>지령 파형 <span class="dim">— 젯슨이 내보내는 목표 속도 (최근 10초)</span></h2>
<div class="legend">
  <span><i class="sw" style="background:#2ecc71"></i>vx 전후</span>
  <span><i class="sw" style="background:#3498db"></i>vy 게걸음</span>
  <span><i class="sw" style="background:#e67e22"></i>w 회전</span>
</div>
<canvas id="wave"></canvas>

<h2>펄스 폭 <span class="dim">— 한 번의 "톡"이 실제로 몇 ms 나갔나</span></h2>
<div id="pulses">아직 펄스 없음 — 주행을 시작하면 여기 찍힙니다</div>
"""

_DIAG_JS = """
  const conn = document.getElementById('conn');
  const grid = document.getElementById('grid');
  const cv = document.getElementById('wave');
  const SPAN = 10.0;                 // 파형 표시 구간(초)
  let wave = [];

  document.getElementById('out').textContent =
    '읽기 전용 화면입니다 — 조작은 [수동 조작], 튜닝은 [시스템 설정]에서.';

  function card(lbl, val, sub, cls) {
    return '<div class="m ' + (cls || '') + '"><div class="lbl">' + lbl + '</div>'
         + '<div class="val">' + val + '</div>'
         + '<div class="sub">' + (sub || '') + '</div></div>';
  }
  function upt(ms) {
    if (ms === null || ms === undefined) return '—';
    const t = Math.floor(ms / 1000);
    return Math.floor(t / 60) + '분 ' + ('0' + (t % 60)).slice(-2) + '초';
  }

  function renderCards(d) {
    const s = d.stats || {}, r = d.rates || {};
    const ok = s.connected;
    conn.textContent = (ok ? '● 링크 정상' : '○ 링크 끊김')
      + '  ·  ' + (s.port || '포트 없음')
      + (s.hb_age == null ? '' : '  ·  하트비트 ' + s.hb_age.toFixed(2) + 's 전')
      + '  ·  ' + d.clock + '  ·  ' + d.sample_hz + 'Hz 샘플링'
      + (s.error ? '   ⚠ ' + s.error : '');
    conn.style.background = ok ? 'color-mix(in srgb, #2ecc71 18%, Canvas)'
                               : 'color-mix(in srgb, #e74c3c 18%, Canvas)';

    // ★ 이 화면에서 제일 중요한 숫자. 주행 중엔 20ms마다 보내므로 보드의 rx
    //    카운터가 초당 50씩 늘어야 한다. 그보다 낮으면 젯슨 송신 스레드가 밀린
    //    것이고, 그게 곧 펄스가 들쭉날쭉한 이유다.
    //    ⚠ 정지 중 기대치는 10/s다(MotorLink가 IDLE_INTERVAL_SEC로 늘어진다).
    //    이걸 구분 안 하면 세워둔 로봇이 늘 빨간불이라 경고가 무의미해진다.
    let h = '';
    if (r.rx == null) {
      h += card('수신율', '—', '데이터 모으는 중');
    } else {
      const exp = d.expect_rx, moving = d.driving;
      const cls = r.rx >= exp * 0.9 ? 'ok' : (r.rx >= exp * 0.6 ? 'warn' : 'bad');
      const note = cls === 'ok' ? (moving ? '주행 중 · 정상' : '정지 중 · 정상')
                 : (cls === 'warn' ? '⚠ 젯슨 송신 지연' : '⚠ 심각 — 지연/단절');
      h += card('수신율 (보드 rx)', r.rx.toFixed(1) + '<small>/s</small>',
                '기대 ' + exp + '/s · ' + note, cls);
    }
    h += card('깨진 프레임', (s.fw_bad || 0),
              (r.bad ? r.bad.toFixed(1) + '/s 증가 중' : '증가 없음'),
              s.fw_bad ? (r.bad ? 'bad' : 'warn') : 'ok');
    h += card('거부(nak)', (s.nak || 0),
              (r.nak ? r.nak.toFixed(1) + '/s 증가 중' : '증가 없음'),
              s.nak ? (r.nak ? 'bad' : 'warn') : 'ok');
    // ★ 재부팅 원인. v2.1 펌웨어가 배너 직전에 뱉는 boot 줄에서 왔다.
    //    WDT   = 워치독 → loop가 250ms 막혔다(펌웨어/I2C 문제. 전원 아님)
    //    POWER = SRAM이 깨졌다 → 전원이 실제로 나갔다(전원계 문제)
    //    EXT|BOD = 리셋핀(DTR=우리가 포트를 연 것) 또는 브라운아웃
    const causes = s.boot_causes || {};
    const ckeys = Object.keys(causes);
    const worst = causes.WDT ? 'bad' : (causes.POWER ? 'bad' : (s.board_resets ? 'warn' : 'ok'));
    h += card('보드 재부팅', (s.board_resets || 0),
              ckeys.length ? ckeys.map(k => k + ' ' + causes[k] + '회').join(' · ')
                           : (s.board_resets ? '원인 미상(구 펌웨어)' : '없음'),
              worst);
    // I2C가 물린 횟수 = TWI 버스에 모터 노이즈가 타는 직접 증거.
    h += card('I2C 버스 물림', (s.i2c_err || 0),
              s.i2c_err ? '⚠ 모터 노이즈가 I2C에 탑니다' : '없음',
              s.i2c_err ? 'warn' : 'ok');
    // 워치독이 물었다가 살아난 횟수. 재부팅은 안 났지만 loop가 250ms 막혔다는 뜻.
    h += card('loop 막힘(근접)', (s.wdt_near || 0),
              s.wdt_near ? '⚠ 250ms 막혔다 복구됨' : '없음',
              s.wdt_near ? 'warn' : 'ok');
    h += card('링크 복구', (s.hb_resets || 0),
              s.hb_resets ? '⚠ 하트비트 끊겨 재연결함' : '없음',
              s.hb_resets ? 'warn' : 'ok');
    h += card('보드 uptime', upt(s.fw_ms), '되감기면 재부팅한 것');
    // 샘플러가 100Hz 스케줄에서 밀린 정도 = MotorLink의 20ms 틱이 밀리는 것과
    // 같은 원인(GIL·CPU 부하)을 본다. 20ms를 넘으면 지령 간격이 이미 깨진다.
    h += card('젯슨 지연', d.sample_lag_ms.toFixed(1) + '<small>ms</small>',
              d.sample_lag_ms > 20 ? '⚠ 지령 간격이 흔들립니다' : '샘플러 스케줄 밀림(최대)',
              d.sample_lag_ms > 20 ? 'bad' : (d.sample_lag_ms > 8 ? 'warn' : 'ok'));
    const tg = s.target || [0, 0, 0];
    h += card('현재 지령', tg[0] + ' / ' + tg[1] + ' / ' + tg[2], 'vx / vy / w');
    const t = s.tuning;
    if (t) h += card('보드 튜닝(요청)', t.hz + '<small>Hz</small>',
                     '듀티 ' + t.max_pwm + ' · 슬루 ' + t.accel + '/' + t.decel);
    grid.innerHTML = h;

    const acks = document.getElementById('acks');
    const ackTxt = (s.acks && s.acks.length)
      ? '보드 확인 응답: ' + s.acks.join('  |  ')
      : '보드 확인 응답 없음 — 설정(F/P/R)이 아직 안 먹었을 수 있습니다';
    // 마지막 리셋 원인 원문도 같이 — 판정이 어디서 나왔는지 눈으로 확인 가능하게.
    // ⚠ 이 JS는 파이썬 삼중따옴표 문자열 안에 들어 있다. 그래서 여기 쓰는
    //    역슬래시-n은 반드시 한 번 더 이스케이프해야 한다 — 안 그러면 파이썬이
    //    진짜 개행으로 바꿔버려 JS 문자열 리터럴이 그 자리에서 깨진다.
    acks.textContent = ackTxt + (s.boot_report ? '\\n마지막 부팅: ' + s.boot_report : '');
    acks.style.whiteSpace = 'pre-wrap';
  }

  function renderPulses(d) {
    const el = document.getElementById('pulses');
    const ps = d.pulses || [], st = d.pulse_stat;
    if (!ps.length) { el.textContent = '아직 펄스 없음 — 주행을 시작하면 여기 찍힙니다'; return; }
    let h = '<div>최근 ON 폭(ms): ';
    for (const p of ps) h += '<span class="pw">' + p.on_ms.toFixed(0) + '</span>';
    h += '</div>';
    if (st) {
      // ★ 여기가 "톡톡이 매번 다르다"의 정량. 편차가 크면 펄스 길이가
      //    양자화(LineDriver 20ms + MotorLink 20ms)에 먹히고 있는 것이다.
      const spread = st.max - st.min;
      const pct = st.avg ? Math.round(100 * spread / st.avg) : 0;
      h += '<div>최소 <b>' + st.min.toFixed(0) + '</b> · 평균 <b>' + st.avg.toFixed(0)
         + '</b> · 최대 <b>' + st.max.toFixed(0) + '</b> ms  →  편차 <b>'
         + spread.toFixed(0) + 'ms (±' + pct + '%)</b>'
         + (pct > 40 ? '  ⚠ 한 걸음의 크기가 매번 다릅니다' : '  · 고름') + '</div>';
    }
    const withGap = ps.filter(p => p.gap_ms != null);
    if (withGap.length) {
      const gaps = withGap.map(p => p.gap_ms);
      const per = withGap.map(p => p.gap_ms + p.on_ms);
      const mn = a => Math.min.apply(null, a).toFixed(0);
      const mx = a => Math.max.apply(null, a).toFixed(0);
      h += '<div class="dim">쉼(OFF) ' + mn(gaps) + '~' + mx(gaps) + 'ms · 주기 '
         + mn(per) + '~' + mx(per) + 'ms · 펄스 중 최대지령 '
         + Math.max.apply(null, ps.map(p => p.peak)) + '</div>';
    }
    h += '<div class="dim">※ <b>젯슨이 내보낸 지령</b> 기준입니다(보드가 실제 인가한 '
       + '속도가 아님). 샘플링 ' + d.sample_hz + 'Hz라 ±'
       + Math.round(1000 / d.sample_hz) + 'ms 양자화가 섞여 있습니다.</div>';
    el.innerHTML = h;
  }

  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const W = cv.clientWidth, H = cv.clientHeight;
    if (!W || !H) return;
    if (cv.width !== Math.round(W * dpr)) {
      cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr);
    }
    const g = cv.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    if (!wave.length) return;
    const tEnd = wave[wave.length - 1][0], tStart = tEnd - SPAN;
    const mid = H / 2, scale = (H / 2 - 10) / 255;
    g.strokeStyle = 'rgba(128,128,128,0.35)'; g.lineWidth = 1;
    g.beginPath(); g.moveTo(0, mid); g.lineTo(W, mid); g.stroke();
    g.setLineDash([3, 4]);
    for (const v of [255, 128, -128, -255]) {
      const y = mid - v * scale;
      g.beginPath(); g.moveTo(0, y); g.lineTo(W, y); g.stroke();
    }
    for (let k = Math.ceil(tStart); k < tEnd; k++) {       // 1초 눈금
      const x = (k - tStart) / SPAN * W;
      g.beginPath(); g.moveTo(x, 0); g.lineTo(x, H); g.stroke();
    }
    g.setLineDash([]);
    // 계단(step)으로 그린다 — 지령은 다음 갱신까지 유지되는 값이라
    // 직선으로 이으면 없던 램프가 생겨 펄스가 실제보다 부드러워 보인다.
    const cols = ['#2ecc71', '#3498db', '#e67e22'];
    for (let i = 0; i < 3; i++) {
      g.strokeStyle = cols[i]; g.lineWidth = 1.8;
      g.beginPath();
      let px = null, py = null;
      for (const s of wave) {
        if (s[0] < tStart) continue;
        const x = (s[0] - tStart) / SPAN * W, y = mid - s[1 + i] * scale;
        if (px === null) { g.moveTo(x, y); } else { g.lineTo(x, py); g.lineTo(x, y); }
        px = x; py = y;
      }
      g.stroke();
    }
    g.fillStyle = 'rgba(128,128,128,0.9)'; g.font = '11px sans-serif';
    g.fillText('+255', 4, mid - 255 * scale + 11);
    g.fillText('0', 4, mid - 4);
    g.fillText('-255', 4, mid + 255 * scale - 4);
  }

  function apply(d) {
    if (d.wave && d.wave.length) {
      for (const s of d.wave) wave.push(s);
      // 표시 구간보다 1초 넉넉히만 들고 있는다(브라우저 메모리 무한증가 방지).
      const cut = wave[wave.length - 1][0] - SPAN - 1;
      while (wave.length && wave[0][0] < cut) wave.shift();
    }
    renderCards(d); renderPulses(d); draw();
  }

  // EventSource는 끊기면 브라우저가 알아서 재연결한다 — 젯슨을 재시작해도
  // 탭을 새로고침할 필요가 없다(데모 중에 이게 크다).
  const es = new EventSource('/diag-events');
  es.onmessage = (e) => { try { apply(JSON.parse(e.data)); } catch (err) { /* 다음 프레임 */ } };
  es.onerror = () => {
    conn.textContent = '○ 스트림 끊김 — 재연결 중...';
    conn.style.background = 'color-mix(in srgb, #e74c3c 18%, Canvas)';
  };
  window.addEventListener('resize', draw);
"""


def diag_page() -> str:
    # common=False: 1초 /status 폴링을 끈다. SSE와 겹치면 낭비이고,
    # 그 폴링 부하가 이 화면이 재려는 "젯슨 지연"에 섞여 들어간다.
    return _shell("토마토피커 — 링크 실시간 계측", _DIAG_BODY, _DIAG_JS, "diag", common=False)

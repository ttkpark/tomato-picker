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
    <button onclick="cmd({action:'line_align_mark'})">📍 지점 정렬</button>
    <span class="dim">정렬=기준선·평행 · 지점 정렬=마커를 화면 중앙에(도착 시 자동)</span>
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

<h2>시퀀스 <span class="dim">— 팔 프리셋 + 지점 이동을 섞은 대본</span></h2>
<div class="card">
  <div id="seqState" style="margin-bottom:0.6rem">시퀀스 상태 확인 중...</div>
  <!-- 실행 버튼과 편집칸은 **서버가 주는 키로** 만든다 — 대본이 늘어도 여기 안 고친다
       (음성 "위/아래 토마토"가 쓰는 '위'·'아래' 대본이 이렇게 자동으로 나온다) -->
  <div class="row-flex" style="margin-bottom:0.5rem">
    <span id="seqRunBtns"></span>
    <button class="stop" onclick="cmd({action:'seq_stop'})">■ 시퀀스 정지</button>
  </div>
  <div id="seqEditRows"></div>
  <p class="dim" style="margin:0.4rem 0 0; font-size:0.85rem">
    <b>숫자</b>=팔 프리셋 · <b>m숫자</b>=지점 이동 · <b>w초</b>=대기 &nbsp;
    예) <code>m2 1 2 3 7 m0 8 9 0</code> = 지점2(종착)로 → 2층 따기 → 지점0(바구니) → 놓기 → 대기.
    지점 이동은 <b>도착할 때까지 기다렸다가</b> 다음 단계로 갑니다(주행 중 팔이 움직이지 않게).
    이동이 실패하면 거기서 멈춥니다 — 엉뚱한 위치에서 팔을 뻗지 않도록.
  </p>
</div>

<h2>리더암 <span class="dim">— 자세 만들기 · 팔이 안 움직이면 →</span>
  <button class="small" onclick="cmd({action:'service_restart',name:'tomato-voice'})">🦾 팔 서비스 재시작</button>
</h2>
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
  // 대본 편집칸을 키로 찾는다 — 키가 한글("위")이라 DOM id에 이어 붙이지 않고
  // 여기에 담아 둔다.
  const seqInputs = {};
  let seqKeys = '';
  function saveSeq(key) {
    const inp = seqInputs[key];
    if (inp) cmd({action:'seq_save', key: key, text: inp.value});
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

    // 시퀀스 — 실행 중이면 진행 단계를, 아니면 대본 설명을 보여준다.
    const sq = state.seq || {};
    const se = document.getElementById('seqState');
    if (se) {
      const defs = sq.sequences || {};
      const keys = Object.keys(defs);
      // 대본 목록이 바뀌었을 때만 다시 그린다 — 매 틱마다 새로 만들면 타이핑 중인
      // 편집칸이 초기화된다.
      if (keys.length && keys.join('\\u0000') !== seqKeys) {
        seqKeys = keys.join('\\u0000');
        const runs = document.getElementById('seqRunBtns');
        const rows = document.getElementById('seqEditRows');
        runs.textContent = ''; rows.textContent = '';
        for (const k of keys) {
          const b = document.createElement('button');
          b.textContent = '▶ ' + k;
          b.onclick = () => cmd({action:'seq_start', key:k});
          runs.append(b);

          const lab = document.createElement('label');
          lab.append(k + ' ');
          const inp = document.createElement('input');
          inp.type = 'text'; inp.style.width = 'min(420px,80vw)';
          inp.value = defs[k].text || '';
          seqInputs[k] = inp;
          const save = document.createElement('button');
          save.className = 'small'; save.textContent = '저장';
          save.onclick = () => saveSeq(k);
          lab.append(inp, ' ', save);
          rows.append(lab);
        }
      }
      if (sq.running) {
        se.textContent = '▶ 시퀀스 ' + sq.running + ' 실행 중 — '
                       + (sq.step || 0) + '/' + (sq.total || 0) + ' · ' + (sq.detail || '');
        se.style.background = 'color-mix(in srgb, #2ecc71 20%, Canvas)';
      } else {
        const parts = [];
        for (const k of keys) {
          parts.push(k + ': ' + (defs[k].error ? '⚠ ' + defs[k].error : defs[k].desc));
        }
        se.textContent = (sq.detail && sq.detail !== '대기' ? '지난 실행: ' + sq.detail + '  ·  ' : '')
                       + parts.join('   |   ');
        se.style.background = '';
      }
    }

    const el = document.getElementById('lineState');
    if (!ln || ln.mode === 'off') { el.textContent = ln.error || '라인 주행 비활성'; }
    else {
      const p = [ln.mode === 'idle' ? '⏸ 대기' : '▶ ' + ln.mode, ln.detail || ''];
      if (ln.found) {
        const dy = Math.round(ln.offset_y_px);
        p.push('테이프 ' + (dy > 0 ? '아래 ' : '위 ') + Math.abs(dy) + 'px'
               + (ln.angle_deg == null ? '' : ' ∠' + ln.angle_deg + '°'));
        if (ln.mark_dx != null) p.push('마커 중앙오차 ' + Math.round(ln.mark_dx) + 'px');
        if (ln.markers_dropped) p.push('⚠ 코스밖 색덩어리 ' + ln.markers_dropped + '개 무시');
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
<h2>호출어 <span class="dim">— 부른 뒤에만 명령을 받는다</span></h2>
<div class="card">
  <div id="wakeState" style="margin-bottom:0.6rem">호출어 상태 확인 중...</div>
  <div class="row-flex">
    <label><input type="checkbox" id="wOn" onchange="applyWake()"> 호출어 사용</label>
    <label>대기 시간 <input type="number" id="wSec" min="3" max="300" step="1"
      style="width:5rem"> 초 <button class="small" onclick="applyWake()">적용</button></label>
  </div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    마이크는 늘 켜져 있고 부스에는 사람이 말합니다. 낱말을 아무리 잘 골라도 옆 대화가
    언젠가는 명령으로 읽히는데, 그때 <b>실제로 바퀴가 구릅니다</b>. 부른 뒤에만 받게 하면
    그 사고가 구조적으로 막힙니다.
    <br>한 번에 말해도 됩니다 — <code>안녕 아래 토마토 따줘</code>. 명령을 하나 받을 때마다
    대기 시간이 다시 채워지므로 연속 조작 중에 매번 부를 필요는 없습니다.
    <br>호출어가 자꾸 안 걸리면 아래 [호출어] 칸에 실제로 들린 말을 넣거나, 여기서 꺼서
    예전처럼 항상 듣게 하세요(끈 상태도 대시보드 배지에 표시됩니다).
  </p>
</div>

<h2>음성 명령어 <span class="dim">— 안 걸리는 말은 여기에 추가한다</span></h2>
<div class="card">
  <div class="row-flex" style="margin-bottom:0.6rem">
    <input type="text" id="vTest" placeholder="들린 말을 넣어 시험 (예: 아래 토마토 따줘)"
      style="width:min(360px,70vw)" onkeydown="if(event.key==='Enter')testVoice()">
    <button onclick="testVoice()">시험</button>
    <button class="small" onclick="if(confirm('모든 명령어를 기본값으로 되돌릴까요?'))
      cmd({action:'voice_words_reset'})">전체 기본값</button>
  </div>
  <div id="voiceRows"></div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    쉼표로 구분합니다. <b>많이 적을 필요 없습니다</b> — 발음을 접어서 비교하므로
    "토마토" 하나로 <code>도마도·또마또·도마토</code>가 다 걸립니다. 로그에 찍힌 오인식이
    계속 안 걸릴 때만 그 말을 그대로 추가하세요.
    저장하면 <b>즉시 반영</b>됩니다(재시작 불필요, <code>~/voice_words.json</code>에 남습니다).
    <br>⚠ 짧고 흔한 말은 넣지 마세요 — 잡담이 명령이 되어 바퀴가 구릅니다.
    넣기 전에 위 [시험] 칸으로 확인하는 게 안전합니다.
  </p>
</div>

<h2>서비스 <span class="dim">— 안 움직일 때 여기서 되살린다 (ssh 불필요)</span></h2>
<div class="card">
  <div class="row-flex">
    <button onclick="cmd({action:'service_restart',name:'tomato-voice'})">🦾 팔·음성·바퀴 재시작</button>
    <button class="small" onclick="cmd({action:'service_restart',name:'line-follow'})">라인 검출 재시작</button>
    <button class="small" onclick="cmd({action:'service_restart',name:'line-cam'})">바닥 카메라 재시작</button>
    <button class="small" onclick="cmd({action:'service_restart',name:'tomato-vision'})">토마토 비전 재시작</button>
    <button class="small" onclick="cmd({action:'service_status'})">상태 확인</button>
  </div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    <b>팔이 안 움직이면</b> [🦾 팔·음성·바퀴 재시작] — 팔·바퀴 연결은 tomato-voice가 들고
    있어서 이게 곧 재연결입니다. 재시작하면 이 화면이 몇 초 끊겼다 돌아옵니다(새로고침 불필요).
    게임패드(controller-drive)는 여기 없습니다 — tomato-voice와 모터 포트를 다투므로
    둘 중 하나만 켜야 합니다.
  </p>
</div>

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
  <!-- 저속 연속 주행의 정지마찰 대책. vx만으로는 바퀴가 안 풀려서(파형은 평평한데
       로봇은 제자리) 출발엔 세게 한 방, 주행 중엔 주기적으로 옆으로 톡 친다. -->
  <label>출발 한 방 <input type="range" id="pKick" min="0" max="255" step="5" value="200"
    oninput="pKickOut.textContent=this.value"> <b id="pKickOut">200</b> / 255
    <span class="dim">— 출발 0.3초만 이 속도(정지마찰 깨기). 0이면 없음</span>
  <label>주행 좌우 톡 <input type="range" id="pWiggle" min="0" max="255" step="5" value="130"
    oninput="pWiggleOut.textContent=this.value"> <b id="pWiggleOut">130</b> / 255
    <span class="dim">— 0.5초마다 0.1초씩 좌우로 톡(부호 교대라 제자리). 안 나가면 올리세요</span>
  <!-- 게걸음은 롤러가 옆으로 굴러야 생겨 가장 잘 걸리는 축이다. 회전은 네 바퀴가
       모두 제 축으로 도는 동작이라 정지마찰이 훨씬 쉽게 풀린다 — 같이 쓴다. -->
  <label>흔들 때 회전 <input type="range" id="pWigYaw" min="0" max="255" step="5" value="60"
    oninput="pWigYawOut.textContent=this.value"> <b id="pWigYawOut">60</b> / 255
    <span class="dim">— 옆으로 톡 칠 때 같이 낼 회전(부호 교대라 제자리). 0이면 게걸음만</span>
  <label>정렬 흔들기 <input type="range" id="pDither" min="0" max="255" step="5" value="150"
    oninput="pDitherOut.textContent=this.value"> <b id="pDitherOut">150</b> / 255
    <span class="dim">— <b>진행축</b> 흔들기(게걸음 아님). 0이면 없음. 옆으로 안 먹히면 올리세요</span>
  <!-- ⚠ 게걸음 크기는 속도 슬라이더와 **무관**하다. 정렬 중 옆으로 미는 값은
       _corr_pulse가 이 하한~상한 사이에서 정하며, 진행축 speed는 안 곱해진다.
       "속도를 0으로 해도 게걸음이 크다"(2026-08-13 현장)의 답이 여기다. -->
  <!-- ⚠ 게걸음이 나가는 길은 **둘**이다. 상한만 공통이고 나머지는 서로 다르다.
         정렬(제자리) = _corr_pulse : corr_min ~ corr_max 사이의 펄스
         주행(저속연속) = _corr_smooth : dy × smooth_dy_gain, corr_max에서 잘림
       예전엔 아래 두 슬라이더가 펄스 경로만 건드려서, 저속연속으로 주행 중일 때는
       아무리 내려도 변화가 없었다(2026-08-13 현장 보고). -->
  <label>게걸음 최소 <span class="dim">(정렬 전용)</span> <input type="range" id="pCorrMin" min="30" max="255" step="5" value="130"
    oninput="pCorrMinOut.textContent=this.value"> <b id="pCorrMinOut">130</b> / 255
    <span class="dim">— <b>제자리 정렬</b>에서 옆으로 한 번 미는 크기의 하한.
    너무 내리면 정지마찰을 못 넘어 아예 안 움직입니다</span>
  <label>게걸음 최대 <span class="dim">(정렬·주행 공통)</span> <input type="range" id="pCorrMax" min="30" max="255" step="5" value="180"
    oninput="pCorrMaxOut.textContent=this.value"> <b id="pCorrMaxOut">180</b> / 255
    <span class="dim">— 두 경로 모두의 상한. 한 걸음 거리 = 크기 × 펄스 ON</span>
  <label>주행 중 게걸음 세기 <input type="range" id="pSmoothGain" min="20" max="600" step="10" value="300"
    oninput="pSmoothGainOut.textContent=this.value"> <b id="pSmoothGainOut">300</b>
    <span class="dim">— <b>저속 연속 주행</b>에서 dy 오차에 곱하는 이득(비례제어).
    300이면 62px 벗어났을 때 게걸음 52가 나갑니다. <b>주행 중 좌우로 크게 흔들리면 여기를 내리세요</b></span>
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
    <!-- 지그재그(주행 좌우 톡 · 굳음 해제 직각 흔들기)가 **어느 쪽부터** 나가는가.
         교대라 결국 제자리지만 첫 반 주기는 한쪽으로 나가므로, 그쪽이 토마토
         나무면 매번 부딪힌다. 어느 부호가 안전한지는 실기로만 안다. -->
    <button class="small" onclick="cmd({action:'line_flip',what:'wiggle_sign'})">흔들기 시작쪽 ±</button>
    <!-- 두 칸 이상 이동할 때 중간 지점을 다 들를지. 중간에서는 정렬을 건너뛴다. -->
    <button class="small" onclick="cmd({action:'line_flip',what:'stop_each'})">중간지점 들르기 켜기/끄기</button>
  </div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    로봇을 손으로 밀어 <b>테이프에서 멀어졌을 때</b> 위 "보정" 값이 <b>되돌아가는 방향</b>이면 맞습니다.
  </p>
</div>

<h2>정렬 목표 범위 <span class="dim">— "이 안에 들어오면 그만"</span></h2>
<div class="card">
  <!-- 이 값들은 **정렬을 끝낼지**의 기준이다(주행 중 보정을 낼지의 데드밴드와 다름).
       좁으면 정착 후 재측정에서 조금만 밀려도 정렬이 다시 시작돼 끝나지 않는다. -->
  <label>x 허용(진행축) <input type="range" id="rTolX" min="5" max="200" step="5" value="50"
    oninput="rTolXOut.textContent=this.value"> ±<b id="rTolXOut">50</b> px
    <span class="dim">— 마커가 화면 중앙에서 이 안이면 맞은 것</span></label>
  <label>y 허용 — 테이프 <b>아래</b> <input type="range" id="rDyBelow" min="5" max="200" step="5" value="50"
    oninput="rDyBelowOut.textContent=this.value"> <b id="rDyBelowOut">50</b> px
    <span class="dim">— 테이프가 기준선보다 아래로 이만큼까지 허용</span></label>
  <label>y 허용 — 테이프 <b>위</b> <input type="range" id="rDyAbove" min="5" max="200" step="5" value="20"
    oninput="rDyAboveOut.textContent=this.value"> <b id="rDyAboveOut">20</b> px
    <span class="dim">— 위로 벗어나면 시야 밖이라 좁게 두는 쪽</span></label>
  <!-- ⚠ 너무 좁게 잡으면 **끝나지 않는다** — 각도는 바닥 띠에서 추정하는 값이라
       프레임마다 흔들리고, 제자리 회전은 롤러 정지마찰 때문에 잘게 못 준다.
       수렴 못 하면 타임아웃으로 끝나는데 그때는 각도가 얼마든 상관없이 멈춘다.
       닿을 수 있는 값을 두는 편이 각도를 **더** 잘 보장한다. -->
  <label>회전 허용 <input type="range" id="rYaw" min="0.5" max="20" step="0.5" value="10"
    oninput="rYawOut.textContent=this.value"> ±<b id="rYawOut">10</b> °
    <span class="dim">⚠ x·y가 다 맞아도 이게 좁으면 정렬이 안 끝난다</span></label>
  <div class="row-flex"><button onclick="applyRange()">적용</button>
    <button class="small" onclick="rpreset(50,50,20,2)">기본(50 / 50·20 / 2°)</button>
    <button class="small" onclick="rpreset(70,80,40,5)">넉넉히</button>
    <span id="rNow" class="dim"></span></div>
  <p class="dim" style="margin:0.6rem 0 0; font-size:0.85rem">
    완료 판정은 x · y · 회전 <b>셋 다</b> 들어와야 합니다(가장 나쁜 축이 기준).
    지금 값은 위 <b>라인 영점</b> 카드의 "테이프 아래/위 NNpx"와 같은 단위입니다 —
    거기 숫자를 보면서 맞추세요.
  </p>
</div>

<h2>모터 튜닝 <span class="dim">— 소음 ↔ 속도</span></h2>
<div class="card">
  <!-- 전체 속도 배율. V 지령의 유일한 길목(MotorLink.set_velocity)에 곱해지므로
       라인 주행·수동 조작·음성·시퀀스가 **전부** 같은 비율로 느려진다. 개별
       속도 슬라이더를 하나씩 만지지 않아도 되게 만든 지표다. -->
  <label><b>전체 속도</b> <input type="range" id="tScale" min="20" max="150" step="5" value="100"
    oninput="tScaleOut.textContent=this.value"> <b id="tScaleOut">100</b> %
    <span class="dim">— <b>모든 움직임</b>(라인 주행·수동·음성·시퀀스)에 한 번에 곱해집니다.
    100 = 지금 설정 그대로. ⚠ 너무 낮추면 정지마찰을 못 넘어 아예 안 움직입니다(하한 20%)</span></label>
  <div class="row-flex" style="margin-bottom:0.5rem">
    <button class="small" onclick="spreset(60)">60% 느리게</button>
    <button class="small" onclick="spreset(80)">80%</button>
    <button class="small" onclick="spreset(100)">100% 기본</button>
  </div>
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
  let tuneInit = false, pulseInit = false, rangeInit = false;
  // --- 음성 명령어 ---
  // 칸은 서버가 주는 목록으로 만든다(코드에 항목을 또 적지 않게). 타이핑 중에
  // 폴링이 값을 덮어쓰면 안 되므로 **한 번만** 그리고, 그 뒤로는 '고침' 표시만 갱신한다.
  const vInputs = {};
  let wakeInit = false;
  function applyWake() {
    cmd({action:'voice_wake', enabled: document.getElementById('wOn').checked,
         window_sec: +document.getElementById('wSec').value});
  }
  function renderWake(w) {
    const el = document.getElementById('wakeState');
    if (!el || !w || w.enabled === undefined) return;
    // 칸은 **한 번만** 채운다 — 1초 폴링이 매번 덮으면 값을 고칠 수가 없다.
    if (!wakeInit) {
      wakeInit = true;
      document.getElementById('wOn').checked = !!w.enabled;
      document.getElementById('wSec').value = Math.round(w.window_sec);
    }
    const ws = (w.wake_words || []).join(', ') || '(비어 있음 — 부를 수가 없습니다)';
    el.textContent = !w.enabled
      ? '🎤 항상 듣는 중 (호출어 꺼짐) — 부르지 않아도 명령이 바로 실행됩니다'
      : (w.remaining_sec > 0
          ? '🎤 듣는 중 — ' + Math.round(w.remaining_sec) + '초 남음  ·  호출어: ' + ws
          : '💤 대기 중 — 호출어: ' + ws);
    el.style.background = (!w.enabled || w.remaining_sec > 0)
      ? 'color-mix(in srgb, #2ecc71 20%, Canvas)' : '';
  }
  function testVoice() {
    const t = document.getElementById('vTest').value.trim();
    if (t) cmd({action:'voice_test', text:t});
  }
  function saveWords(key) {
    cmd({action:'voice_words_save', key:key, text:vInputs[key].value});
  }
  function renderVoice(items) {
    const box = document.getElementById('voiceRows');
    if (!box || !items || !items.length) return;
    if (box.dataset.n !== String(items.length)) {
      box.dataset.n = String(items.length);
      box.textContent = '';
      for (const it of items) {
        const row = document.createElement('div');
        row.style.margin = '0.45rem 0';
        const head = document.createElement('div');
        head.innerHTML = '<b>' + it.label + '</b> <span class="dim" style="font-size:0.82rem">'
                       + it.hint + '</span>';
        const line = document.createElement('div');
        line.className = 'row-flex';
        const inp = document.createElement('input');
        inp.type = 'text'; inp.value = it.text; inp.style.width = 'min(420px,72vw)';
        vInputs[it.key] = inp;
        const save = document.createElement('button');
        save.className = 'small'; save.textContent = '저장';
        save.onclick = () => saveWords(it.key);
        const def = document.createElement('button');
        def.className = 'small'; def.textContent = '기본값';
        def.onclick = () => cmd({action:'voice_words_reset', key:it.key});
        const flag = document.createElement('span');
        flag.className = 'dim'; flag.id = 'vFlag_' + it.key;
        line.append(inp, save, def, flag);
        row.append(head, line);
        box.append(row);
      }
    }
    // 저장·기본값 복원 뒤 서버가 내려주는 값으로 칸과 표시를 맞춘다.
    for (const it of items) {
      const inp = vInputs[it.key];
      if (inp && document.activeElement !== inp) inp.value = it.text;
      const flag = document.getElementById('vFlag_' + it.key);
      if (flag) flag.textContent = it.changed ? '· 기본값에서 고침' : '';
    }
  }
  function applyPulse() {
    cmd({action:'line_params',
         speed: +document.getElementById('pSpeed').value,
         pulse_on: +document.getElementById('pOn').value,
         pulse_period: +document.getElementById('pPeriod').value,
         align_dither: +document.getElementById('pDither').value,
         smooth_speed: +document.getElementById('pSmooth').value,
         travel_kick: +document.getElementById('pKick').value,
         travel_wiggle: +document.getElementById('pWiggle').value,
         wiggle_yaw: +document.getElementById('pWigYaw').value,
         corr_min: +document.getElementById('pCorrMin').value,
         corr_max: +document.getElementById('pCorrMax').value,
         smooth_dy_gain: +document.getElementById('pSmoothGain').value});
  }
  function preset(sp, on, per) {
    setRange('pSpeed', sp); setRange('pOn', on); setRange('pPeriod', per); applyPulse();
  }
  // 정렬을 **끝낼** 목표 범위. 주행 설정과 저장처는 같지만(line_params →
  // ~/line_tuning.json) 만지는 이유가 달라 카드를 나눴다.
  function applyRange() {
    cmd({action:'line_params',
         align_tol_x: +document.getElementById('rTolX').value,
         align_dy_below: +document.getElementById('rDyBelow').value,
         align_dy_above: +document.getElementById('rDyAbove').value,
         align_yaw_tol: +document.getElementById('rYaw').value});
  }
  function rpreset(x, below, above, yaw) {
    setRange('rTolX', x); setRange('rDyBelow', below);
    setRange('rDyAbove', above); setRange('rYaw', yaw); applyRange();
  }
  function setRange(id, v) {
    document.getElementById(id).value = v;
    document.getElementById(id + 'Out').textContent = v;
  }
  function applyTune() {
    cmd({action:'base_tune', hz: +document.getElementById('tHz').value,
         max_pwm: +document.getElementById('tPwm').value,
         accel: +document.getElementById('tAcc').value,
         speed_scale: +document.getElementById('tScale').value});
  }
  function tpreset(hz, pwm) { setRange('tHz', hz); setRange('tPwm', pwm); applyTune(); }
  function spreset(pct) { setRange('tScale', pct); applyTune(); }

  function render() {
    if (!state) return;
    const arm = state.arm || {}, base = state.base || {}, ln = state.line || {};
    renderVoice((state.voice || {}).items);
    renderWake((state.voice || {}).wake);

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
      + '(' + (ln.dir_source || '?') + ')'
      + '  ·  흔들기 시작쪽 ' + (ln.wiggle_sign > 0 ? '+' : '−')
      + '(회전 ' + Math.round(ln.wiggle_yaw || 0) + ')'
      + '  ·  중간지점 ' + (ln.stop_each ? '모두 들름' : '건너뜀')
      + (ln.chain_target != null ? ' → 최종 ' + ln.chain_target + '번' : '');

    if (!pulseInit && ln.speed != null) {
      pulseInit = true;
      setRange('pSpeed', Math.round(ln.speed));
      setRange('pOn', ln.pulse_on); setRange('pPeriod', ln.pulse_period);
      if (ln.align_dither != null) setRange('pDither', Math.round(ln.align_dither));
      if (ln.smooth_speed != null) setRange('pSmooth', Math.round(ln.smooth_speed));
      if (ln.travel_kick != null) setRange('pKick', Math.round(ln.travel_kick));
      if (ln.travel_wiggle != null) setRange('pWiggle', Math.round(ln.travel_wiggle));
      if (ln.wiggle_yaw != null) setRange('pWigYaw', Math.round(ln.wiggle_yaw));
      if (ln.corr_min != null) setRange('pCorrMin', Math.round(ln.corr_min));
      if (ln.corr_max != null) setRange('pCorrMax', Math.round(ln.corr_max));
      if (ln.smooth_dy_gain != null) setRange('pSmoothGain', Math.round(ln.smooth_dy_gain));
    }
    document.getElementById('pNow').textContent = ln.speed == null ? '' :
      ('현재: 속도 ' + Math.round(ln.speed) + ' · '
       + (ln.pulsing ? '펄스 ' + ln.pulse_on + 's / ' + ln.pulse_period + 's' : '연속 주행')
       + '  ·  정렬 흔들기(진행축) ' + Math.round(ln.align_dither || 0)
       + '  ·  게걸음 ' + (ln.smooth
            ? '주행이득 ' + Math.round(ln.smooth_dy_gain || 0)
              + ' (상한 ' + Math.round(ln.corr_max || 0) + ') · 정렬 '
              + Math.round(ln.corr_min || 0) + '~' + Math.round(ln.corr_max || 0)
            : Math.round(ln.corr_min || 0) + '~' + Math.round(ln.corr_max || 0))
       + '  ·  지금 게걸음 ' + (ln.dy_axis === 'vy' ? ln.would_vy : ln.would_vx)
       + '  ·  주행 ' + (ln.smooth ? '저속연속 ' + Math.round(ln.smooth_speed || 0)
                                     + ' (출발 ' + Math.round(ln.travel_kick || 0)
                                     + ' · 좌우톡 ' + Math.round(ln.travel_wiggle || 0) + ')'
                                   : '펄스(톡톡)'));

    // 정렬 목표 범위 — 슬라이더는 처음 한 번만 서버 값으로 채운다(만지는 중에
    // 덮어쓰면 손에서 값이 튄다). 아래 현재값 표시는 매 틱 갱신한다.
    if (!rangeInit && ln.align_tol_x != null) {
      rangeInit = true;
      setRange('rTolX', Math.round(ln.align_tol_x));
      setRange('rDyBelow', Math.round(ln.align_dy_below));
      setRange('rDyAbove', Math.round(ln.align_dy_above));
      setRange('rYaw', ln.align_yaw_tol);
    }
    document.getElementById('rNow').textContent = ln.align_tol_x == null ? '' :
      ('현재: x ±' + Math.round(ln.align_tol_x) + 'px  ·  y 아래 '
       + Math.round(ln.align_dy_below) + ' / 위 ' + Math.round(ln.align_dy_above)
       + 'px  ·  회전 ' + (ln.yaw_gain ? '±' + ln.align_yaw_tol + '°' : '판정 안 함(회전보정 꺼짐)'));

    const t = base.tuning;
    if (t) {
      document.getElementById('tNow').textContent =
        '전체 속도 ' + (t.speed_scale == null ? '?' : t.speed_scale) + '% · '
        + '요청값 ' + t.hz + 'Hz · 듀티 ' + t.max_pwm + ' · 가속 ' + t.accel
        + (base.acks && base.acks.length ? '   ← 보드 확인: ' + base.acks.join(' | ') : '   ← 보드 확인 없음');
      if (!tuneInit) {
        tuneInit = true;
        setRange('tHz', t.hz); setRange('tPwm', t.max_pwm); setRange('tAcc', t.accel);
        if (t.speed_scale != null) setRange('tScale', t.speed_scale);
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

"""대시보드의 HTML 페이지 템플릿.

server.py에서 분리한 이유 — 조작 화면이 커지면서 한 파일 안에서 다루기
어려워졌고, **조작(운전)** 과 **설정(영점·튜닝)** 을 화면 단위로 갈라야 했다.

  /control   운전 화면 — 데모 중에 실제로 누르는 것만
  /settings  시스템 설정 — 영점/축·부호/모터 튜닝/팔 캘리브레이션처럼
             "한 번 잡아두고 잘 안 건드리는" 것

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
</nav>
"""


def _shell(title: str, body: str, script: str, here: str) -> str:
    nav = _NAV.replace("__C__", "here" if here == "control" else "") \
              .replace("__S__", "here" if here == "settings" else "")
    return ("<!doctype html>\n<html lang=\"ko\"><head><meta charset=\"utf-8\">\n"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
            f"<title>{title}</title>\n<style>{_CSS}</style></head>\n<body>\n"
            f"<h1>{title}</h1>\n{nav}\n{body}\n<div id=\"out\">명령 대기 중</div>\n"
            f"<script>{_COMMON_JS}{script}\nrefresh();</script>\n</body></html>")


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
  <div class="row-flex" style="margin-bottom:0.5rem">
    <button onclick="cmd({action:'line_jog',side:'left'})">◀ 톡</button>
    <button onclick="cmd({action:'line_jog',side:'right'})">톡 ▶</button>
    <span class="dim">한 번 누를 때마다 펄스 한 번 — 같은 걸음이 반복된다</span>
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
      } else p.push('⚠ 테이프 없음');
      p.push(ln.position_px != null
             ? '변위 ' + Math.round(ln.position_px) + 'px'
               + (ln.position_pct != null ? ' (' + ln.position_pct + '%)' : '')
             : '변위 미기준 — 끝단을 한 번 지나가세요');
      if (ln.color_name) p.push('🎨 ' + ln.color_name);
      if (ln.end_side) p.push('■ 끝(' + (ln.end_side === 'left' ? '좌' : '우') + ')');
      p.push(ln.station != null ? '📍 ' + ln.station_label
                                : '📍 지점 미확인 — 주황 끝지점을 한 번 지나가세요');
      const seen = (ln.markers || []).map(m => Math.round(m.hue));
      if (seen.length) p.push('보이는 마커 hue ' + seen.join(','));
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
  <label>펄스 ON <input type="range" id="pOn" min="0.03" max="1" step="0.01" value="0.12"
    oninput="pOnOut.textContent=this.value"> <b id="pOnOut">0.12</b> 초
    <span class="dim">한 번에 가는 거리</span></label>
  <label>펄스 주기 <input type="range" id="pPeriod" min="0.03" max="1.5" step="0.01" value="0.32"
    oninput="pPeriodOut.textContent=this.value"> <b id="pPeriodOut">0.32</b> 초
    <span class="dim">ON과 같게 두면 <b>연속 주행</b>이 된다</span></label>
  <div class="row-flex">
    <button onclick="applyPulse()">적용</button>
    <button class="small" onclick="preset(150,0.12,0.32)">깔짝깔짝(기본)</button>
    <button class="small" onclick="preset(120,0.08,0.40)">더 잘게</button>
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
         pulse_period: +document.getElementById('pPeriod').value});
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
    } else el.textContent = '⚠ 테이프 없음 — 카메라가 띠를 보게 한 뒤 누르세요';

    document.getElementById('axisState').textContent =
      '보정축 ' + ln.dy_axis + '(부호 ' + ln.dy_sign + ')  ·  진행축 ' + ln.travel_axis
      + '(부호 ' + ln.travel_sign + ')  ·  회전 ' + (ln.yaw_gain ? '켜짐(부호 ' + ln.yaw_sign + ')' : '꺼짐')
      + '  ·  지금 보정 ' + (ln.dy_axis === 'vy' ? ln.would_vy : ln.would_vx)
      + ' w=' + ln.would_w;

    if (!pulseInit && ln.speed != null) {
      pulseInit = true;
      setRange('pSpeed', Math.round(ln.speed));
      setRange('pOn', ln.pulse_on); setRange('pPeriod', ln.pulse_period);
    }
    document.getElementById('pNow').textContent = ln.speed == null ? '' :
      ('현재: 속도 ' + Math.round(ln.speed) + ' · '
       + (ln.pulsing ? '펄스 ' + ln.pulse_on + 's / ' + ln.pulse_period + 's' : '연속 주행'));

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

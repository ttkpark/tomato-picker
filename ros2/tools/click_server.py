#!/usr/bin/env python3
"""**젯슨에서 띄우는 조작 페이지 + API** — 사람도 로봇도 같은 문으로 시킨다.

    ~/lerobot/.venv/bin/python ros2/tools/click_server.py --port 8090
    (systemd: click-server.service — 부팅 자동실행)

브라우저에서 `http://<젯슨IP>:8090/` — 같은 망이면 폰에서도 된다.

────────────────────────────────────────────────────────────────────────
왜 이렇게 만드나

(가) **표적은 사람만 안다.** 화면에 빨간 것이 여럿이다 — 매달린 열매 둘,
     책상에 떨어진 열매, 선반의 주황 안전조끼. 2026-09-03에 서보가 화소오차
     8~9까지 말끔히 수렴했는데 겨눈 것이 조끼였고 책상 열매였다. 알고리즘이
     아니라 표적 선택이 틀렸다. 클릭 한 번이 그걸 끊는다.

(나) **에이전트가 하는 일을 사람이 그대로 재현할 수 있어야 한다.** 그래서
     모든 조작을 이 API 하나로 내린다 — 화면의 버튼과 `curl`과 에이전트가
     **같은 경로**를 쓴다. 터미널에서만 되는 조작을 남기지 않는다.

⚠ **한 번에 한 가지만 돈다.** 팔 포트(`/dev/ttyACM0`)는 한 프로세스만 열 수
  있어서, 일이 도는 중에는 새 일을 거절한다(409). `/api/stop`으로 끊는다.
⚠ 화면은 `depth-cam` 이 `/dev/shm` 에 쓰는 것을 그대로 낸다 — 죽어 있으면
  낡은 그림이 나오므로 프레임 나이를 같이 띄운다.
⚠ 바깥(PC 웹캠) 화면은 여기 없다 — 그건 PC에 붙어 있다.

API 요약 (전부 JSON)
    GET  /state                      상태 한 줌
    GET  /log?since=N                로그 이어받기
    GET  /frame.jpg                  손목 화면
    POST /click   {u,v,mode}         mode=target(줄기) | grip(십자)
    POST /clear                      표적 지움
    POST /run     {job,args}         job=stage|jog|grasp|grip|pose|park
    POST /stop                       도는 일을 끊는다
"""

from __future__ import annotations

import argparse
import json
import os
import posixpath
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PY = os.environ.get("TOMATO_PY", "/home/server/lerobot/.venv/bin/python")
COLOR = "/dev/shm/d405_color.jpg"
DEPTH = "/dev/shm/d405_depth.npy"
META = "/dev/shm/d405_meta.json"
TARGET = os.path.expanduser("~/click_target.json")
GRIPF = os.path.expanduser("~/grip_uv.json")
GRIP_UV = (471, 395)              # 집게를 여닫아 실측한 기본값
PARK = "60,65,0,-100,6"           # 열매가 보이던 대기 자세(도)


def grip_uv():
    try:
        g = json.load(open(GRIPF))
        return int(g["u"]), int(g["v"])
    except Exception:                                      # noqa: BLE001
        return GRIP_UV


def depth_at(u: int, v: int, half: int = 9) -> float:
    try:
        dep = np.load(DEPTH).astype(float)
        sc = float(json.load(open(META)).get("depth_scale_mm", 1.0))
    except Exception:                                      # noqa: BLE001
        return -1.0
    h, w = dep.shape[:2]
    y0, y1 = max(0, v - half), min(h, v + half + 1)
    x0, x1 = max(0, u - half), min(w, u + half + 1)
    d = dep[y0:y1, x0:x1].reshape(-1) * sc
    d = d[d > 0]
    return float(np.percentile(d, 25)) if d.size >= 12 else -1.0


# ── 일 돌리기 ──────────────────────────────────────────────────────────
class Job:
    """한 번에 하나만 도는 바깥 명령. 로그는 줄 단위로 쌓아 둔다."""

    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.name = ""
        self.lines = []
        self.started = 0.0
        self.rc = None

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, name, argv):
        with self.lock:
            if self.running():
                return False, "%s 가 도는 중이다" % self.name
            self.name, self.rc, self.started = name, None, time.time()
            self.lines.append("$ " + " ".join(argv[1:]))
            self.proc = subprocess.Popen(
                argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, cwd=os.path.join(HERE, "..", ".."),
                preexec_fn=os.setsid)
            threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()
            return True, ""

    def _pump(self, proc):
        for ln in proc.stdout:
            ln = ln.rstrip("\n")
            if ln:
                self.lines.append(ln)
                if len(self.lines) > 4000:
                    del self.lines[:1000]
        proc.wait()
        self.rc = proc.returncode
        self.lines.append("— 끝 (%s, %.0f초)" % (self.rc, time.time() - self.started))

    def stop(self):
        if not self.running():
            return False
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            time.sleep(1.2)
            if self.running():
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except Exception:                                  # noqa: BLE001
            return False
        self.lines.append("— 사람이 끊었다")
        return True


JOB = Job()


def num(args, key, default, lo, hi):
    try:
        v = float(args.get(key, default))
    except (TypeError, ValueError):
        v = float(default)
    return max(lo, min(hi, v))


def build(job, args):
    """job 이름 + 인자 → 실제로 돌릴 argv. **여기서만** 명령이 정해진다."""
    T = lambda n: os.path.join(HERE, n)                     # noqa: E731
    if job == "park":
        return [PY, T("arm_stage.py"), "--target=" + PARK]
    if job == "stage":
        t = str(args.get("target", PARK))
        parts = t.replace(" ", "").split(",")
        if len(parts) != 5:
            raise ValueError("target 은 관절 5개(도)여야 한다")
        for p in parts:
            float(p)
        return [PY, T("arm_stage.py"), "--target=" + ",".join(parts)]
    if job == "pose":
        return [PY, T("arm_stage.py"), "--dry", "--target=0,0,0,0,0"]
    if job == "grip":
        return [PY, T("grip_set.py"), "%.0f" % num(args, "value", 78, 0, 100)]
    if job == "jog":
        a = [PY, T("tool_jog.py")]
        for k, lo, hi in (("dx", -200, 200), ("dy", -200, 200), ("dz", -200, 200),
                          ("along", -250, 250), ("horiz", -250, 250),
                          ("pitch", -60, 60)):
            v = num(args, k, 0, lo, hi)
            if abs(v) > 1e-6:
                a += ["--" + k, "%.1f" % v]
        if len(a) == 2:
            raise ValueError("옮길 양이 없다")
        a += ["--piece", "%.0f" % num(args, "piece", 25, 5, 60)]
        if args.get("free_pitch"):
            a += ["--free-pitch"]
        return a
    if job == "grasp":
        a = [PY, T("stem_grasp.py"), "--aim", str(args.get("aim", "click"))]
        a += ["--steps", "%d" % int(num(args, "steps", 16, 1, 60)),
              "--adv", "%.1f" % num(args, "adv", 8, 1, 30),
              "--max-turn", "%.1f" % num(args, "max_turn", 2, 0.3, 8),
              "--gain", "%.2f" % num(args, "gain", 0.35, 0.05, 1.0),
              "--tol", "%.0f" % num(args, "tol", 28, 5, 120),
              "--rejacobian", "%d" % int(num(args, "rejacobian", 3, 1, 20)),
              "--stop-z", "%.0f" % num(args, "stop_z", 88, 0, 400)]
        if args.get("no_close"):
            a += ["--no-close"]
        if args.get("dry"):
            a += ["--dry"]
        return a
    raise ValueError("모르는 일: %s" % job)


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>토마토 조작대</title>
<style>
:root{color-scheme:light dark;--ink:#14170f;--bg:#f6f7f2;--panel:#fff;--line:#d5d9cf;
 --dim:#5d655a;--hit:#e4572e;--grip:#2f6fb0;--ok:#3f7d3a}
@media (prefers-color-scheme:dark){:root{--ink:#eceee6;--bg:#111409;--panel:#1b1f16;
 --line:#3a4032;--dim:#98a091;--hit:#ff7a52;--grip:#6ba6de;--ok:#7fb069}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
 font:15px/1.5 system-ui,-apple-system,"Noto Sans KR",sans-serif}
.wrap{max-width:1180px;margin:0 auto;padding:14px 12px 60px;display:grid;
 grid-template-columns:minmax(0,1fr) 340px;gap:16px;align-items:start}
@media (max-width:940px){.wrap{grid-template-columns:minmax(0,1fr)}}
header{grid-column:1/-1;display:flex;flex-wrap:wrap;gap:10px;align-items:baseline;
 border-bottom:1px solid var(--line);padding-bottom:8px}
h1{margin:0;font-size:19px;letter-spacing:-.01em}
.sub{color:var(--dim);font-size:13px}
.stage{position:relative;background:#000;border:1px solid var(--line);border-radius:10px;
 overflow:hidden;line-height:0;touch-action:manipulation}
.stage img{width:100%;height:auto;display:block}
.stage svg{position:absolute;inset:0;width:100%;height:100%}
.col{display:flex;flex-direction:column;gap:12px;min-width:0}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:11px 12px}
.card h2{margin:0 0 8px;font-size:13px;letter-spacing:.04em;text-transform:uppercase;color:var(--dim)}
.row{display:flex;flex-wrap:wrap;gap:7px;align-items:center}
button{font:inherit;padding:7px 11px;border-radius:8px;border:1px solid var(--line);
 background:var(--panel);color:var(--ink);cursor:pointer}
button:hover{border-color:var(--dim)}
button.go{border-color:var(--hit);color:var(--hit);font-weight:600}
button.stop{border-color:#b4231b;color:#b4231b;font-weight:600}
input{font:inherit;width:66px;padding:6px 7px;border-radius:7px;border:1px solid var(--line);
 background:var(--bg);color:var(--ink)}
label{font-size:12.5px;color:var(--dim);display:inline-flex;gap:5px;align-items:center}
pre{margin:0;max-height:270px;overflow:auto;font:12px/1.45 ui-monospace,Menlo,monospace;
 background:var(--bg);border:1px solid var(--line);border-radius:8px;padding:8px;white-space:pre-wrap}
.k{color:var(--dim);font-size:12px}
.v{font-family:ui-monospace,Menlo,monospace;font-size:13px}
.busy{color:var(--hit);font-weight:600}
.idle{color:var(--ok)}
code{font:12px ui-monospace,Menlo,monospace;color:var(--dim)}
</style></head><body><div class="wrap">
<header><h1>토마토 조작대</h1>
<span class="sub">화면을 눌러 <b>줄기</b>를 정하고, 오른쪽에서 시킨다. 사람과 에이전트가 같은 API를 쓴다.</span>
</header>

<div class="col">
  <div class="row">
    <button id="m_t" class="go">① 줄기 찍기</button>
    <button id="m_g">② 십자(집게 자리) 옮기기</button>
    <button id="clear">표적 지움</button>
    <button id="pause">화면 멈춤/재생</button>
    <span class="k" id="age"></span>
  </div>
  <div class="stage" id="stage">
    <img id="im" alt="손목 카메라">
    <svg id="ov" viewBox="0 0 848 480" preserveAspectRatio="none"></svg>
  </div>
  <div class="card"><h2>기록</h2><pre id="log"></pre></div>
</div>

<div class="col">
  <div class="card">
    <h2>지금</h2>
    <div class="v" id="cur">표적 없음</div>
    <div class="v" id="jobline">—</div>
  </div>

  <div class="card"><h2>잡기</h2>
    <div class="row">
      <label>걸음<input id="g_steps" value="16"></label>
      <label>전진<input id="g_adv" value="8"></label>
      <label>최대각<input id="g_turn" value="2"></label>
    </div>
    <div class="row" style="margin-top:6px">
      <label>게인<input id="g_gain" value="0.35"></label>
      <label>문턱px<input id="g_tol" value="28"></label>
      <label>무는거리<input id="g_stop" value="88"></label>
    </div>
    <div class="row" style="margin-top:8px">
      <button class="go" onclick="run('grasp',grasp(false))">잡으러 간다</button>
      <button onclick="run('grasp',grasp(true))">안 닫고 가기</button>
      <button onclick="run('grasp',Object.assign(grasp(true),{dry:1}))">겨냥만 본다</button>
    </div>
  </div>

  <div class="card"><h2>조그 (손끝을 옮긴다, mm)</h2>
    <div class="row">
      <label>한 번에<input id="j_mm" value="20"></label>
      <label><input type="checkbox" id="j_free"> 자세 안 지킴</label>
    </div>
    <div class="row" style="margin-top:8px">
      <button onclick="jog('dz',1)">▲ 위</button>
      <button onclick="jog('dz',-1)">▼ 아래</button>
      <button onclick="jog('horiz',1)">▶ 수평 앞</button>
      <button onclick="jog('horiz',-1)">◀ 수평 뒤</button>
    </div>
    <div class="row" style="margin-top:6px">
      <button onclick="jog('along',1)">접근축 앞</button>
      <button onclick="jog('along',-1)">접근축 뒤</button>
      <button onclick="jog('pitch',1)">피치 +</button>
      <button onclick="jog('pitch',-1)">피치 −</button>
    </div>
  </div>

  <div class="card"><h2>집게 · 자세</h2>
    <div class="row">
      <button onclick="run('grip',{value:78})">집게 연다</button>
      <button onclick="run('grip',{value:4})">집게 닫는다</button>
      <button onclick="run('pose',{})">자세 읽기</button>
      <button onclick="run('park',{})">대기 자세</button>
    </div>
    <div class="row" style="margin-top:8px">
      <input id="s_t" style="width:190px" placeholder="60,65,0,-100,6">
      <button onclick="run('stage',{target:document.getElementById('s_t').value})">그 자세로</button>
    </div>
  </div>

  <div class="card"><h2>멈춤</h2>
    <div class="row"><button class="stop" onclick="stop()">지금 하는 일 끊기</button></div>
    <div class="k" style="margin-top:6px">터미널에서도 같은 일:
      <code>curl -s -XPOST 1.2.3.4:8090/run -d '{"job":"grasp","args":{}}'</code></div>
  </div>
</div>
</div><script>
var W=848,H=480,live=true,pt=null,gp={u:471,v:395},mode='target',since=0;
var im=document.getElementById('im'),ov=document.getElementById('ov'),stage=document.getElementById('stage');
function draw(){
  var s='<line x1="'+(gp.u-16)+'" y1="'+gp.v+'" x2="'+(gp.u+16)+'" y2="'+gp.v+'" stroke="#2f6fb0" stroke-width="3"/>'
       +'<line x1="'+gp.u+'" y1="'+(gp.v-16)+'" x2="'+gp.u+'" y2="'+(gp.v+16)+'" stroke="#2f6fb0" stroke-width="3"/>';
  if(pt){s+='<circle cx="'+pt.u+'" cy="'+pt.v+'" r="13" fill="none" stroke="#e4572e" stroke-width="4"/>'
        +'<circle cx="'+pt.u+'" cy="'+pt.v+'" r="2.5" fill="#e4572e"/>';}
  ov.innerHTML=s;
}
function tick(){ if(live) im.src='/frame.jpg?t='+Date.now(); }
function post(p,b){return fetch(p,{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify(b||{})}).then(function(r){return r.json();});}
stage.addEventListener('click',function(e){
  var r=im.getBoundingClientRect();
  var u=Math.round((e.clientX-r.left)/r.width*W), v=Math.round((e.clientY-r.top)/r.height*H);
  if(u<0||v<0||u>=W||v>=H) return;
  post('/click',{u:u,v:v,mode:mode}).then(function(j){
    if(j.mode==='grip'){gp={u:j.u,v:j.v}; setMode('target');}
    else{pt={u:j.u,v:j.v};
      document.getElementById('cur').textContent='표적 ('+j.u+', '+j.v+')  깊이 '+(j.z>0?j.z.toFixed(0)+'mm':'없음');}
    draw();});
});
document.getElementById('clear').onclick=function(){post('/clear').then(function(){pt=null;draw();
  document.getElementById('cur').textContent='표적 없음';});};
document.getElementById('pause').onclick=function(){live=!live;};
function setMode(m){mode=m;
  document.getElementById('m_t').className=(m==='target')?'go':'';
  document.getElementById('m_g').className=(m==='grip')?'go':'';}
document.getElementById('m_t').onclick=function(){setMode('target');};
document.getElementById('m_g').onclick=function(){setMode('grip');};
function val(id,d){var x=parseFloat(document.getElementById(id).value);return isNaN(x)?d:x;}
function grasp(nc){return {steps:val('g_steps',16),adv:val('g_adv',8),max_turn:val('g_turn',2),
  gain:val('g_gain',0.35),tol:val('g_tol',28),stop_z:val('g_stop',88),no_close:nc?1:0};}
function jog(k,s){var a={};a[k]=s*val('j_mm',20);
  if(document.getElementById('j_free').checked)a.free_pitch=1; run('jog',a);}
function run(job,args){post('/run',{job:job,args:args}).then(function(j){
  if(!j.ok) alert(j.why||'거절됨');});}
function stop(){post('/stop');}
function state(){
  fetch('/state').then(function(r){return r.json();}).then(function(j){
    if(j.gu!=null) gp={u:j.gu,v:j.gv};
    if(j.u!=null){pt={u:j.u,v:j.v};
      document.getElementById('cur').textContent='표적 ('+j.u+', '+j.v+')  깊이 '+(j.z>0?j.z.toFixed(0)+'mm':'없음');}
    document.getElementById('age').textContent='프레임 '+j.age.toFixed(1)+'초 전'+(j.age>5?' ⚠ depth-cam 확인':'');
    var e=document.getElementById('jobline');
    e.textContent=j.running?('도는 중: '+j.job):('놀고 있음'+(j.job?(' (마지막 '+j.job+', rc='+j.rc+')'):''));
    e.className='v '+(j.running?'busy':'idle');
    draw();}).catch(function(){});
}
function log(){
  fetch('/log?since='+since).then(function(r){return r.json();}).then(function(j){
    if(j.lines.length){var e=document.getElementById('log');
      e.textContent+=(e.textContent?'\n':'')+j.lines.join('\n');
      e.scrollTop=e.scrollHeight;}
    since=j.next;}).catch(function(){});
}
setInterval(tick,400); setInterval(state,1200); setInterval(log,900); tick(); state(); log();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *a):
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):                       # noqa: N802
        p = posixpath.normpath(urlparse(self.path).path)
        if p in ("/", "/index.html"):
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif p == "/frame.jpg":
            try:
                with open(COLOR, "rb") as fh:
                    self._send(200, fh.read(), "image/jpeg")
            except Exception:                              # noqa: BLE001
                self._send(503, b"no frame", "text/plain")
        elif p == "/state":
            gu, gv = grip_uv()
            out = {"age": 99.0, "u": None, "v": None, "z": -1.0, "gu": gu, "gv": gv,
                   "running": JOB.running(), "job": JOB.name, "rc": JOB.rc,
                   "log_len": len(JOB.lines)}
            try:
                out["age"] = time.time() - os.path.getmtime(COLOR)
            except OSError:
                pass
            if os.path.exists(TARGET):
                try:
                    t = json.load(open(TARGET))
                    out["u"], out["v"] = int(t["u"]), int(t["v"])
                    out["z"] = depth_at(out["u"], out["v"])
                except Exception:                          # noqa: BLE001
                    pass
            self._send(200, json.dumps(out))
        elif p == "/log":
            q = parse_qs(urlparse(self.path).query)
            try:
                since = max(0, int(q.get("since", ["0"])[0]))
            except ValueError:
                since = 0
            lines = JOB.lines[since:since + 400]
            self._send(200, json.dumps({"lines": lines, "next": since + len(lines)}))
        else:
            self._send(404, b"nope", "text/plain")

    def _body(self):
        n = int(self.headers.get("Content-Length", "0") or 0)
        if n <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8"))
        except Exception:                                  # noqa: BLE001
            return {}

    def do_POST(self):                      # noqa: N802
        p = posixpath.normpath(urlparse(self.path).path)
        if p == "/clear":
            try:
                os.remove(TARGET)
            except OSError:
                pass
            self._send(200, json.dumps({"ok": True}))
            return
        if p == "/stop":
            self._send(200, json.dumps({"ok": JOB.stop()}))
            return
        if p == "/click":
            b = self._body()
            try:
                u, v = int(b["u"]), int(b["v"])
            except Exception:                              # noqa: BLE001
                self._send(400, json.dumps({"ok": False}))
                return
            u, v = max(0, min(847, u)), max(0, min(479, v))
            if str(b.get("mode", "target")) == "grip":
                json.dump({"u": u, "v": v, "when": time.strftime("%Y-%m-%d %H:%M:%S")},
                          open(GRIPF, "w"), indent=1)
                self._send(200, json.dumps({"ok": True, "u": u, "v": v, "mode": "grip"}))
                return
            z = depth_at(u, v)
            json.dump({"u": u, "v": v, "z": z, "when": time.strftime("%Y-%m-%d %H:%M:%S")},
                      open(TARGET, "w"), indent=1)
            self._send(200, json.dumps({"ok": True, "u": u, "v": v, "z": z,
                                        "mode": "target"}))
            return
        if p in ("/run", "/api/run"):
            b = self._body()
            job = str(b.get("job", ""))
            args = b.get("args") or {}
            try:
                argv = build(job, args)
            except Exception as e:                         # noqa: BLE001
                self._send(400, json.dumps({"ok": False, "why": str(e)}))
                return
            ok, why = JOB.start(job, argv)
            self._send(200 if ok else 409, json.dumps({"ok": ok, "why": why,
                                                       "argv": argv[1:]}))
            return
        self._send(404, b"nope", "text/plain")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--host", default="0.0.0.0")
    args = ap.parse_args()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print("http://<젯슨IP>:%d/   표적=%s  십자=%s" % (args.port, TARGET, GRIPF), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

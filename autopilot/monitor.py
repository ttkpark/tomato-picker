#!/usr/bin/env python3
"""자동운전 모니터 — 지금 무엇이 돌고 있는지 브라우저로 본다.

  python autopilot/monitor.py            # http://127.0.0.1:8099/
  python autopilot/monitor.py --port 9000 --host 0.0.0.0

왜 웹인가 — 터미널 하나를 계속 붙잡고 있지 않아도 되고, 폰에서도 열린다.
읽기 전용이 아니다: 아래 입력칸이 `autopilot/say.py`와 같은 창구다(INBOX).

무엇이 보이나
  · 지금 도는 역할·모델·경과, **그 사이클이 방금 부른 도구들**(live.json)
  · 주간 한도 소진률과 리셋까지 남은 시간 — 지금 이 장치의 가장 큰 제약
  · 작업판(열린 일), 오늘 일지, 안 읽힌 내 말, 최근 사이클
"""
import argparse
import json
import os
import re
import sys
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STATE_DIR = os.path.join(HERE, "state")
JOURNAL_DIR = os.path.join(HERE, "journal")
LOG_DIR = os.path.join(HERE, "logs")

ROLE_COLOR = {"planner": "#7aa2f7", "builder": "#9ece6a", "auditor": "#e0af68",
              "tester": "#bb9af7", "metrologist": "#f7768e"}


def jread(path, default=None):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return default if default is not None else {}


def tail(path, n):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()[-n:]
    except OSError:
        return ""


def today_journal():
    p = os.path.join(JOURNAL_DIR, datetime.now().strftime("%Y-%m-%d") + ".md")
    if not os.path.exists(p):
        files = sorted(f for f in os.listdir(JOURNAL_DIR)) if os.path.isdir(JOURNAL_DIR) else []
        if not files:
            return "(아직 없음)"
        p = os.path.join(JOURNAL_DIR, files[-1])
    return tail(p, 9000)


def recent_cycles(n=12):
    """runner.log에서 끝난 사이클 줄만 골라 최근 것부터."""
    out = []
    for line in reversed(tail(os.path.join(LOG_DIR, "runner.log"), 40000).splitlines()):
        # 옛 형식 `role(model)`과 새 형식 `role(engine:model)`을 둘 다 읽는다.
        m = re.search(r"사이클 (\d+) · (\w+)\((\S+?)\) (완료|실패) · (\d+)초 · \$([\d.]+) · (\d+)턴", line)
        if m:
            tag = m.group(3)
            eng, _, mod = tag.partition(":")
            if not mod:
                eng, mod = "claude", tag
            out.append({"cycle": int(m.group(1)), "role": m.group(2),
                        "engine": eng, "model": mod,
                        "ok": m.group(4) == "완료", "sec": int(m.group(5)),
                        "cost": float(m.group(6)), "turns": int(m.group(7)),
                        "at": line[1:15]})
            if len(out) >= n:
                break
    return out


def pending_say():
    txt = tail(os.path.join(HERE, "INBOX.md"), 20000)
    return [L.strip()[5:].strip() for L in txt.splitlines() if L.strip().startswith("- [ ]")]


def board_open():
    """board.json을 직접 읽는다 — CLI를 매 초 띄우면 그게 더 무겁다."""
    b = jread(os.path.join(STATE_DIR, "board.json"), {"tasks": []})
    ts = [t for t in b.get("tasks", []) if t.get("status") != "done"]
    ts.sort(key=lambda t: (t.get("status") != "doing", t.get("priority", 5), t.get("id", "")))
    done = sum(1 for t in b.get("tasks", []) if t.get("status") == "done")
    return ts[:14], done, len(b.get("tasks", []))


def snapshot():
    st = jread(os.path.join(STATE_DIR, "state.json"))
    hb = jread(os.path.join(STATE_DIR, "heartbeat.json"))
    live = jread(os.path.join(STATE_DIR, "live.json"))
    tasks, done, total = board_open()
    rates = st.get("rates") or {}
    if not rates and (st.get("rate") or live.get("rate")):
        r0 = st.get("rate") or live.get("rate")
        rates = {r0.get("rateLimitType", "unknown"): r0}
    rate = max(rates.values(), key=lambda r: (r or {}).get("utilization") or 0) if rates else {}
    attrib = st.get("attrib") or {}
    started = st.get("started")
    day = None
    if started:
        try:
            day = (datetime.now() - datetime.fromisoformat(started)).days + 1
        except ValueError:
            pass
    paused = os.path.exists(os.path.join(HERE, "PAUSE"))
    stopped = os.path.exists(os.path.join(HERE, "STOP"))
    hb_age = None
    if hb.get("time"):
        try:
            hb_age = (datetime.now() - datetime.fromisoformat(hb["time"])).total_seconds()
        except ValueError:
            pass
    return {
        "now": datetime.now().strftime("%m-%d %H:%M:%S"),
        "day": day, "cycle": st.get("cycle", 0), "cost_today": st.get("cost_today", 0.0),
        "fails": st.get("fails", 0), "jetson": st.get("jetson_ip"),
        "hb": hb, "hb_age": hb_age, "paused": paused, "stopped": stopped,
        "live": live, "rate": rate, "rates": rates, "attrib": attrib,
        "du": st.get("du_ema"), "engines": {},
        "tasks": tasks, "done": done, "total": total,
        "say": pending_say(), "journal": today_journal(),
        "cycles": recent_cycles(),
    }


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>자동운전 모니터</title><style>
:root{--bg:#1a1b26;--fg:#c0caf5;--dim:#565f89;--card:#1f2335;--line:#2f3549;--ok:#9ece6a;--bad:#f7768e;--warn:#e0af68}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.6 "Malgun Gothic",system-ui,sans-serif}
header{position:sticky;top:0;background:var(--bg);border-bottom:1px solid var(--line);padding:10px 16px;z-index:5;
 display:flex;flex-wrap:wrap;gap:8px 18px;align-items:baseline}
h1{font-size:15px;margin:0;font-weight:700}
.wrap{padding:16px;display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));max-width:1500px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px;min-width:0}
.card h2{font-size:12px;margin:0 0 8px;color:var(--dim);letter-spacing:.08em;text-transform:uppercase}
.big{font-size:22px;font-weight:700}
.dim{color:var(--dim)}.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}
.bar{height:8px;background:#0f1117;border-radius:4px;overflow:hidden;margin:6px 0}
.bar>i{display:block;height:100%}
table{width:100%;border-collapse:collapse}td{padding:2px 4px;vertical-align:top}
.mono{font-family:Consolas,monospace;font-size:12px}
pre{white-space:pre-wrap;word-break:break-word;margin:0;font-family:Consolas,monospace;font-size:12px;
 max-height:420px;overflow:auto}
.tool{border-left:2px solid var(--line);padding-left:8px;margin:3px 0}
.pill{border-radius:99px;padding:1px 9px;font-size:11px;font-weight:700;color:#1a1b26}
input,button{font:inherit;background:#0f1117;color:var(--fg);border:1px solid var(--line);
 border-radius:6px;padding:7px 10px}
button{cursor:pointer;background:#2a2f45}
.row{display:flex;gap:8px}.row input{flex:1;min-width:0}
.t{display:flex;gap:8px;padding:3px 0;border-bottom:1px solid #22263a}
.t b{font-weight:700;min-width:38px}
</style></head><body>
<header>
  <h1>자동운전 모니터</h1>
  <span id="hstate"></span><span id="hcycle" class="dim"></span>
  <span id="hrate"></span><span id="hjet" class="dim"></span>
  <span id="hnow" class="dim" style="margin-left:auto"></span>
</header>
<div class="wrap">
  <div class="card" style="grid-column:1/-1">
    <h2>지금 도는 것</h2><div id="live"></div>
  </div>
  <div class="card"><h2>사용량 한도 — 창마다, 누가 먹었나</h2><div id="rate"></div></div>
  <div class="card"><h2>내가 하는 말 (INBOX)</h2>
    <div class="row"><input id="msg" placeholder="예: 그 방향 아니다. 평행이동부터"><button onclick="say()">보내기</button></div>
    <div id="say" style="margin-top:8px"></div>
  </div>
  <div class="card"><h2>작업판</h2><div id="board"></div></div>
  <div class="card"><h2>최근 사이클</h2><div id="cycles"></div></div>
  <div class="card" style="grid-column:1/-1"><h2>오늘 일지</h2><pre id="journal"></pre></div>
</div>
<script>
const RC={planner:"#7aa2f7",builder:"#9ece6a",auditor:"#e0af68",tester:"#bb9af7",metrologist:"#f7768e"};
const esc=s=>(s==null?"":String(s)).replace(/[&<>]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;"}[c]));
const mmss=s=>s==null?"-":Math.floor(s/60)+"분 "+Math.floor(s%60)+"초";
function pill(r){return `<span class="pill" style="background:${RC[r]||"#565f89"}">${esc(r)}</span>`}
async function tick(){
 let d; try{ d=await (await fetch("/api",{cache:"no-store"})).json() }catch(e){ return }
 document.getElementById("hnow").textContent=d.now;
 const dead=d.hb_age!=null&&d.hb_age>2700;
 document.getElementById("hstate").innerHTML=
   d.stopped?'<b class="bad">STOP</b>':d.paused?'<b class="warn">PAUSE</b>':
   dead?'<b class="bad">응답 없음</b>':'<b class="ok">돌고 있다</b>';
 document.getElementById("hcycle").textContent=
   `${d.day?d.day+"일째 · ":""}사이클 ${d.cycle} · 오늘 $${(d.cost_today||0).toFixed(2)}`
   +(d.fails?` · 연속실패 ${d.fails}`:"");
 document.getElementById("hjet").textContent="젯슨 "+(d.jetson||"없음");

 // 지금 도는 것
 const L=d.live||{};
 let h="";
 const eng=L.engine&&L.engine!=="claude"?` <span class="pill" style="background:#565f89">${esc(L.engine)}</span>`:"";
 if(L.state==="running"){
   h+=`<div class="big">${pill(L.role)}${eng} <span class="dim">${esc(L.model)}</span> `
     +`사이클 ${L.cycle} · ${mmss(L.elapsed)} · ${L.turns||0}턴</div>`;
   h+=(L.tools||[]).slice().reverse().map(t=>
      `<div class="tool"><span class="dim mono">${esc(t.t)}</span> <b>${esc(t.name)}</b> `
      +`<span class="mono">${esc(t.brief)}</span></div>`).join("");
   if(L.last_text) h+=`<pre style="margin-top:8px;max-height:160px">${esc(L.last_text)}</pre>`;
 }else{
   h=`<div class="dim">쉬는 중 — 마지막: ${L.role?pill(L.role)+eng+" 사이클 "+L.cycle+" "+esc(L.state||""):"없음"}</div>`;
 }
 document.getElementById("live").innerHTML=h;

 // 한도 — 창(5시간/7일)마다 한 덩이. 누가 먹었는지까지.
 const NAMES={five_hour:"5시간 창",seven_day:"7일 창",unknown:"창 미상"};
 const rs=d.rates||{}; const keys=Object.keys(rs);
 if(keys.length){
   let out="", worst=0;
   for(const k of keys){
     const r=rs[k]||{}, u=r.utilization; if(u==null) continue;
     worst=Math.max(worst,u);
     const col=u>0.95?"var(--bad)":u>0.8?"var(--warn)":"var(--ok)";
     const hrs=r.resetsAt?((r.resetsAt*1000-Date.now())/3.6e6):null;
     const a=(d.attrib||{})[k]||{}; const roles=a.roles||{};
     const mine=Object.values(roles).reduce((x,y)=>x+y,0), other=a.other||0;
     const seen=mine+other, unseen=Math.max(0,(u-(a.start_u!=null?a.start_u:u))-seen);
     out+=`<div style="margin-bottom:12px">`
       +`<div><b>${NAMES[k]||k}</b> <span class="big" style="color:${col}">${(u*100).toFixed(1)}%</span>`
       +(hrs!=null?` <span class="dim">리셋 ${hrs.toFixed(1)}시간 뒤 (${new Date(r.resetsAt*1000).toLocaleString("ko-KR")})</span>`:"")+`</div>`
       +`<div class="bar"><i style="width:${(u*100).toFixed(1)}%;background:${col}"></i></div>`;
     if(seen>0){
       out+=`<table><tr><td class="dim">자동운전이 먹은 몫</td><td><b>${(mine*100).toFixed(2)}%p</b>`
         +` <span class="dim">(관측 구간에서)</span></td></tr>`
         +`<tr><td class="dim">그 밖(내 대화 등)</td><td>${(other*100).toFixed(2)}%p</td></tr>`;
       const top=Object.entries(roles).sort((x,y)=>y[1]-x[1]).slice(0,6);
       for(const [name,v] of top){
         if(v<=0) continue;
         const [role,eng]=name.split(":");
         out+=`<tr><td class="dim" style="padding-left:14px">${pill(role)} <span class="mono">${esc(eng)}</span></td>`
           +`<td class="mono">${(v*100).toFixed(2)}%p</td></tr>`;
       }
       if(unseen>0.0001) out+=`<tr><td class="dim">못 본 구간</td><td class="mono">${(unseen*100).toFixed(2)}%p</td></tr>`;
       out+=`</table>`;
     } else {
       out+=`<div class="dim">아직 이 창에서 귀속할 만큼 신호를 못 봤다.</div>`;
     }
     out+=`</div>`;
   }
   out+=`<div class="dim">사이클당 소모 ${d.du?(d.du*100).toFixed(2)+"%":"측정 중"}`
     +(d.du?` · 남은 사이클 ≈ ${Math.floor((1-worst)/d.du)}회`:"")+`</div>`;
   document.getElementById("rate").innerHTML=out;
   const wc=worst>0.95?"var(--bad)":worst>0.8?"var(--warn)":"var(--ok)";
   document.getElementById("hrate").innerHTML=`<b style="color:${wc}">한도 ${(worst*100).toFixed(0)}%</b>`;
 }else{
   document.getElementById("rate").innerHTML='<div class="dim">아직 한도 신호를 못 받았다(사이클 하나가 끝나면 보인다).</div>';
 }

 // 작업판
 document.getElementById("board").innerHTML=
   `<div class="dim" style="margin-bottom:6px">열림 ${d.tasks.length} · 완료 ${d.done}/${d.total}</div>`
   +d.tasks.map(t=>`<div class="t"><b class="mono">${esc(t.id)}</b>`
     +`<span class="mono dim">p${t.priority}</span>${pill(t.role)}`
     +`<span>${t.status==="blocked"?'<span class="bad">[막힘]</span> ':""}`
     +`${t.status==="doing"?'<span class="ok">[진행]</span> ':""}${esc(t.title)}</span></div>`).join("");

 // 내 말
 document.getElementById("say").innerHTML=d.say.length
   ? d.say.map(s=>`<div class="t"><span class="warn">안 읽힘</span><span>${esc(s.slice(0,160))}</span></div>`).join("")
   : '<div class="dim">안 읽힌 말 없음 — 다 처리됐다.</div>';

 // 사이클
 document.getElementById("cycles").innerHTML=d.cycles.map(c=>
   `<div class="t"><b class="mono">${c.cycle}</b>${pill(c.role)}`
   +`<span class="mono dim">${esc(c.engine)}:${esc(c.model)}</span>`
   +`<span class="${c.ok?"ok":"bad"}">${c.ok?"완료":"실패"}</span>`
   +`<span class="mono dim">${Math.round(c.sec/60)}분 · $${c.cost.toFixed(2)} · ${c.turns}턴</span></div>`).join("");

 document.getElementById("journal").textContent=d.journal;
}
async function say(){
 const el=document.getElementById("msg"); const v=el.value.trim(); if(!v) return;
 el.value=""; await fetch("/say",{method:"POST",body:v}); tick();
}
document.getElementById("msg").addEventListener("keydown",e=>{if(e.key==="Enter")say()});
tick(); setInterval(tick,2000);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        raw = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif path == "/api":
            self._send(200, json.dumps(snapshot(), ensure_ascii=False))
        else:
            self._send(404, "{}")

    def do_POST(self):
        if urlparse(self.path).path != "/say":
            return self._send(404, "{}")
        n = int(self.headers.get("Content-Length") or 0)
        msg = self.rfile.read(n).decode("utf-8", "replace").strip()
        if msg:
            # say.py와 **같은 파일·같은 형식**으로 쓴다. 창구가 둘이면 하나는 잊힌다.
            with open(os.path.join(HERE, "INBOX.md"), "a", encoding="utf-8") as f:
                f.write("\n- [ ] {} · {}\n".format(
                    datetime.now().strftime("%Y-%m-%d %H:%M"), " ".join(msg.split())))
        self._send(200, json.dumps({"ok": bool(msg)}))

    def log_message(self, *a):
        pass        # 2초마다 폴링한다 — 콘솔을 덮지 않는다


def main():
    ap = argparse.ArgumentParser(description="자동운전 모니터")
    ap.add_argument("--port", type=int, default=8099)
    ap.add_argument("--host", default="127.0.0.1",
                    help="0.0.0.0으로 두면 폰에서도 열린다(같은 공유기 안에서만)")
    a = ap.parse_args()
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    print("모니터: http://{}:{}/  (Ctrl+C로 끝)".format(
        "127.0.0.1" if a.host == "0.0.0.0" else a.host, a.port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

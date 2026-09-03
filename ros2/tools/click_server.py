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
    POST /run     {job,args}         job=stage|jog|grasp|grip|pose|park|loop
    POST /stop                       도는 일을 끊는다
"""

from __future__ import annotations

import argparse
import json
import os
import posixpath
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(REPO, "src"))

import arm_calib                                             # noqa: E402


def _kin_constants() -> dict:
    """**3D 미리보기용** 링크 길이·관절 한계 — 실측·보정 파일에서 뽑는다.

    ⚠ 여기서 절대 예외를 흘리지 않는다 — 이 계산이 실패해도 조작대 자체는
      떠야 한다(미리보기는 있으면 좋은 것이지 핵심 기능이 아니다). 그래서
      기본값으로 조용히 물러난다.
    ⚠ 관절 한계는 **arm_calib.Calib에서만** 가져온다 — tool_jog.py 등
      조그 스크립트의 legal()과 같은 계산이어야 한다. 예전엔 여기서 따로
      계산했는데, 그러면 이 패널은 "갈 수 있다"는데 조그는 거절하는 일이
      생긴다(2026-09-03 실측).
    """
    geom = {"z0": 119.5, "d0": -31.5, "l1": 116.5, "l2": 138.0, "l3": 168.0}
    try:
        from tomato_picker.hardware import kinematics as kin
        g = kin.ArmGeometry()
        geom = {"z0": g.z0, "d0": g.d0, "l1": g.l1, "l2": g.l2, "l3": g.l3}
    except Exception:                                      # noqa: BLE001
        pass
    joints = {j: {"min": -100.0, "max": 100.0} for j in
              ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")}
    try:
        calib = arm_calib.Calib()
        joints = {}
        for name in calib.zero:
            lim = calib.joint_range(name)
            if lim is not None:
                joints[name] = {"min": round(lim[0], 1), "max": round(lim[1], 1)}
    except Exception:                                      # noqa: BLE001
        pass
    # ⚠ **손-눈 보정(T_tool_cam)이 있으면 손목 카메라도 그린다** — 지금까지
    #   미리보기는 관절만 그려서, "카메라가 실제로 어디를 보고 있는가"는 숫자
    #   (t_tool_cam)로만 확인해야 했다. 사람이 |t|=250mm 같은 값을 보고
    #   "그게 팔 길이보다 긴데?"를 눈치채는 것보다, 팔 그림 옆에 카메라가
    #   허공에 붕 떠 있는 걸 보는 쪽이 훨씬 빨리 걸린다(2026-09-04, 실제로
    #   그런 물리적으로 말이 안 되는 해가 나온 적이 있다 — §손-눈 보정 부호).
    #   실패해도 조용히 없는 채로 둔다 — 미리보기는 핵심 기능이 아니다.
    eye = None
    try:
        from tomato_picker.hardware.eye import EyeConfig
        cfg = EyeConfig()
        if cfg.mount == "on_arm" and cfg.has_calibration:
            t = cfg.transform
            eye = {"R": t.R.flatten().tolist(), "t": t.t.tolist(),
                   "rms_mm": cfg.rms_mm, "good": cfg.good}
    except Exception:                                      # noqa: BLE001
        pass
    return {"geom": geom, "joints": joints, "eye": eye,
            "floor": -arm_calib.MOUNT_Z_MM + arm_calib.FLOOR_MARGIN_MM}


KIN = _kin_constants()

PY = os.environ.get("TOMATO_PY", "/home/server/lerobot/.venv/bin/python")
COLOR = "/dev/shm/d405_color.jpg"
DEPTH = "/dev/shm/d405_depth.npy"
META = "/dev/shm/d405_meta.json"
TARGET = os.path.expanduser("~/click_target.json")
GRIPF = os.path.expanduser("~/grip_uv.json")
# target_check.py(손-눈 보정용 4점 표적 인식)가 매번 남기는 실시간 상태 —
# SSH로만 도는 handeye_collect.py도 여기서 지켜볼 수 있게 화면에 겹쳐 그린다.
CALIB_VIEW = os.path.expanduser("~/target_view.json")
# 같은 이유로, SSH로 도는 팔 자세도 여기로 — 조작대의 "일" 체계를 안 거치는
# 스크립트는 로그(k3dParseLog)로 3D 미리보기가 안 따라오므로 파일로 남긴다.
ARM_POSE_VIEW = os.path.expanduser("~/arm_pose_view.json")
GRIP_UV = (471, 395)              # 집게를 여닫아 실측한 기본값
PARK = "60,65,0,-100,6"           # 열매가 보이던 대기 자세(도)


def voice_active():
    """⚠ `tomato-voice`(레거시 대시보드)가 팔 포트를 쥐고 있으면 여기서 하는
       모든 일이 조용히 실패한다 — 둘을 동시에 못 켠다(CLAUDE.md의 경계).
       2026-09-03 인수인계 직전에 이게 되살아나 있었다. 화면에 띄운다."""
    try:
        r = subprocess.run(["systemctl", "is-active", "tomato-voice"],
                           capture_output=True, text=True, timeout=4)
        return r.stdout.strip() == "active"
    except Exception:                                      # noqa: BLE001
        return False


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
            # ⚠ **PYTHONUNBUFFERED가 없으면 진행 상황이 안 보인다.** 자식
            #   파이썬은 표준출력이 파이프면(터미널이 아니면) 기본이 블록
            #   버퍼링이라, 몇 KB가 쌓이거나 끝나야 나온다 — 도는 중에 /log를
            #   봐도 몇 분째 명령줄 한 줄뿐이었다(2026-09-03, harvest_loop 디버깅
            #   중 발견). 그동안 걸음별 진행을 못 보고 완전히 끝날 때까지 기다려야 했다.
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            self.proc = subprocess.Popen(
                argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, cwd=os.path.join(HERE, "..", ".."),
                env=env, preexec_fn=os.setsid)
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
    if job == "loop":
        a = [PY, T("harvest_loop.py")]
        a += ["--hours", "%.2f" % num(args, "hours", 6, 0.1, 24)]
        a += ["--rest", "%.0f" % num(args, "rest", 25, 5, 300)]
        a += ["--stop-z", "%.0f" % num(args, "stop_z", 88, 0, 400)]
        return a
    if job == "grasp":
        a = [PY, T("stem_grasp.py"), "--aim", str(args.get("aim", "click"))]
        a += ["--steps", "%d" % int(num(args, "steps", 16, 1, 60)),
              "--adv", "%.1f" % num(args, "adv", 8, 1, 30),
              "--max-turn", "%.1f" % num(args, "max_turn", 2, 0.3, 8),
              "--gain", "%.2f" % num(args, "gain", 0.35, 0.05, 1.0),
              "--tol", "%.0f" % num(args, "tol", 28, 5, 120),
              "--rejacobian", "%d" % int(num(args, "rejacobian", 6, 1, 20)),
              "--stop-z", "%.0f" % num(args, "stop_z", 88, 0, 400),
              "--max-adv", "%.0f" % num(args, "max_adv", 200, 20, 500)]
        if args.get("no_close"):
            a += ["--no-close"]
        if args.get("dry"):
            a += ["--dry"]
        return a
    raise ValueError("모르는 일: %s" % job)


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>토마토 조작대</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
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
.k3d{width:100%;height:190px;border-radius:9px;background:#11151c;touch-action:none}
.legend{display:flex;flex-wrap:wrap;gap:8px;font-size:11px;color:var(--dim);margin-top:5px}
.legend b{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px;vertical-align:middle}
.jgrid{display:grid;grid-template-columns:1fr 1fr;gap:4px 8px;margin-top:6px}
.jrow{display:grid;grid-template-columns:56px 1fr;gap:5px;align-items:center;font-size:11.5px}
.jrow input{width:100%;padding:4px 5px}
.jrow .lim{font-family:ui-monospace,Menlo,monospace;font-size:9.5px;color:var(--dim);grid-column:1/-1;margin-top:-2px}
.jbad{color:var(--hit)!important;font-weight:700}
.h2tgl{cursor:pointer;user-select:none;display:flex;align-items:center;justify-content:space-between}
.tgl{color:var(--dim);font-size:11px}
.cbody[hidden]{display:none}
</style></head><body><div class="wrap">
<header><h1>토마토 조작대</h1>
<span class="sub">화면을 눌러 <b>줄기</b>를 정하고, 오른쪽에서 시킨다. 사람과 에이전트가 같은 API를 쓴다.</span>
</header>

<div class="col">
  <div id="warn" style="display:none;background:#fdecea;color:#8f1d14;border:1px solid #e4572e;
    border-radius:9px;padding:9px 11px;font-size:13.5px;font-weight:600"></div>
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

  <div class="card"><h2>3D 미리보기 — 지금 vs 목표</h2>
    <canvas class="k3d" id="k3d"></canvas>
    <div class="legend">
      <span><b style="background:#4a9eff"></b>지금</span>
      <span><b style="background:#ff9a3d"></b>목표</span>
      <span><b style="background:#3a4550"></b>바닥/한계면</span>
      <span><b style="background:#22d3ee"></b>손목 카메라(보정 있으면)</span>
      <span>드래그=회전·휠=확대</span>
    </div>
    <div class="k" id="k3dWhy" style="margin-top:5px"></div>
    <h2 class="h2tgl" onclick="toggleCard(this)" style="margin-top:8px;font-size:11.5px">관절값 직접 편집 <span class="tgl">▸</span></h2>
    <div class="cbody" id="cb_edit" hidden>
      <div class="jgrid" id="jrows"></div>
      <div class="row" style="margin-top:8px">
        <button onclick="k3dLoadCurrent()">지금 값 불러오기</button>
        <button class="go" onclick="k3dSend()">이 목표로 보내기</button>
      </div>
    </div>
  </div>

  <div class="card"><h2 class="h2tgl" onclick="toggleCard(this)">잡기 <span class="tgl">▸</span></h2>
    <div class="cbody" id="cb_grasp" hidden>
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
    <div class="row" style="margin-top:6px">
      <label>믿는거리(mm)<input id="g_maxadv" value="200"></label>
    </div>
    <div class="k" style="margin-top:2px">깊이는 못 믿을 수 있다 — 다 합쳐 이 거리 이상은
      절대 안 나아간다(닿을 때까지가 아니라).</div>
    <div class="row" style="margin-top:8px">
      <button class="go" onclick="run('grasp',grasp(false))">잡으러 간다</button>
      <button onclick="run('grasp',grasp(true))">안 닫고 가기</button>
      <button onclick="run('grasp',Object.assign(grasp(true),{dry:1}))">겨냥만 본다</button>
    </div>
    </div>
  </div>

  <div class="card"><h2 class="h2tgl" onclick="toggleCard(this)">무인 반복 <span class="tgl">▸</span></h2>
    <div class="cbody" id="cb_loop" hidden>
    <div class="row">
      <label>시간(h)<input id="l_hours" value="6"></label>
      <label>휴식(초)<input id="l_rest" value="25"></label>
    </div>
    <div class="row" style="margin-top:8px">
      <button class="go" onclick="run('loop',{hours:val('l_hours',6),rest:val('l_rest',25)})">사람 없이 반복 시작</button>
    </div>
    <div class="k" style="margin-top:6px">열매를 스스로 찾아(--aim top) 잡고 놓기를 반복한다.
      열매가 없으면 노란 테이프 아래를 겨눈다. 멈추려면 아래 "지금 하는 일 끊기".</div>
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
var W=848,H=480,live=true,pt=null,gp={u:471,v:395},mode='target',since=0,calibView=null;
var im=document.getElementById('im'),ov=document.getElementById('ov'),stage=document.getElementById('stage');
function draw(){
  var s='<line x1="'+(gp.u-16)+'" y1="'+gp.v+'" x2="'+(gp.u+16)+'" y2="'+gp.v+'" stroke="#2f6fb0" stroke-width="3"/>'
       +'<line x1="'+gp.u+'" y1="'+(gp.v-16)+'" x2="'+gp.u+'" y2="'+(gp.v+16)+'" stroke="#2f6fb0" stroke-width="3"/>';
  if(pt){s+='<circle cx="'+pt.u+'" cy="'+pt.v+'" r="13" fill="none" stroke="#e4572e" stroke-width="4"/>'
        +'<circle cx="'+pt.u+'" cy="'+pt.v+'" r="2.5" fill="#e4572e"/>';}
  // ⚠ handeye_collect.py는 SSH로만 돈다 — target_check.py가 남긴 상태를 여기
  //   겹쳐 그려서, 사람이 이 화면만 보고도 뭘 찾았는지/놓쳤는지 알 수 있게 한다.
  if(calibView){
    if(calibView.ok && calibView.dots){
      var recon=calibView.reconstructed, names=['tl','tr','bl','br'], pts=[];
      names.forEach(function(k){
        var p=calibView.dots[k]; if(!p) return;
        pts.push(p);
        var bad=(k===recon);
        s+='<circle cx="'+p[0]+'" cy="'+p[1]+'" r="10" fill="none" '
          +'stroke="'+(bad?'#c9822a':'#3f7d3a')+'" stroke-width="3"'
          +(bad?' stroke-dasharray="4 3"':'')+'/>';
      });
      if(pts.length>=3){
        s+='<polygon points="'+pts.map(function(p){return p[0]+','+p[1];}).join(' ')
          +'" fill="none" stroke="#3f7d3a" stroke-width="1.5" stroke-dasharray="3 3" opacity="0.55"/>';
      }
    } else if(!calibView.ok){
      s+='<text x="10" y="26" fill="#e4572e" font-size="15" '
        +'style="paint-order:stroke;stroke:#000;stroke-width:3px">보정 표적: '
        +(calibView.why||'안 보임')+'</text>';
    }
  }
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
function toggleCard(h){
  var body=h.nextElementSibling, tgl=h.querySelector('.tgl');
  var willShow=body.hasAttribute('hidden');
  if(willShow) body.removeAttribute('hidden'); else body.setAttribute('hidden','');
  tgl.textContent=willShow?'▾':'▸';
}
function val(id,d){var x=parseFloat(document.getElementById(id).value);return isNaN(x)?d:x;}
function grasp(nc){return {steps:val('g_steps',16),adv:val('g_adv',8),max_turn:val('g_turn',2),
  gain:val('g_gain',0.35),tol:val('g_tol',28),stop_z:val('g_stop',88),max_adv:val('g_maxadv',200),
  no_close:nc?1:0};}
function jog(k,s){
  var amt=s*val('j_mm',20), free=document.getElementById('j_free').checked;
  tgtJ=jogPreviewTarget(curJ,k,amt,free);
  JNAMES.forEach(function(n){var el=document.getElementById('tj_'+n); if(el) el.value=tgtJ[n].toFixed(1);});
  k3dRenderOnce();
  var a={};a[k]=amt; if(free)a.free_pitch=1; run('jog',a);
}
function run(job,args){post('/run',{job:job,args:args}).then(function(j){
  if(!j.ok) alert(j.why||'거절됨');});}
function stop(){post('/stop');}
function state(){
  fetch('/state').then(function(r){return r.json();}).then(function(j){
    calibView=j.calib_view||null;
    if(j.arm_pose){
      // SSH로 직접 도는 스크립트(handeye_collect.py 등)는 조작대의 "일"
      // 체계를 안 거쳐 로그 기반 추적(k3dParseLog)이 안 통한다 — 이 자리가
      // 그 빈틈을 메운다.
      JNAMES.forEach(function(n){ if(j.arm_pose[n]!=null) curJ[n]=j.arm_pose[n]; });
      k3dRenderOnce();
    }
    if(j.gu!=null) gp={u:j.gu,v:j.gv};
    if(j.u!=null){pt={u:j.u,v:j.v};
      document.getElementById('cur').textContent='표적 ('+j.u+', '+j.v+')  깊이 '+(j.z>0?j.z.toFixed(0)+'mm':'없음');}
    document.getElementById('age').textContent='프레임 '+j.age.toFixed(1)+'초 전'+(j.age>5?' ⚠ depth-cam 확인':'');
    var wv=document.getElementById('warn');
    if(j.voice){wv.textContent='⚠ tomato-voice 가 켜져 있다 — 팔 포트를 뺏겨 여기서 시키는 일이 전부 실패한다. '
      +'젯슨에서 sudo systemctl stop tomato-voice';wv.style.display='';}
    else wv.style.display='none';
    var e=document.getElementById('jobline');
    e.textContent=j.running?('도는 중: '+j.job):('놀고 있음'+(j.job?(' (마지막 '+j.job+', rc='+j.rc+')'):''));
    e.className='v '+(j.running?'busy':'idle');
    draw();}).catch(function(){});
}
function log(){
  fetch('/log?since='+since).then(function(r){return r.json();}).then(function(j){
    if(j.lines.length){var e=document.getElementById('log');
      e.textContent+=(e.textContent?'\n':'')+j.lines.join('\n');
      e.scrollTop=e.scrollHeight;
      k3dParseLog(j.lines.join('\n'));}
    since=j.next;}).catch(function(){});
}

// ── 3D 미리보기 — 지금/목표 자세를 같은 기구학으로 그린다 ──────────────
// (ros2/src/.../hardware/kinematics.py 의 forward()를 그대로 옮긴 것)
var KIN = __KIN_JSON__;
var EYE = KIN.eye;   // T_tool_cam(있으면) — 손목 카메라를 3D 미리보기에 같이 그린다
// ⚠ **부호가 `ros2/tools/handeye_collect.py`의 `ROLL_SIGN`과 반드시 같아야 한다.**
//   2026-09-04 실측으로 -1 확정(+1은 벽 법선 어긋남이 93.8°, -1은 11.4°였다).
//   `handeye.py`의 `tool_frame()`은 아예 다른 규약(roll을 안 넣는다)이니 그쪽과
//   헷갈리지 말 것 — 여긴 손목 카메라 보정 쪽(`handeye_resolve.py`) 규약이다.
var CAM_ROLL_SIGN = -1.0;
var JNAMES=["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"];
var JLABELS={shoulder_pan:"어깨돌림(pan)",shoulder_lift:"어깨듦(lift)",elbow_flex:"팔꿈치",
  wrist_flex:"손목숙임",wrist_roll:"손목돌림"};
var curJ={shoulder_pan:59.6,shoulder_lift:65,elbow_flex:0,wrist_flex:-100,wrist_roll:6};
var tgtJ={shoulder_pan:59.6,shoulder_lift:65,elbow_flex:0,wrist_flex:-100,wrist_roll:6};

function fk(j){
  var d=Math.PI/180;
  var pan=j.shoulder_pan*d, lift=j.shoulder_lift*d, elbow=j.elbow_flex*d, wrist=j.wrist_flex*d;
  var a1=lift, a2=lift+elbow, a3=lift+elbow+wrist;
  var g=KIN.geom;
  var rz=[[0,0],[g.d0,g.z0]];
  rz.push([rz[1][0]+g.l1*Math.cos(a1), rz[1][1]+g.l1*Math.sin(a1)]);
  rz.push([rz[2][0]+g.l2*Math.cos(a2), rz[2][1]+g.l2*Math.sin(a2)]);
  rz.push([rz[3][0]+g.l3*Math.cos(a3), rz[3][1]+g.l3*Math.sin(a3)]);
  // ⚠ 화면 좌우(X)만 뒤집는다 — 그리는 용도뿐이라(야코비안 계산은
  //   kinForward()가 따로 한다) 안전하다. 2026-09-03: 실물과 비교해보니
  //   미리보기가 좌우 반전으로 보였다.
  return rz.map(function(p){return new THREE.Vector3(-p[0]*Math.cos(pan), p[1], -p[0]*Math.sin(pan));});
}
function kinForward(j){
  // kinematics.py forward()를 그대로 — (x,y,z,pitch)를 KIN 좌표(회전 전)로.
  var d=Math.PI/180;
  var pan=j.shoulder_pan*d, lift=j.shoulder_lift*d, elbow=j.elbow_flex*d, wrist=j.wrist_flex*d;
  var a1=lift, a2=lift+elbow, a3=lift+elbow+wrist;
  var g=KIN.geom;
  var r=g.d0+g.l1*Math.cos(a1)+g.l2*Math.cos(a2)+g.l3*Math.cos(a3);
  var z=g.z0+g.l1*Math.sin(a1)+g.l2*Math.sin(a2)+g.l3*Math.sin(a3);
  return {x:r*Math.cos(pan), y:r*Math.sin(pan), z:z, pitch:a3/d};
}
function solve4x4(Ain,bin){
  var n=4, M=[]; for(var i=0;i<n;i++) M.push(Ain[i].concat([bin[i]]));
  for(var col=0;col<n;col++){
    var piv=col;
    for(var r=col+1;r<n;r++) if(Math.abs(M[r][col])>Math.abs(M[piv][col])) piv=r;
    var tmp=M[col]; M[col]=M[piv]; M[piv]=tmp;
    if(Math.abs(M[col][col])<1e-9) continue;
    for(var r2=0;r2<n;r2++){
      if(r2===col) continue;
      var f=M[r2][col]/M[col][col];
      for(var c=col;c<=n;c++) M[r2][c]-=f*M[col][c];
    }
  }
  var x=[]; for(var i2=0;i2<n;i2++) x.push(Math.abs(M[i2][i2])<1e-9?0:M[i2][n]/M[i2][i2]);
  return x;
}
function solveDamped(A,b,lambda){
  // ⚠ **특이(singular) 자세 근처에서 직접 풀이가 터진다.** 팔꿈치가 거의 편
  //   자세(elbow_flex≈0, 대기자세가 정확히 이렇다) 근처는 이 4x4가 거의
  //   특이행렬이라, np.linalg.solve든 손으로 짠 가우스 소거든 1e14 같은
  //   말도 안 되는 해를 낸다(실측). tool_jog.py는 lstsq(SVD)라 안 이러는데,
  //   여기서는 SVD 없이 감쇠 최소자승(A^T A + λI)으로 같은 안정성을 얻는다 —
  //   미리보기라 픽셀/각도 정밀도보다 "터지지 않는 것"이 우선이다.
  var n=A.length, m=A[0].length, AtA=[], Atb=[];
  for(var i=0;i<m;i++){
    AtA.push(new Array(m).fill(0));
    var s=0; for(var k=0;k<n;k++) s+=A[k][i]*b[k];
    Atb.push(s);
  }
  for(var i2=0;i2<m;i2++) for(var j=0;j<m;j++){
    var s2=0; for(var k2=0;k2<n;k2++) s2+=A[k2][i2]*A[k2][j];
    AtA[i2][j]=s2+(i2===j?lambda:0);
  }
  return solve4x4(AtA,Atb);
}
function jogPreviewTarget(cur,kind,amount,freePitch){
  // 조그 버튼이 실제로 보내기 **전에** 어디로 향하는지 미리 계산한다 —
  // tool_jog.py의 step_keep()/move_base()(피치 고정 4관절 풀이)를 그대로 옮긴 것.
  var j=Object.assign({},cur);
  if(kind==='pitch'){ j.wrist_flex=cur.wrist_flex+amount; return j; }
  var p0=kinForward(cur), want=[0,0,0];
  if(kind==='dz'){ want=[0,0,amount]; }
  else if(kind==='along' || kind==='horiz'){
    var th=Math.atan2(p0.y,p0.x), ph=p0.pitch*Math.PI/180;
    var appr=[Math.cos(ph)*Math.cos(th), Math.cos(ph)*Math.sin(th), Math.sin(ph)];
    if(kind==='horiz'){
      var n=Math.hypot(appr[0],appr[1]);
      if(n<1e-6) return j;
      appr=[appr[0]/n, appr[1]/n, 0];
    }
    want=[appr[0]*amount, appr[1]*amount, appr[2]*amount];
  } else return j;
  var cols=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex'];
  var A=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,1,1,1]];
  for(var ci=0; ci<4; ci++){
    var e=Object.assign({},cur); e[cols[ci]]=cur[cols[ci]]+0.5;
    var p1=kinForward(e);
    A[0][ci]=(p1.x-p0.x)/0.5; A[1][ci]=(p1.y-p0.y)/0.5; A[2][ci]=(p1.z-p0.z)/0.5;
  }
  var b=[want[0],want[1],want[2],0];
  var sol=freePitch ? solveDamped([A[0],A[1],A[2]],[want[0],want[1],want[2]],0.5)
                    : solveDamped(A,b,0.5);
  cols.forEach(function(c,i){ j[c]=cur[c]+sol[i]; });
  return j;
}
function toolAxesBase(j){
  // 관절각 → base 좌표의 (approach, lateral, up) — handeye_resolve.py의
  // tool_frame()과 같은 식(wrist_roll 포함). fk()/kinForward()와 달리 손목
  // 카메라는 pan 평면 밖으로도 튀어나와 있을 수 있어 3축이 다 필요하다.
  var d=Math.PI/180;
  var pan=j.shoulder_pan*d, a3=(j.shoulder_lift+j.elbow_flex+j.wrist_flex)*d;
  var appr=[Math.cos(a3)*Math.cos(pan), Math.cos(a3)*Math.sin(pan), Math.sin(a3)];
  var lat0=[-Math.sin(pan), Math.cos(pan), 0];
  var up0=[-Math.sin(a3)*Math.cos(pan), -Math.sin(a3)*Math.sin(pan), Math.cos(a3)];
  var roll=CAM_ROLL_SIGN*j.wrist_roll*d, cr=Math.cos(roll), sr=Math.sin(roll);
  var lat=[lat0[0]*cr+up0[0]*sr, lat0[1]*cr+up0[1]*sr, lat0[2]*cr+up0[2]*sr];
  var upr=[-lat0[0]*sr+up0[0]*cr, -lat0[1]*sr+up0[1]*cr, -lat0[2]*sr+up0[2]*cr];
  return [appr, lat, upr];   // 열이 approach/lat/up인 회전행렬의 "행 나열"
}
function mul3(M9,v){
  return [M9[0]*v[0]+M9[1]*v[1]+M9[2]*v[2],
          M9[3]*v[0]+M9[4]*v[1]+M9[5]*v[2],
          M9[6]*v[0]+M9[7]*v[1]+M9[8]*v[2]];
}
function baseToDraw(x,y,z){
  // fk()가 쓰는 것과 **같은** base→그리기 사영(주석 참조): (x,y,z) → (-x,z,-y).
  return new THREE.Vector3(-x, z, -y);
}
function camWorld(j){
  // 손목 카메라의 base 좌표 위치 + 광학축(조준) 방향 — 없으면 null.
  if(!EYE) return null;
  var Rcols=toolAxesBase(j);         // Rcols[k] = approach/lat/up의 base 성분
  var p0=kinForward(j);
  var camTool=EYE.t;                 // 카메라 원점은 도구좌표에서 그냥 t_x다
  var aimTool=mul3(EYE.R,[0,0,1]);   // 카메라 광학축(+z, cam 로컬) → 도구좌표 방향
  var camBase=[p0.x,p0.y,p0.z], aimBase=[0,0,0];
  for(var k=0;k<3;k++){
    camBase[0]+=Rcols[k][0]*camTool[k]; camBase[1]+=Rcols[k][1]*camTool[k]; camBase[2]+=Rcols[k][2]*camTool[k];
    aimBase[0]+=Rcols[k][0]*aimTool[k]; aimBase[1]+=Rcols[k][1]*aimTool[k]; aimBase[2]+=Rcols[k][2]*aimTool[k];
  }
  return {pos:baseToDraw(camBase[0],camBase[1],camBase[2]),
          aim:baseToDraw(camBase[0]+aimBase[0]*50,camBase[1]+aimBase[1]*50,camBase[2]+aimBase[2]*50)};
}
function k3dAddCam(group,j,color){
  var c=camWorld(j);
  if(!c) return;
  var m=new THREE.Mesh(new THREE.BoxGeometry(16,11,11),
    new THREE.MeshStandardMaterial({color:color}));
  m.position.copy(c.pos);
  group.add(m);
  var lg=new THREE.BufferGeometry().setFromPoints([c.pos,c.aim]);
  group.add(new THREE.Line(lg,new THREE.LineBasicMaterial({color:color})));
}
function jointBad(name, deg){
  var l=KIN.joints[name]; if(!l) return false;
  var span=(l.max-l.min)||1;
  return deg < l.min + span*0.03 || deg > l.max - span*0.03;
}

var k3dScene,k3dCam,k3dRenderer,curGroup,tgtGroup,k3dVecLine,k3dReady=false;
var k3dRotX=-0.35,k3dRotY=0.8,k3dDist=750;
function k3dSeg(A,B,color){
  var dir=new THREE.Vector3().subVectors(B,A); var len=dir.length()||0.001;
  var m=new THREE.Mesh(new THREE.CylinderGeometry(6,6,len,10),
    new THREE.MeshStandardMaterial({color:color}));
  m.position.copy(A).addScaledVector(dir,0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), dir.clone().normalize());
  return m;
}
function k3dJoint(P,color){
  var m=new THREE.Mesh(new THREE.SphereGeometry(11,12,10),
    new THREE.MeshStandardMaterial({color:color}));
  m.position.copy(P); return m;
}
function k3dBuild(group,j,base,bad){
  while(group.children.length) group.remove(group.children[0]);
  var pts=fk(j), jn=["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex"];
  for(var i=0;i<pts.length-1;i++)
    group.add(k3dSeg(pts[i],pts[i+1], jointBad(jn[i],j[jn[i]])?bad:base));
  for(var k=1;k<pts.length;k++)
    group.add(k3dJoint(pts[k], jointBad(jn[k-1],j[jn[k-1]])?bad:base));
  group.add(k3dJoint(pts[0],0x555f6b));
  return pts[pts.length-1];
}
function k3dCamUpdate(){
  var cx=Math.sin(k3dRotY)*Math.cos(k3dRotX)*k3dDist;
  var cz=Math.cos(k3dRotY)*Math.cos(k3dRotX)*k3dDist;
  var cy=Math.sin(k3dRotX)*k3dDist+150;
  k3dCam.position.set(cx,cy,cz); k3dCam.lookAt(0,150,0);
}
function k3dWhy(){
  var msgs=[];
  if(!EYE) msgs.push('손목 카메라 보정 없음(~/arm_eye.json) — 카메라 마커 안 보임');
  else if(!EYE.good) msgs.push('카메라 보정 잔차 '+EYE.rms_mm.toFixed(0)+'mm(기준 15mm 초과) — 마커 위치를 믿지 말 것');
  JNAMES.slice(0,4).forEach(function(n){
    var bad=jointBad(n,tgtJ[n]);
    var el=document.getElementById('tl_'+n);
    if(el) el.className='lim'+(bad?' jbad':'');
    if(bad){var l=KIN.joints[n];
      msgs.push(JLABELS[n]+' 한계 근처('+tgtJ[n].toFixed(0)+'° / 한계 '+l.min.toFixed(0)+'~'+l.max.toFixed(0)+'°)');}
  });
  var tp=fk(tgtJ)[4];
  if(tp.y < KIN.floor+5) msgs.push('목표 손끝이 바닥 근처/아래 (높이 '+tp.y.toFixed(0)+'mm)');
  document.getElementById('k3dWhy').textContent = msgs.length? ('⚠ 이래서 그쪽으로 안 된다 — '+msgs.join(' · '))
    : '목표가 관절 한계 안입니다 — 이 방향은 갈 수 있습니다.';
}
function k3dRenderOnce(){
  if(!k3dReady) return;
  k3dCamUpdate();
  var curTip=k3dBuild(curGroup,curJ,0x4a9eff,0xff3b3b);
  var tgtTip=k3dBuild(tgtGroup,tgtJ,0xff9a3d,0xff3b3b);
  k3dAddCam(curGroup,curJ,0x22d3ee);
  k3dAddCam(tgtGroup,tgtJ,0x22d3ee);
  if(k3dVecLine) k3dScene.remove(k3dVecLine);
  var lg=new THREE.BufferGeometry().setFromPoints([curTip,tgtTip]);
  k3dVecLine=new THREE.Line(lg,new THREE.LineBasicMaterial({color:0xffe066}));
  k3dScene.add(k3dVecLine);
  k3dRenderer.render(k3dScene,k3dCam);
  k3dWhy();
}
function k3dParseLog(text){
  // ⚠ 앞 태그 단어(지금/끝/자세/관절이름 등)로 "목표"만 따로 가른다 —
  //   stem_grasp.py --dry가 "목표"로 찍으면 tgtJ(주황, 미리보기)로 가고,
  //   나머지는 전부 curJ(파랑, 실제 자세)로 간다. arm_stage.py도 시작할 때
  //   "목표" 한 줄을 찍는데, 그것도 이제 tgtJ로 가는 게 맞다 — 아직 안
  //   움직인 계획일 뿐 실제 자세가 아니었다.
  var re=/(\S+)\s+shoulder=\s*(-?[\d.]+)\s+shoulder=\s*(-?[\d.]+)\s+elbow=\s*(-?[\d.]+)\s+wrist=\s*(-?[\d.]+)\s+wrist=\s*(-?[\d.]+)/g;
  var m,lastCur=null,lastTgt=null;
  while((m=re.exec(text))!==null){ if(m[1]==='목표') lastTgt=m; else lastCur=m; }
  if(lastCur){curJ.shoulder_pan=parseFloat(lastCur[2]);curJ.shoulder_lift=parseFloat(lastCur[3]);
    curJ.elbow_flex=parseFloat(lastCur[4]);curJ.wrist_flex=parseFloat(lastCur[5]);curJ.wrist_roll=parseFloat(lastCur[6]);}
  if(lastTgt){tgtJ.shoulder_pan=parseFloat(lastTgt[2]);tgtJ.shoulder_lift=parseFloat(lastTgt[3]);
    tgtJ.elbow_flex=parseFloat(lastTgt[4]);tgtJ.wrist_flex=parseFloat(lastTgt[5]);tgtJ.wrist_roll=parseFloat(lastTgt[6]);
    JNAMES.forEach(function(n){var el=document.getElementById('tj_'+n); if(el) el.value=tgtJ[n].toFixed(1);});}
  if(lastCur||lastTgt) k3dRenderOnce();
}
function k3dInitRows(){
  var host=document.getElementById('jrows'), html='';
  JNAMES.forEach(function(n){
    var l=KIN.joints[n]||{min:-100,max:100};
    html+='<div class="jrow"><span>'+JLABELS[n]+'</span>'+
      '<input type="number" step="1" id="tj_'+n+'" value="'+tgtJ[n].toFixed(1)+'">'+
      '<span class="lim" id="tl_'+n+'">'+l.min.toFixed(0)+'°~'+l.max.toFixed(0)+'°</span></div>';
  });
  host.innerHTML=html;
  JNAMES.forEach(function(n){
    document.getElementById('tj_'+n).addEventListener('input',function(){
      tgtJ[n]=parseFloat(this.value)||0; k3dRenderOnce();});
  });
}
function k3dLoadCurrent(){
  JNAMES.forEach(function(n){tgtJ[n]=curJ[n];
    var el=document.getElementById('tj_'+n); if(el) el.value=curJ[n].toFixed(1);});
  k3dRenderOnce();
}
function k3dSend(){
  var t=[tgtJ.shoulder_pan,tgtJ.shoulder_lift,tgtJ.elbow_flex,tgtJ.wrist_flex,tgtJ.wrist_roll]
    .map(function(v){return v.toFixed(1);}).join(',');
  run('stage',{target:t});
}
function k3dInit(){
  var cv=document.getElementById('k3d');
  k3dScene=new THREE.Scene();
  k3dCam=new THREE.PerspectiveCamera(45,(cv.clientWidth||300)/(cv.clientHeight||300),1,5000);
  k3dRenderer=new THREE.WebGLRenderer({canvas:cv,antialias:true});
  function resize(){var w=cv.clientWidth||300,h=cv.clientHeight||300;
    k3dRenderer.setSize(w,h,false); k3dCam.aspect=w/h; k3dCam.updateProjectionMatrix(); k3dRenderOnce();}
  window.addEventListener('resize',resize);
  k3dScene.add(new THREE.AmbientLight(0xffffff,0.65));
  var dl=new THREE.DirectionalLight(0xffffff,0.7); dl.position.set(300,600,400); k3dScene.add(dl);
  var grid=new THREE.GridHelper(800,16,0x3a4550,0x232a33); grid.position.y=KIN.floor; k3dScene.add(grid);
  var fm=new THREE.Mesh(new THREE.PlaneGeometry(800,800),
    new THREE.MeshBasicMaterial({color:0x2a323d,transparent:true,opacity:0.25,side:THREE.DoubleSide}));
  fm.rotation.x=-Math.PI/2; fm.position.y=KIN.floor; k3dScene.add(fm);
  k3dScene.add(new THREE.AxesHelper(120));
  curGroup=new THREE.Group(); tgtGroup=new THREE.Group();
  k3dScene.add(curGroup); k3dScene.add(tgtGroup);
  var dragging=false,lastX=0,lastY=0;
  cv.addEventListener('pointerdown',function(e){dragging=true;lastX=e.clientX;lastY=e.clientY;});
  window.addEventListener('pointerup',function(){dragging=false;});
  window.addEventListener('pointermove',function(e){
    if(!dragging) return;
    k3dRotY+=(e.clientX-lastX)*0.01; k3dRotX+=(e.clientY-lastY)*0.01;
    k3dRotX=Math.max(-1.3,Math.min(1.3,k3dRotX));
    lastX=e.clientX; lastY=e.clientY; k3dRenderOnce();
  });
  cv.addEventListener('wheel',function(e){e.preventDefault();
    k3dDist=Math.max(200,Math.min(2500,k3dDist+e.deltaY)); k3dRenderOnce();},{passive:false});
  resize(); k3dReady=true; k3dRenderOnce();
}
k3dInit(); k3dInitRows();

setInterval(tick,400); setInterval(state,1200); setInterval(log,900); tick(); state(); log();
</script></body></html>"""
PAGE = PAGE.replace("__KIN_JSON__", json.dumps(KIN))


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
                   "log_len": len(JOB.lines), "voice": voice_active()}
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
            if os.path.exists(CALIB_VIEW):
                try:
                    cv_ = json.load(open(CALIB_VIEW))
                    if time.time() - float(cv_.get("ts", 0)) < 5.0:
                        out["calib_view"] = cv_
                except Exception:                          # noqa: BLE001
                    pass
            if os.path.exists(ARM_POSE_VIEW):
                try:
                    pv_ = json.load(open(ARM_POSE_VIEW))
                    if time.time() - float(pv_.get("ts", 0)) < 5.0:
                        out["arm_pose"] = pv_["deg"]
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

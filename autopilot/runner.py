#!/usr/bin/env python3
"""자동운전 감독자 — `claude -p`를 역할별로 번갈아 띄워 15일 동안 일을 안 끊는다.

  python autopilot/runner.py run            # 무한 루프(기본). 데드라인까지.
  python autopilot/runner.py once --role builder
  python autopilot/runner.py status
  python autopilot/runner.py probe          # 젯슨 IP만 찾아본다

왜 순차인가 — 네 역할이 같은 작업 트리와 같은 git 인덱스를 쓴다. 동시에 돌리면
서로의 편집을 덮고 `index.lock`에서 싸운다. 대신 한 바퀴를 짧게(=자주) 돈다.
연속성은 문맥이 아니라 **파일**(작업판·일지·git)이 잇는다 — 매 사이클은 새 세션이다.

멈추는 법: `autopilot/PAUSE` 파일을 만들면 쉬고, `autopilot/STOP`이면 끝낸다.
"""
import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timedelta

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STATE_DIR = os.path.join(HERE, "state")
LOG_DIR = os.path.join(HERE, "logs")
JOURNAL_DIR = os.path.join(HERE, "journal")
CONFIG = os.path.join(HERE, "config.json")
STATE = os.path.join(STATE_DIR, "state.json")
HEARTBEAT = os.path.join(STATE_DIR, "heartbeat.json")
LIVE = os.path.join(STATE_DIR, "live.json")
PAUSE = os.path.join(HERE, "PAUSE")
STOP = os.path.join(HERE, "STOP")

DEFAULTS = {
    "branch": "feat/base-firmware-v2-and-arm-presets",
    "model": "opus",
    # 역할마다 모델을 나눈다 — 한도는 돈이 아니라 시간이고, 넷 다 opus면 이틀이면 벽을 친다.
    # 만드는 쪽(planner/builder)은 판단이 비싸고, 보는 쪽(auditor/tester)은 대체로 대조와 실행이다.
    "models": {},
    "fallback_model": "sonnet",
    "min_cycle_interval_sec": 3600,
    "rotation": ["planner", "builder", "tester", "builder", "auditor", "builder", "tester"],
    "cycle_timeout_sec": 2700,
    "sleep_between_sec": 90,
    "days": 15,
    "push": True,
    "hardware_allowed": True,
    "jetson_user": "server",
    "jetson_key": "C:/Users/parkg/.ssh/id_ed25519",
    "jetson_candidates": ["192.168.0.8", "192.168.0.6"],
    "jetson_subnet": "192.168.0",
    "jetson_reprobe_cycles": 8,
    "daily_cost_cap_usd": 0,
    "max_consecutive_fails": 6,
    # 엔진 배정 — **사고는 클로드, 코딩 유닛은 제미나이.**
    # 클로드 주간 한도가 벽이므로, 손이 많이 가는 역할을 다른 계정의 무료 등급으로 내린다.
    "engines": {},                 # 예: {"builder": "gemini", "tester": "gemini"}
    # 엔진 정의 — 새 에이전트 CLI(antigravity 등)는 **여기 한 칸만 더하면** 붙는다.
    #   exe   실행파일 이름(PATH에서 찾는다)
    #   args  인자. {pfile}=이번 사이클 지시가 든 파일, {model}, {role}, {safety}
    #   ready_env / ready_files  이 중 하나라도 있어야 '자격 있음'으로 본다(없으면 클로드로)
    "engine_defs": {
        "gemini": {
            "exe": "gemini",
            "args": ["-p", "{ask}", "-o", "stream-json", "--yolo", "-m", "{model}"],
            "model": "gemini-2.5-pro",
            "ready_env": ["GEMINI_API_KEY", "GOOGLE_API_KEY"],
            # 개인 OAuth 로그인이 막혀서(2026-09-18, Antigravity로 통합 안내) API 키로 붙는다.
            # aistudio.google.com/apikey 에서 발급한 키를 이 파일 하나에 넣어 두면
            # 시스템 환경변수를 안 건드리고도(재로그인 불필요) 다음 사이클부터 바로 쓰인다.
            "api_key_file": "autopilot/secrets/gemini_api_key.txt",
            "api_key_env": "GEMINI_API_KEY",
        },
        "agy": {
            # Antigravity CLI — 클로드와 인터페이스가 거의 같다(같은 하니스 계열).
            # 사람이 이미 로그인해 뒀다(2026-09-18) — ready_env/ready_files 없이 그냥 쓴다.
            "exe": "agy",
            "args": ["-p", "{ask}", "--output-format", "stream-json",
                     "--dangerously-skip-permissions", "--model", "{model}"],
            "model": "claude-sonnet-4-6",
        },
    },
    # agy.exe가 아직 PATH에 없을 수 있다 — 있으면 shutil.which보다 이 절대경로를 먼저 본다.
    "engine_paths": {"agy": "C:/Users/parkg/AppData/Local/agy/bin/agy.exe"},
    "saver": False,                # 절약 모드: 클로드 쪽 문맥을 줄이고 값싼 모델을 쓴다
    "cheap_when_tight": True,      # 한도가 빠듯하면 전 역할을 fallback_model로
    "cheap_below_remaining": 0.10,
    "rate_reserve": 0.02,
    "du_guess": 0.01,             # 안 재 봤을 때 가정하는 사이클당 한도 소모          # 사람이 직접 쓸 몫으로 남겨 두는 주간 한도

    "permission_flag": "--dangerously-skip-permissions",
}

SAFETY = (
    "너는 tomato-picker 자동운전 루프의 한 역할로 무인 실행 중이다. "
    "사람이 없으니 질문하지 말고, 확인을 기다리지 말고, 네가 판단해 끝까지 해라. "
    "금지: master 브랜치로 전환·푸시, force push, 태그 삭제, git reset --hard, "
    "저장소 밖 파일 삭제, autopilot/runner.py·board.py 수정(러너가 돌고 있다). "
    "작업판 상태는 반드시 `python autopilot/board.py`로만 바꿔라. "
    "마지막 메시지는 다음 사이클이 읽는 유일한 요약이다 — 20줄 안으로, "
    "'한 일 / 확인한 사실 / 다음 한 수 / 막힌 것' 네 줄기로 써라."
)


def log(msg: str) -> None:
    line = "[{}] {}".format(datetime.now().strftime("%m-%d %H:%M:%S"), msg)
    print(line, flush=True)
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(os.path.join(LOG_DIR, "runner.log"), "a", encoding="utf-8") as f:
        f.write(line + "\n")


def load_json(path, default):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def save_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


def cfg() -> dict:
    """기본값 + config.json. **엔진류 딕셔너리는 얕게 덮지 않고 병합한다** —

    안 그러면 config.json에 `engine_defs`를 살짝 손보다가 내장 gemini/agy 정의가
    통째로 사라진다(실제로 한 번 그랬다: 옛 `antigravity` 메모만 남기고 나머지가 지워짐).
    """
    c = dict(DEFAULTS)
    override = load_json(CONFIG, {})
    for key in ("models", "engines", "engine_defs", "engine_paths"):
        if key in override and isinstance(override[key], dict):
            merged = dict(c.get(key) or {})
            merged.update(override.pop(key))
            c[key] = merged
    c.update(override)
    return c


def state() -> dict:
    s = load_json(STATE, {})
    s.setdefault("started", datetime.now().isoformat(timespec="seconds"))
    s.setdefault("cycle", 0)
    s.setdefault("rot_idx", 0)
    s.setdefault("fails", 0)
    s.setdefault("cost_date", "")
    s.setdefault("cost_today", 0.0)
    s.setdefault("jetson_ip", None)
    s.setdefault("jetson_checked_cycle", -999)
    return s


def sh(args, timeout=60, cwd=ROOT):
    """작은 명령 하나. 표준출력을 문자열로."""
    try:
        p = subprocess.run(args, cwd=cwd, capture_output=True, timeout=timeout)
        out = (p.stdout or b"").decode("utf-8", "replace")
        err = (p.stderr or b"").decode("utf-8", "replace")
        return p.returncode, out.strip(), err.strip()
    except (subprocess.TimeoutExpired, OSError) as e:
        return 1, "", str(e)


# ---------------------------------------------------------------- 젯슨 찾기

def port_open(ip, port=22, timeout=0.6) -> bool:
    try:
        with socket.create_connection((ip, port), timeout):
            return True
    except OSError:
        return False


def ssh_ok(c, ip) -> bool:
    rc, out, _ = sh(["ssh", "-i", c["jetson_key"], "-o", "ConnectTimeout=5",
                     "-o", "StrictHostKeyChecking=no", "-o", "BatchMode=yes",
                     "{}@{}".format(c["jetson_user"], ip), "hostname"], timeout=15)
    return rc == 0 and bool(out)


def find_jetson(c, known=None):
    """후보 → 서브넷 훑기. IP가 DHCP로 바뀌어도 15일이 안 멈추게."""
    cands = []
    if known:
        cands.append(known)
    cands += [ip for ip in c["jetson_candidates"] if ip != known]
    for ip in cands:
        if port_open(ip) and ssh_ok(c, ip):
            return ip
    sub = c.get("jetson_subnet")
    if not sub:
        return None
    import concurrent.futures as cf
    hosts = ["{}.{}".format(sub, i) for i in range(2, 255)]
    with cf.ThreadPoolExecutor(max_workers=64) as ex:
        alive = [ip for ip, ok in zip(hosts, ex.map(port_open, hosts)) if ok]
    for ip in alive:
        if ip not in cands and ssh_ok(c, ip):
            return ip
    return None


# ---------------------------------------------------------------- 문맥 만들기

def tail_file(path, n_chars):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            t = f.read()
        return t[-n_chars:]
    except OSError:
        return ""


def recent_journal(n_chars=5000):
    try:
        files = sorted(f for f in os.listdir(JOURNAL_DIR) if f.endswith(".md"))
    except OSError:
        return "(아직 없음)"
    text = ""
    for f in reversed(files[-3:]):
        text = tail_file(os.path.join(JOURNAL_DIR, f), n_chars) + "\n" + text
        if len(text) > n_chars:
            break
    return text[-n_chars:] or "(아직 없음)"


def board(args):
    rc, out, err = sh([sys.executable, os.path.join(HERE, "board.py")] + args, timeout=30)
    return out or err


def build_prompt(c, s, role):
    charter = (tail_file(os.path.join(HERE, "roles", role + ".md"), 20000)
               + "\n\n---\n"
               + tail_file(os.path.join(HERE, "roles", "_공통.md"), 20000))
    objective = tail_file(os.path.join(HERE, "OBJECTIVE.md"), 6000)
    _, branch, _ = sh(["git", "rev-parse", "--abbrev-ref", "HEAD"])
    _, commits, _ = sh(["git", "log", "-4" if c.get("saver") else "-8", "--oneline"])
    _, status, _ = sh(["git", "status", "--short"])
    status = "\n".join(status.splitlines()[:40])
    started = datetime.fromisoformat(s["started"])
    day = (datetime.now() - started).days + 1

    jet = s.get("jetson_ip")
    if jet:
        hw = "젯슨 {} 접속 가능. 실기 조작 {}.".format(
            jet, "허용됨(다만 위험한 동작은 스스로 판단해 피해라)" if c["hardware_allowed"] else "금지")
        hw += "\n  ssh -i {} {}@{} '<명령>'".format(c["jetson_key"], c["jetson_user"], jet)
        hw += "\n  조작대 API: http://{}:8090/".format(jet)
    else:
        hw = ("젯슨에 **닿지 않는다**(전원이 꺼졌거나 망이 다르다). 실기가 필요한 일은 "
              "block으로 돌리고, 지금 PC에서 할 수 있는 일(코드·문서·자체검증)로 옮겨라.")

    # 사람의 말이 맨 위다. 자기 일보다 먼저 읽게 — 아래에 묻으면 안 읽은 것과 같다.
    say = []
    try:
        sys.path.insert(0, HERE)
        import say as say_mod
        say = say_mod.pending()
    except Exception:
        say = []
    head = ""
    if say:
        head = ("# ⚠ 사람이 남긴 말 (autopilot/INBOX.md) — 네 일보다 **먼저** 처리한다\n\n"
                + "\n".join("- " + x for x in say)
                + "\n\n처리 방법: 지시면 지금 따르고, 방향을 바꾸는 말이면 작업판에 반영한다"
                  "(`board.py add/prio/block`). 다 하고 나서 `autopilot/INBOX.md`에서 그 줄의"
                  " `- [ ]`를 `- [x]`로 바꾸고 같은 줄 끝에 ` → 처리: <무엇을 했는지>`를 붙여라."
                  " **처리하지 않은 줄은 체크하지 마라.** 내 판단과 어긋나면 사람 말이 이긴다;"
                  " 그래도 아니라고 보면 따르되 이유를 일지에 한 줄 남겨라.\n\n---\n\n")

    saver = bool(c.get("saver"))
    parts = [
        head,
        charter,
        "\n\n---\n# 지금 상황 (러너가 붙임)\n",
        "- 오늘: {} · 자동운전 {}일째 / {}일 · 사이클 {} · 역할 **{}**".format(
            datetime.now().strftime("%Y-%m-%d %H:%M"), day, c["days"], s["cycle"] + 1, role),
        "- 브랜치: {} (여기서만 작업한다)".format(branch),
        "- 하드웨어: " + hw,
        "\n## 최상위 목표 (autopilot/OBJECTIVE.md)\n" + objective,
        "\n## 작업판 — 열린 일 전부\n```\n" + board(["list", "--open", "--wide"]) + "\n```",
        "\n## 내 역할의 다음 일 (`board.py next --role {}`)\n```\n".format(role)
        + board(["next", "--role", role]) + "\n```",
        "\n## 최근 커밋\n```\n" + commits + "\n```",
        "\n## 작업 트리 상태\n```\n" + (status or "(깨끗함)") + "\n```",
        "\n## 최근 일지 (autopilot/journal/)\n" + recent_journal(),
        "\n---\n지금 한 사이클을 수행하라. 끝에 요약 네 줄기를 남겨라.",
    ]
    return "\n".join(parts)


# ---------------------------------------------------------------- 한 사이클

def claude_exe():
    for name in ("claude", "claude.cmd", "claude.exe"):
        p = shutil.which(name)
        if p:
            return p
    raise SystemExit("claude CLI를 찾을 수 없다")


def run_cycle(c, s, role):
    prompt = build_prompt(c, s, role)
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%m%d-%H%M%S")
    with open(os.path.join(LOG_DIR, "{}-{}.prompt.txt".format(stamp, role)),
              "w", encoding="utf-8") as f:
        f.write(prompt)

    engine = engine_for(c, role)
    spec = (c.get("engine_defs") or {}).get(engine) if engine != "claude" else None
    if engine != "claude":
        why = engine_blocked(c, engine, spec)
        if why:
            log("{} 못 씀({}) — 이번 사이클은 클로드로 돈다".format(engine, why))
            engine = "claude"
            spec = None
    if spec:
        model = c.get(engine + "_model") or spec.get("model") or ""
        # 다른 CLI들은 프롬프트를 인자로 받는다(stdin을 안 읽는다). 윈도우 명령줄 32KB
        # 한계가 있으므로 지시는 파일로 주고 인자로는 '읽어라'만 건넨다.
        pfile = os.path.join(LOG_DIR, "{}-{}.prompt.txt".format(stamp, role))
        ask = ("아래 파일이 이번 사이클의 전체 지시다. 먼저 그 파일을 읽고 그대로 수행하라: "
               + pfile.replace(os.sep, "/") + "   ||   " + SAFETY)
        subs = {"ask": ask, "pfile": pfile.replace(os.sep, "/"), "model": model,
                "role": role, "safety": SAFETY}
        cmd = [resolve_engine_exe(c, engine, spec)]
        cmd += [str(a).format(**subs) for a in spec.get("args", [])]
    else:
        model = pick_model(c, s, role)
        # stream-json으로 받는다 — 끝나야 아는 json과 달리 **도는 중에** 무엇을 하고 있는지
        # 보인다(monitor.py가 live.json을 읽는다). 한도 경고도 이 스트림에만 실려 온다.
        cmd = [claude_exe(), "-p",
               "--output-format", "stream-json", "--verbose",
               c["permission_flag"],
               "--model", model,
               "--append-system-prompt", SAFETY,
               "--name", "autopilot-" + role]
        if c.get("fallback_model") and c["fallback_model"] != model:
            cmd += ["--fallback-model", c["fallback_model"]]

    env = dict(os.environ)
    env["AUTOPILOT_ROLE"] = role
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    if spec and spec.get("api_key_file") and spec.get("api_key_env"):
        key = api_key_from_file(spec)
        if key:
            env[spec["api_key_env"]] = key   # 이 자식 프로세스에만 준다 — 시스템 환경은 안 건드린다

    res = {"role": role, "model": model, "engine": engine, "seconds": 0, "cost": 0.0,
           "ok": False, "text": "", "turns": 0, "limit_until": None, "rate": None}
    live = {"cycle": s["cycle"] + 1, "role": role, "model": model, "engine": engine,
            "started": datetime.now().isoformat(timespec="seconds"),
            "state": "running", "turns": 0, "cost": 0.0,
            "last_text": "", "tools": [], "rate": s.get("rate")}
    raw_path = os.path.join(LOG_DIR, "{}-{}.out.jsonl".format(stamp, role))
    t0 = time.time()
    last_write = [0.0]

    def flush_live(force=False):
        if force or time.time() - last_write[0] > 1.0:
            live["elapsed"] = round(time.time() - t0)
            save_json(LIVE, live)
            last_write[0] = time.time()

    def on_event(ev):
        t = ev.get("type")
        if spec:
            return on_event_generic(ev, res, live, flush_live)
        if t == "assistant":
            for blk in (ev.get("message") or {}).get("content") or []:
                if blk.get("type") == "text" and blk.get("text", "").strip():
                    live["last_text"] = blk["text"].strip()[-600:]
                elif blk.get("type") == "tool_use":
                    live["tools"].append({
                        "t": datetime.now().strftime("%H:%M:%S"),
                        "name": blk.get("name", "?"),
                        "brief": tool_brief(blk.get("name"), blk.get("input") or {}),
                    })
                    del live["tools"][:-14]
            live["turns"] += 1
        elif t == "rate_limit_event":
            info = ev.get("rate_limit_info") or {}
            if info and info.get("utilization") is not None:
                live["rate"] = res["rate"] = info
                # 창(5시간/7일)마다 따로 담는다 — 덮어쓰면 둘 중 하나가 사라진다.
                kind = info.get("rateLimitType") or "unknown"
                seen = res.setdefault("rate_by_type", {})
                slot = seen.setdefault(kind, {"first": None, "last": None})
                if slot["first"] is None:
                    slot["first"] = dict(info)
                slot["last"] = dict(info)
        elif "total_cost_usd" in ev:            # 마지막 결과 줄
            res["text"] = (ev.get("result") or "").strip()
            res["cost"] = float(ev.get("total_cost_usd") or 0.0)
            res["turns"] = int(ev.get("num_turns") or 0)
            res["ok"] = not ev.get("is_error")
            live["cost"] = res["cost"]
        flush_live()

    def pump(pipe, sink):
        with open(raw_path, "ab") as raw:
            for line in iter(pipe.readline, b""):
                raw.write(line)
                if sink is None:
                    continue
                try:
                    on_event(json.loads(line.decode("utf-8", "replace")))
                except (ValueError, KeyError, TypeError):
                    pass   # 스트림 한 줄이 깨져도 사이클을 죽이지 않는다

    p = subprocess.Popen(cmd, cwd=ROOT, env=env, stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    errbuf = []
    th_out = threading.Thread(target=pump, args=(p.stdout, on_event), daemon=True)
    th_err = threading.Thread(target=lambda: errbuf.append(p.stderr.read()), daemon=True)
    th_out.start()
    th_err.start()
    try:
        if not spec:
            p.stdin.write(prompt.encode("utf-8"))
        p.stdin.close()
    except OSError:
        pass

    try:
        p.wait(timeout=c["cycle_timeout_sec"])
    except subprocess.TimeoutExpired:
        # 자식(및 그 손자들)을 확실히 죽인다 — 남으면 다음 사이클이 포트·팔을 못 연다.
        if os.name == "nt":
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)], capture_output=True)
        else:
            p.kill()
        errbuf.append(b"\n[runner] cycle timeout")
    th_out.join(10)
    th_err.join(5)

    res["seconds"] = round(time.time() - t0)
    err_s = b"".join(x for x in errbuf if x).decode("utf-8", "replace")
    if p.returncode not in (0, None) and not res["text"]:
        res["ok"] = False
    if not res["text"]:
        res["text"] = (live["last_text"] or err_s[-1500:] or "(출력 없음)").strip()
    if not res["ok"]:
        res["limit_until"] = parse_limit(res["text"] + "\n" + err_s)
    if res["rate"]:
        s["rate"] = res["rate"]
        s["rate_seen"] = time.time()
    attribute_usage(s, role, engine, res)
    s["last_engine"] = engine
    live["state"] = "done" if res["ok"] else "failed"
    flush_live(force=True)
    return res


def attribute_usage(s, role, engine, res):
    """이 사이클이 한도를 얼마나 먹었나 — **창마다, 역할마다 따로 적는다.**

    사이클 **안에서** 오른 만큼이 우리 몫이고, 지난 사이클이 끝난 뒤부터 이번 사이클이
    시작될 때까지 오른 만큼은 **다른 것**(사람이 직접 쓴 대화 등)의 몫이다. 그래야
    "자동운전이 먹은 건가 내가 먹은 건가"에 숫자로 답할 수 있다.

    ⚠ 한도 신호는 매번 오지 않는다(임계에 가까울 때 실린다). 못 본 구간은 어느 쪽에도
      안 넣고 `unseen`으로 센다 — 모르는 것을 아는 척하면 이 표가 거짓말이 된다.
    """
    seen = res.get("rate_by_type") or {}
    if not seen:
        return
    book = s.setdefault("attrib", {})
    for kind, slot in seen.items():
        first, last = slot.get("first") or {}, slot.get("last") or {}
        if first.get("utilization") is None:
            continue
        reset = last.get("resetsAt") or first.get("resetsAt")
        b = book.get(kind)
        if not b or b.get("resetsAt") != reset:
            # 창이 새로 열렸다(리셋). 장부도 새로 연다.
            b = book[kind] = {"resetsAt": reset, "roles": {}, "other": 0.0,
                              "opened": datetime.now().isoformat(timespec="seconds"),
                              "start_u": float(first["utilization"]), "last_u": None}
        u0, u1 = float(first["utilization"]), float(last.get("utilization", first["utilization"]))
        if b.get("last_u") is not None:
            gap = u0 - float(b["last_u"])       # 사이클 사이에 오른 몫 = 내가 아닌 것
            if gap > 0:
                b["other"] = round(b.get("other", 0.0) + gap, 6)
        mine = max(0.0, u1 - u0)
        key = "{}:{}".format(role, engine)
        b["roles"][key] = round(b["roles"].get(key, 0.0) + mine, 6)
        b["last_u"] = u1
        b["last_seen"] = datetime.now().isoformat(timespec="seconds")
    s["rates"] = {k: (v.get("last") or v.get("first")) for k, v in seen.items()}


def binding_rate(s):
    """지금 우리를 묶고 있는 창 — 소진률이 가장 높은 것. (5시간이 먼저 찰 수도 있다)"""
    rs = [r for r in (s.get("rates") or {}).values() if r and r.get("utilization") is not None]
    if not rs:
        r = s.get("rate") or {}
        return r if r.get("utilization") is not None else None
    return max(rs, key=lambda r: r["utilization"])


def api_key_from_file(spec):
    """`api_key_file`에 키가 있으면 읽어 온다. 없거나 비었으면 None.

    시스템 환경변수를 안 건드린다 — 이 러너 프로세스에만 주입되고, 파일 하나를
    갈아 끼우는 것만으로 다음 사이클부터 반영된다(재로그인·재시작 불필요).
    """
    rel = spec.get("api_key_file")
    if not rel:
        return None
    path = rel if os.path.isabs(rel) else os.path.join(ROOT, rel)
    try:
        with open(path, encoding="utf-8") as f:
            key = f.read().strip()
        return key or None
    except OSError:
        return None


def resolve_engine_exe(c, engine, spec):
    """실행파일 경로 — PATH에 없으면 `engine_paths`의 절대경로를 본다.

    `agy`처럼 설치 직후 PATH가 아직 안 갱신된 경우가 있다(재로그인 전까지).
    절대경로를 알면 그걸로 바로 붙는다.
    """
    p = shutil.which(spec.get("exe", ""))
    if p:
        return p
    fixed = (c.get("engine_paths") or {}).get(engine)
    if fixed and os.path.exists(fixed):
        return fixed
    return spec.get("exe", engine)


def engine_blocked(c, engine, spec):
    """이 엔진으로 못 도는 이유(없으면 None) — **막히면 조용히 클로드로 돌아간다.**

    OAuth 로그인은 브라우저가 필요해 무인으로 못 한다. 그래서 자격이 없으면 사이클을
    실패시키지 않고 엔진만 바꾼다 — 사람이 로그인하는 순간부터, 또는 키 파일을
    채워 넣는 순간부터 저절로 쓰인다.
    """
    if not spec:
        return "engine_defs에 정의가 없다"
    exe = resolve_engine_exe(c, engine, spec)
    if not (shutil.which(exe) or os.path.exists(exe)):
        return "실행파일 " + str(spec.get("exe")) + " 없음(engine_paths도 확인)"
    env_keys = spec.get("ready_env") or []
    files = spec.get("ready_files") or []
    has_key_file = bool(spec.get("api_key_file"))
    if not env_keys and not files and not has_key_file:
        return None                       # 자격 조건을 안 적었으면 바로 쓴다
    if any(os.environ.get(k) for k in env_keys):
        return None
    if any(os.path.exists(os.path.expanduser(f)) for f in files):
        return None
    if has_key_file and api_key_from_file(spec):
        return None
    reason = "인증 없음(로그인 한 번 필요)"
    if has_key_file:
        reason = "키 없음 — {} 을 만들고 발급받은 키를 한 줄 넣어라".format(spec["api_key_file"])
    return reason


def on_event_generic(ev, res, live, flush):
    """다른 엔진의 stream-json 한 줄. 스키마가 클로드와 다르고 판마다 바뀌므로 **느슨하게** 읽는다.

    `event` 키가 있으면 agy(Antigravity CLI) 스키마로, 없으면 제미나이류 평평한
    스키마로 본다. 확실한 것 둘만 붙잡는다: 최종 응답과 오류. 나머지는 '무엇을
    하고 있나'를 보여 주는 데만 쓰고, 못 알아보면 조용히 흘린다.
    """
    if "event" in ev:
        return _on_event_agy(ev, res, live, flush)

    kind = ev.get("type") or ""
    if isinstance(ev.get("error"), dict):
        res["ok"] = False
        res["text"] = str(ev["error"].get("message", ""))[:1500]
    if isinstance(ev.get("response"), str) and ev["response"].strip():
        res["text"] = ev["response"].strip()
        res["ok"] = True
    st = ev.get("stats") or {}
    if st:
        live["stats"] = {k: v for k, v in list(st.items())[:6]}
    # 도구 호출로 보이는 것은 무엇이든 한 줄로 남긴다
    name = ev.get("name") or ev.get("tool") or (ev.get("toolCall") or {}).get("name")
    if name:
        args = ev.get("args") or (ev.get("toolCall") or {}).get("args") or {}
        live["tools"].append({"t": datetime.now().strftime("%H:%M:%S"),
                              "name": str(name)[:40],
                              "brief": tool_brief(name, args if isinstance(args, dict) else {})})
        del live["tools"][:-14]
        live["turns"] += 1
    elif kind in ("assistant", "content", "message") and isinstance(ev.get("text"), str):
        live["last_text"] = ev["text"].strip()[-600:]
    flush()


def _on_event_agy(ev, res, live, flush):
    """agy(Antigravity CLI) 스키마 — `{"event": "init"|"step_update"|"result", ...}`.

    구독 계정이라 `total_cost_usd`가 없다 — 토큰만 남기고 비용은 0으로 둔다
    (한도 소진은 이 엔진에선 아직 안 잰다. 클로드와 이중으로 도는 것 자체가
    한도 압박을 줄이는 목적이므로, 안 재는 것이 과대청구보다 안전한 쪽이다).
    """
    kind = ev.get("event")
    if kind == "result":
        r = ev.get("result") or {}
        res["text"] = str(r.get("response") or "").strip() or res["text"]
        res["ok"] = (r.get("status") == "SUCCESS")
        u = r.get("usage") or {}
        if u:
            live["stats"] = {k: v for k, v in list(u.items())[:6]}
    elif kind == "step_update":
        su = ev.get("step_update") or {}
        stype = su.get("step_type")
        if stype == "tool":
            info = su.get("tool_info") or {}
            if su.get("state") == "ACTIVE":
                live["tools"].append({
                    "t": datetime.now().strftime("%H:%M:%S"),
                    "name": str(info.get("name") or su.get("tool_name") or "?")[:40],
                    "brief": tool_brief(info.get("name"), info.get("parameters") or {}),
                })
                del live["tools"][:-14]
                live["turns"] += 1
            err = info.get("error")
            if err:
                live["last_text"] = ("[도구 오류] " + str(err.get("message", "")))[:600]
        elif stype == "agent_response":
            delta = su.get("text_delta")
            if isinstance(delta, str) and delta.strip():
                live["last_text"] = (live.get("last_text", "") + delta)[-600:]
    elif kind == "init":
        live["stats"] = {"tools_available": len((ev.get("init") or {}).get("tools") or [])}
    flush()


def tool_brief(name, inp) -> str:
    """도구 호출 한 줄 요약 — 모니터에서 '지금 무엇을 하고 있나'가 보이게.

    클로드는 소문자(`file_path`), agy는 PascalCase(`CommandLine`)를 쓰므로
    대소문자를 접어서 찾는다.
    """
    low = {str(k).lower(): v for k, v in (inp or {}).items()}
    for key in ("command", "commandline", "file_path", "filepath", "pattern",
               "path", "directorypath", "url", "prompt", "description"):
        v = low.get(key)
        if isinstance(v, str) and v.strip():
            v = " ".join(v.split())
            return v[:110] + ("…" if len(v) > 110 else "")
    return (name or "")[:110]


def engine_for(c, role) -> str:
    return (c.get("engines") or {}).get(role, "claude")


def pick_model(c, s, role):
    """역할별 모델. 단 **주간 한도가 빠듯하면 싼 쪽으로 내려간다.**

    한도는 돈이 아니라 시간이고, 같은 한도로 사이클을 더 많이 도는 쪽이 일을 더 한다.
    """
    model = (c.get("models") or {}).get(role) or c["model"]
    fb = c.get("fallback_model")
    if not (c.get("cheap_when_tight", True) and fb):
        return model
    r = binding_rate(s) or {}
    left = 1.0 - float(r.get("utilization") or 0.0)
    if r and left <= float(c.get("cheap_below_remaining", 0.10)):
        return fb
    return model


def parse_limit(text):
    """사용량 한도에 부딪혔나. 부딪혔으면 '언제 풀리는지'를 초로 돌려준다.

    구독은 돈이 아니라 시간이 벽이다. 한도는 기다리면 반드시 풀리므로, 이것을
    보통 실패로 세어 지수 백오프에 넣으면 안 된다(백오프가 창을 넘겨 더 놀게 된다).
    """
    import re
    low = text.lower()
    if "limit reached" not in low and "usage limit" not in low and "rate_limit" not in low:
        return None
    m = re.search(r"(?:limit reached|resets?)\D{0,20}(\d{10,13})", low)
    if m:
        v = int(m.group(1))
        return v / 1000.0 if v > 10 ** 11 else float(v)
    # 시각을 못 읽으면 5시간 창의 절반만 기다렸다 다시 두드린다(헛치는 비용은 프로세스 하나).
    return time.time() + 1800


def journal(c, s, role, res):
    os.makedirs(JOURNAL_DIR, exist_ok=True)
    path = os.path.join(JOURNAL_DIR, datetime.now().strftime("%Y-%m-%d") + ".md")
    new = not os.path.exists(path)
    text = res["text"]
    if len(text) > 4000:
        text = text[:4000] + "\n…(잘림, 전문은 autopilot/logs/)"
    with open(path, "a", encoding="utf-8") as f:
        if new:
            f.write("# 자동운전 일지 — {}\n".format(datetime.now().strftime("%Y-%m-%d")))
        f.write("\n## {} · {}({}) · 사이클 {} · {}분 · ${:.2f}{}\n\n{}\n".format(
            datetime.now().strftime("%H:%M"), role, res.get("model", "?"), s["cycle"] + 1,
            max(1, res["seconds"] // 60), res["cost"],
            "" if res["ok"] else " · **실패**", text))


def git_sync(c, role, s):
    rc, out, _ = sh(["git", "status", "--porcelain"])
    if rc != 0 or not out.strip():
        return
    sh(["git", "add", "-A"], timeout=120)
    msg = "자동운전 {} · 사이클 {} ({})\n\nCo-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>".format(
        role, s["cycle"] + 1, datetime.now().strftime("%m-%d %H:%M"))
    rc, out, err = sh(["git", "commit", "-m", msg], timeout=120)
    if rc != 0:
        log("commit 실패: " + (err or out)[:200])
        return
    if c["push"]:
        rc, _, err = sh(["git", "push", "origin", "HEAD"], timeout=180)
        if rc != 0:
            log("push 실패(다음에 다시 시도): " + err[:200])


def heartbeat(s, role, extra=""):
    save_json(HEARTBEAT, {
        "pid": os.getpid(), "time": datetime.now().isoformat(timespec="seconds"),
        "cycle": s["cycle"], "role": role, "note": extra,
    })


def guard_branch(c):
    rc, br, _ = sh(["git", "rev-parse", "--abbrev-ref", "HEAD"])
    if rc != 0:
        raise SystemExit("git 저장소가 아니다")
    if br == "master":
        raise SystemExit("master에서는 자동운전을 켜지 않는다. `git switch {}`".format(c["branch"]))
    return br


# ---------------------------------------------------------------- 루프

def loop(c, once_role=None):
    s = state()
    guard_branch(c)
    deadline = datetime.fromisoformat(s["started"]) + timedelta(days=c["days"])
    log("자동운전 시작 — 목표일 {} · 마감 {}".format(c["days"], deadline.strftime("%Y-%m-%d %H:%M")))

    while True:
        if os.path.exists(STOP):
            log("STOP 파일 — 끝낸다")
            return 0
        while os.path.exists(PAUSE):
            heartbeat(s, "paused")
            time.sleep(30)
        if once_role is None and datetime.now() > deadline:
            log("{}일 마감. 마지막 보고를 남긴다".format(c["days"]))
            s["rot_idx"] = 0
            res = run_cycle(c, s, "planner")
            journal(c, s, "planner", res)
            git_sync(c, "planner", s)
            return 0

        role = once_role or c["rotation"][s["rot_idx"] % len(c["rotation"])]

        # 하루 예산
        today = datetime.now().strftime("%Y-%m-%d")
        if s["cost_date"] != today:
            s["cost_date"], s["cost_today"] = today, 0.0
        cap = float(c.get("daily_cost_cap_usd") or 0)
        if cap and s["cost_today"] >= cap:
            log("오늘 예산 ${:.2f} 소진 — 자정까지 쉰다".format(s["cost_today"]))
            heartbeat(s, role, "budget")
            time.sleep(900)
            continue

        # 젯슨은 가끔만 다시 찾는다(서브넷 훑기가 느리다)
        if s["cycle"] - s["jetson_checked_cycle"] >= c["jetson_reprobe_cycles"] or not s["jetson_ip"]:
            ip = find_jetson(c, s.get("jetson_ip"))
            if ip != s.get("jetson_ip"):
                log("젯슨 IP: {} → {}".format(s.get("jetson_ip"), ip))
            s["jetson_ip"] = ip
            s["jetson_checked_cycle"] = s["cycle"]

        heartbeat(s, role, "running")
        log("사이클 {} · {} 시작".format(s["cycle"] + 1, role))
        t_start = time.time()
        res = run_cycle(c, s, role)
        log("사이클 {} · {}({}:{}) {} · {}초 · ${:.2f} · {}턴".format(
            s["cycle"] + 1, role, res.get("engine", "claude"), res["model"],
            "완료" if res["ok"] else "실패",
            res["seconds"], res["cost"], res["turns"]))

        if res["limit_until"]:
            wait = max(120, res["limit_until"] - time.time() + 120)
            log("사용량 한도 — {:.0f}분 뒤에 다시(실패로 세지 않는다)".format(wait / 60))
            heartbeat(s, role, "usage-limit")
            time.sleep(min(wait, 6 * 3600))
            continue  # 같은 역할로 다시. 한 일이 없으니 일지도 커밋도 남기지 않는다.

        journal(c, s, role, res)
        git_sync(c, role, s)

        s["cycle"] += 1
        s["rot_idx"] += 1
        s["cost_today"] += res["cost"]
        s["fails"] = 0 if res["ok"] else s["fails"] + 1
        s["last"] = {"role": role, "ok": res["ok"], "at": datetime.now().isoformat(timespec="seconds")}
        save_json(STATE, s)
        heartbeat(s, role, "idle")

        if once_role:
            return 0 if res["ok"] else 1

        if s["fails"] >= c["max_consecutive_fails"]:
            log("연속 {}회 실패 — 30분 쉬고 다시".format(s["fails"]))
            time.sleep(1800)
            s["fails"] = 0
            save_json(STATE, s)
        elif not res["ok"]:
            # 사용량 제한·과부하는 기다리면 풀린다. 4·8·16…분
            back = min(60 * 4 * (2 ** (s["fails"] - 1)), 3600)
            log("실패 — {}분 뒤 재개".format(back // 60))
            time.sleep(back)
        else:
            gap, why = pace(c, s, time.time() - t_start)
            log("다음 사이클까지 {:.0f}분 — {}".format(gap / 60, why))
            heartbeat(s, role, "idle")
            time.sleep(gap)


def pace(c, s, dur):
    """다음 사이클까지 얼마나 쉴까 — **주간 한도를 리셋 시각에 딱 맞춰 태우도록.**

    여기가 이 장치의 심장이다. 09-18에 하루 만에 주간 한도의 96%를 태웠다.
    그 속도면 남은 나흘을 통째로 논다 — '안 끊기는 것'이 목적인데 정확히 그게 깨진다.
    그래서 한 사이클이 한도를 얼마나 먹는지(du)를 **직접 재서**, 남은 예산을 남은
    시간에 고르게 편다. 사이클이 싸지면(sonnet) 저절로 촘촘해지고, 비싸지면 성겨진다.
    """
    base = max(c["sleep_between_sec"], c.get("min_cycle_interval_sec", 0) - dur)
    r = binding_rate(s) or {}
    u, reset = r.get("utilization"), r.get("resetsAt")
    if u is None or not reset:
        return base, "한도 정보 없음"

    prev = s.get("rate_prev_u")
    s["rate_prev_u"] = float(u)
    if prev is not None and u >= prev:
        du = float(u) - float(prev)
        # 한 번 튄 값에 끌려가지 않게 지수평활. 0은 무시한다(경고가 안 실린 사이클).
        if du > 0:
            s["du_ema"] = du if s.get("du_ema") is None else 0.5 * s["du_ema"] + 0.5 * du

    du = s.get("du_ema")
    guessed = False
    if not du:
        # 아직 안 재 봤다. 여기서 base(=1분)로 돌려보내면 남은 예산을 몇 분 만에 태운다.
        # 모를 때는 **비싼 쪽으로 가정**한다 — 첫 사이클이 끝나면 실측값이 이걸 밀어낸다.
        du = float(c.get("du_guess", 0.01))
        guessed = True
    left_u = max(0.0, 1.0 - float(c.get("rate_reserve", 0.02)) - float(u))
    left_t = float(reset) - time.time()
    if left_t <= 0:
        return base, "리셋 직전"

    cycles = left_u / du
    if cycles < 0.5:
        # 예산이 없다. 리셋까지 자는 게 맞다 — 두드려 봐야 거절만 쌓인다.
        return min(left_t + 120, 6 * 3600), "예산 소진(u={:.1%}) — 리셋까지 대기".format(u)
    gap = max(base, left_t / cycles - dur)
    return gap, "u={:.1%} · 사이클당 {:.2%}{} · 남은 {:.0f}회를 {:.0f}h에 폄".format(
        u, du, "(가정)" if guessed else "", cycles, left_t / 3600)


def cmd_status(c):
    s = state()
    hb = load_json(HEARTBEAT, {})
    started = datetime.fromisoformat(s["started"])
    print("시작 {} · {}일째/{} · 사이클 {} · 오늘 ${:.2f} · 연속실패 {}".format(
        started.strftime("%m-%d %H:%M"), (datetime.now() - started).days + 1,
        c["days"], s["cycle"], s["cost_today"], s["fails"]))
    print("젯슨: {} · 심장박동: {} ({})".format(
        s.get("jetson_ip") or "없음", hb.get("time", "-"), hb.get("note", "-")))
    r = s.get("rate") or {}
    if r.get("utilization") is not None:
        left = 1.0 - r["utilization"]
        reset = datetime.fromtimestamp(r["resetsAt"]) if r.get("resetsAt") else None
        du = s.get("du_ema")
        print("주간한도: {:.1%} 소진 · 남은 {:.1%}{} · 리셋 {}".format(
            r["utilization"], left,
            " ≈ {:.0f}사이클".format(left / du) if du else "",
            reset.strftime("%m-%d %H:%M") if reset else "?"))
    print("작업판: " + board(["stats"]))
    sys.path.insert(0, HERE)
    try:
        import say as say_mod
        p = say_mod.pending()
    except Exception:
        p = []
    print("안 읽힌 내 말: " + (str(len(p)) + "건 — " + p[0][:50] if p else "없음"))
    print(board(["list", "--open"]))


def main():
    ap = argparse.ArgumentParser(description="tomato-picker 자동운전 감독자")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("run")
    p = sub.add_parser("once")
    p.add_argument("--role", required=True, choices=["planner", "builder", "auditor", "tester", "metrologist"])
    sub.add_parser("status")
    sub.add_parser("probe")
    a = ap.parse_args()
    c = cfg()
    if a.cmd == "status":
        cmd_status(c)
        return 0
    if a.cmd == "probe":
        s = state()
        ip = find_jetson(c, s.get("jetson_ip"))
        s["jetson_ip"] = ip
        save_json(STATE, s)
        print("젯슨: " + (ip or "못 찾음"))
        return 0
    if a.cmd == "once":
        return loop(c, once_role=a.role)
    return loop(c)


if __name__ == "__main__":
    sys.exit(main())

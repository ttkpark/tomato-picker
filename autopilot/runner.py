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
    c = dict(DEFAULTS)
    c.update(load_json(CONFIG, {}))
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
    _, commits, _ = sh(["git", "log", "-8", "--oneline"])
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

    parts = [
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

    model = (c.get("models") or {}).get(role) or c["model"]
    cmd = [claude_exe(), "-p",
           "--output-format", "json",
           c["permission_flag"],
           "--model", model,
           "--append-system-prompt", SAFETY,
           "--name", "autopilot-" + role]
    if c.get("fallback_model"):
        cmd += ["--fallback-model", c["fallback_model"]]

    env = dict(os.environ)
    env["AUTOPILOT_ROLE"] = role
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"

    t0 = time.time()
    p = subprocess.Popen(cmd, cwd=ROOT, env=env, stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        out, err = p.communicate(prompt.encode("utf-8"), timeout=c["cycle_timeout_sec"])
    except subprocess.TimeoutExpired:
        # 자식(및 그 손자들)을 확실히 죽인다 — 남으면 다음 사이클이 포트·팔을 못 연다.
        if os.name == "nt":
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)], capture_output=True)
        else:
            p.kill()
        try:
            out, err = p.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            out, err = b"", b""
        err = (err or b"") + b"\n[runner] cycle timeout"
    dt = time.time() - t0

    out_s = out.decode("utf-8", "replace")
    err_s = err.decode("utf-8", "replace")
    with open(os.path.join(LOG_DIR, "{}-{}.out.json".format(stamp, role)),
              "w", encoding="utf-8") as f:
        f.write(out_s + ("\n--- stderr ---\n" + err_s if err_s else ""))

    res = {"role": role, "model": model, "seconds": round(dt), "cost": 0.0,
           "ok": False, "text": "", "turns": 0, "limit_until": None}
    try:
        j = json.loads(out_s)
        res["text"] = (j.get("result") or "").strip()
        res["cost"] = float(j.get("total_cost_usd") or 0.0)
        res["turns"] = int(j.get("num_turns") or 0)
        res["ok"] = not j.get("is_error") and p.returncode == 0
    except ValueError:
        res["text"] = (out_s[-1500:] or err_s[-1500:] or "(출력 없음)").strip()
        res["ok"] = False
    if not res["ok"]:
        res["limit_until"] = parse_limit(res["text"] + "\n" + err_s)
    return res


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
        log("사이클 {} · {}({}) {} · {}초 · ${:.2f} · {}턴".format(
            s["cycle"] + 1, role, res["model"], "완료" if res["ok"] else "실패",
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
            # 속도 조절 — 사이클 **시작**을 기준으로 간격을 맞춘다. 15일을 버티는 것이
            # 하루에 몰아치는 것보다 낫다(한도를 일찍 태우면 그 뒤가 통째로 빈다).
            gap = max(c["sleep_between_sec"],
                      c["min_cycle_interval_sec"] - (time.time() - t_start))
            log("다음 사이클까지 {:.0f}분".format(gap / 60))
            heartbeat(s, role, "idle")
            time.sleep(gap)


def cmd_status(c):
    s = state()
    hb = load_json(HEARTBEAT, {})
    started = datetime.fromisoformat(s["started"])
    print("시작 {} · {}일째/{} · 사이클 {} · 오늘 ${:.2f} · 연속실패 {}".format(
        started.strftime("%m-%d %H:%M"), (datetime.now() - started).days + 1,
        c["days"], s["cycle"], s["cost_today"], s["fails"]))
    print("젯슨: {} · 심장박동: {} ({})".format(
        s.get("jetson_ip") or "없음", hb.get("time", "-"), hb.get("note", "-")))
    print("작업판: " + board(["stats"]))
    print(board(["list", "--open"]))


def main():
    ap = argparse.ArgumentParser(description="tomato-picker 자동운전 감독자")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("run")
    p = sub.add_parser("once")
    p.add_argument("--role", required=True, choices=["planner", "builder", "auditor", "tester"])
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

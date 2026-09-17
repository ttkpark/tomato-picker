#!/usr/bin/env python3
"""작업판 — 네 역할이 공유하는 단 하나의 상태.

에이전트는 이 CLI만 쓴다(JSON을 손으로 고치지 않는다 — 깨지면 15일이 멈춘다).

  python autopilot/board.py list [--status todo] [--role builder] [--json]
  python autopilot/board.py add --title "..." --role builder --why "..." --done-when "..." [-p 2]
  python autopilot/board.py next --role builder [--claim]
  python autopilot/board.py claim T7
  python autopilot/board.py note T7 "어디까지 했는지"
  python autopilot/board.py done T7 --result "무엇이 실제로 되는가"
  python autopilot/board.py block T7 --reason "무엇이 없어서 못 하는가"
  python autopilot/board.py reopen T7
  python autopilot/board.py prio T7 1
  python autopilot/board.py stats
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone

# 윈도우 콘솔은 기본이 cp949라 한글 작업판이 깨진다. 읽는 쪽이 에이전트든 사람이든 UTF-8로 낸다.
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

HERE = os.path.dirname(os.path.abspath(__file__))
BOARD = os.path.join(HERE, "state", "board.json")
LOCK = BOARD + ".lock"

ROLES = ("planner", "builder", "auditor", "tester", "metrologist")
STATUSES = ("todo", "doing", "blocked", "done")
MAX_NOTES = 12          # 오래된 메모는 잘라낸다(파일이 무한히 자라면 매 사이클 문맥을 먹는다)
MAX_DONE = 60           # 끝난 일도 같은 이유로 최근 것만 남긴다


def now() -> str:
    return datetime.now(timezone.utc).astimezone().strftime("%Y-%m-%d %H:%M")


class Lock:
    """파일 락 — 러너가 역할을 순차로 돌리지만, 사람이 끼어들어도 안 깨지게."""

    def __init__(self, path, timeout=30.0):
        self.path = path
        self.timeout = timeout
        self.fd = None

    def __enter__(self):
        deadline = time.time() + self.timeout
        while True:
            try:
                self.fd = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_RDWR)
                os.write(self.fd, str(os.getpid()).encode())
                return self
            except FileExistsError:
                # 죽은 프로세스가 남긴 락은 60초 뒤 무시한다.
                try:
                    if time.time() - os.path.getmtime(self.path) > 60:
                        os.unlink(self.path)
                        continue
                except OSError:
                    pass
                if time.time() > deadline:
                    raise SystemExit("board lock timeout: " + self.path)
                time.sleep(0.2)

    def __exit__(self, *exc):
        if self.fd is not None:
            os.close(self.fd)
        try:
            os.unlink(self.path)
        except OSError:
            pass


def load() -> dict:
    if not os.path.exists(BOARD):
        return {"version": 1, "next_id": 1, "tasks": []}
    with open(BOARD, encoding="utf-8") as f:
        return json.load(f)


def save(b: dict) -> None:
    os.makedirs(os.path.dirname(BOARD), exist_ok=True)
    tmp = BOARD + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(b, f, ensure_ascii=False, indent=1)
    os.replace(tmp, BOARD)


def find(b: dict, tid: str) -> dict:
    tid = tid.upper()
    if not tid.startswith("T"):
        tid = "T" + tid
    for t in b["tasks"]:
        if t["id"] == tid:
            return t
    raise SystemExit("없는 작업: " + tid)


def trim(b: dict) -> None:
    for t in b["tasks"]:
        if len(t.get("notes", [])) > MAX_NOTES:
            t["notes"] = t["notes"][-MAX_NOTES:]
    done = [t for t in b["tasks"] if t["status"] == "done"]
    if len(done) > MAX_DONE:
        drop = {id(t) for t in done[: len(done) - MAX_DONE]}
        b["tasks"] = [t for t in b["tasks"] if id(t) not in drop]


def fmt(t: dict, wide: bool = False) -> str:
    mark = {"todo": " ", "doing": ">", "blocked": "!", "done": "x"}[t["status"]]
    line = "[{}] {} p{} {:<7} {}".format(mark, t["id"], t["priority"], t["role"], t["title"])
    if wide:
        if t.get("done_when"):
            line += "\n      완료조건: " + t["done_when"]
        for n in t.get("notes", [])[-3:]:
            line += "\n      · {} {}: {}".format(n["t"], n["by"], n["text"])
    return line


def main() -> int:
    ap = argparse.ArgumentParser(prog="board.py", description="자동운전 작업판")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("list")
    p.add_argument("--status", choices=STATUSES)
    p.add_argument("--role", choices=ROLES)
    p.add_argument("--open", action="store_true", help="todo+doing+blocked만")
    p.add_argument("--wide", action="store_true")
    p.add_argument("--json", action="store_true")
    p.add_argument("--limit", type=int, default=40)

    p = sub.add_parser("add")
    p.add_argument("--title", required=True)
    p.add_argument("--role", required=True, choices=ROLES)
    p.add_argument("--why", default="")
    p.add_argument("--done-when", default="", dest="done_when")
    p.add_argument("-p", "--priority", type=int, default=3, help="1=급함 … 5=나중")
    p.add_argument("--by", default=os.environ.get("AUTOPILOT_ROLE", "human"))

    p = sub.add_parser("next")
    p.add_argument("--role", required=True, choices=ROLES)
    p.add_argument("--claim", action="store_true")

    for name in ("claim", "reopen"):
        p = sub.add_parser(name)
        p.add_argument("id")

    p = sub.add_parser("note")
    p.add_argument("id")
    p.add_argument("text")
    p.add_argument("--by", default=os.environ.get("AUTOPILOT_ROLE", "human"))

    p = sub.add_parser("done")
    p.add_argument("id")
    p.add_argument("--result", required=True, help="무엇이 실제로 되는가(검증 방법 포함)")
    p.add_argument("--by", default=os.environ.get("AUTOPILOT_ROLE", "human"))

    p = sub.add_parser("block")
    p.add_argument("id")
    p.add_argument("--reason", required=True)
    p.add_argument("--by", default=os.environ.get("AUTOPILOT_ROLE", "human"))

    p = sub.add_parser("prio")
    p.add_argument("id")
    p.add_argument("priority", type=int)

    sub.add_parser("stats")

    a = ap.parse_args()

    if a.cmd == "list":
        b = load()
        ts = b["tasks"]
        if a.open:
            ts = [t for t in ts if t["status"] != "done"]
        if a.status:
            ts = [t for t in ts if t["status"] == a.status]
        if a.role:
            ts = [t for t in ts if t["role"] == a.role]
        ts = sorted(ts, key=lambda t: (t["status"] == "done", t["priority"], t["id"]))[: a.limit]
        if a.json:
            print(json.dumps(ts, ensure_ascii=False, indent=1))
        elif not ts:
            print("(없음)")
        else:
            for t in ts:
                print(fmt(t, a.wide))
        return 0

    with Lock(LOCK):
        b = load()
        if a.cmd == "add":
            tid = "T{}".format(b["next_id"])
            b["next_id"] += 1
            t = {
                "id": tid, "title": a.title, "role": a.role, "status": "todo",
                "priority": max(1, min(5, a.priority)), "why": a.why,
                "done_when": a.done_when, "attempts": 0,
                "created": now(), "updated": now(),
                "notes": [{"t": now(), "by": a.by, "text": "만듦"}],
            }
            b["tasks"].append(t)
            print(tid)
        elif a.cmd == "next":
            cands = [t for t in b["tasks"] if t["role"] == a.role and t["status"] in ("todo", "doing")]
            # 이미 잡아 둔 일이 먼저다 — 반쯤 한 일을 버리고 새 일을 시작하면 15일이 파편이 된다.
            cands.sort(key=lambda t: (t["status"] != "doing", t["priority"], t["id"]))
            if not cands:
                print("(없음)")
                return 0
            t = cands[0]
            if a.claim:
                t["status"] = "doing"
                t["attempts"] = t.get("attempts", 0) + 1
                t["updated"] = now()
            print(json.dumps(t, ensure_ascii=False, indent=1))
        elif a.cmd == "claim":
            t = find(b, a.id)
            t["status"] = "doing"
            t["attempts"] = t.get("attempts", 0) + 1
            t["updated"] = now()
            print(fmt(t))
        elif a.cmd == "note":
            t = find(b, a.id)
            t.setdefault("notes", []).append({"t": now(), "by": a.by, "text": a.text})
            t["updated"] = now()
            print(fmt(t))
        elif a.cmd == "done":
            t = find(b, a.id)
            t["status"] = "done"
            t["updated"] = now()
            t.setdefault("notes", []).append({"t": now(), "by": a.by, "text": "완료: " + a.result})
            print(fmt(t))
        elif a.cmd == "block":
            t = find(b, a.id)
            t["status"] = "blocked"
            t["updated"] = now()
            t.setdefault("notes", []).append({"t": now(), "by": a.by, "text": "막힘: " + a.reason})
            print(fmt(t))
        elif a.cmd == "reopen":
            t = find(b, a.id)
            t["status"] = "todo"
            t["updated"] = now()
            print(fmt(t))
        elif a.cmd == "prio":
            t = find(b, a.id)
            t["priority"] = max(1, min(5, a.priority))
            t["updated"] = now()
            print(fmt(t))
        elif a.cmd == "stats":
            c = {s: 0 for s in STATUSES}
            for t in b["tasks"]:
                c[t["status"]] = c.get(t["status"], 0) + 1
            print(" ".join("{}={}".format(k, v) for k, v in c.items()))
        trim(b)
        save(b)
    return 0


if __name__ == "__main__":
    sys.exit(main())

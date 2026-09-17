#!/usr/bin/env python3
"""자동운전에게 말 걸기 — 한 줄을 INBOX에 넣는다.

  python autopilot/say.py "평행이동부터 잡아라"
  python autopilot/say.py --now "젯슨 지금 내가 쓴다. 실기 만지지 마"   # 도는 사이클을 끊고 즉시
  python autopilot/say.py --list                                        # 아직 안 읽힌 말

다음 사이클이 프롬프트 맨 위에서 읽는다. `--now`는 지금 도는 사이클을 끊으므로
급할 때만(그 사이클이 하던 일은 작업판 메모까지만 남고 버려진다).
"""
import argparse
import os
import subprocess
import sys
from datetime import datetime

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

HERE = os.path.dirname(os.path.abspath(__file__))
INBOX = os.path.join(HERE, "INBOX.md")


def pending(text=None):
    """아직 `- [ ]`인 줄들. 러너도 이 함수를 쓴다."""
    if text is None:
        try:
            with open(INBOX, encoding="utf-8") as f:
                text = f.read()
        except OSError:
            return []
    out = []
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("- [ ]"):
            out.append(s[5:].strip())
    return out


def main():
    ap = argparse.ArgumentParser(description="자동운전에게 말 걸기")
    ap.add_argument("message", nargs="*", help="하고 싶은 말")
    ap.add_argument("--now", action="store_true", help="도는 사이클을 끊고 즉시 읽히게")
    ap.add_argument("--list", action="store_true", dest="show", help="아직 안 읽힌 말")
    a = ap.parse_args()

    if a.show:
        p = pending()
        print("\n".join("- " + x for x in p) if p else "(안 읽힌 말 없음)")
        return 0

    msg = " ".join(a.message).strip()
    if not msg:
        ap.error("할 말을 적어라")

    line = "- [ ] {} · {}".format(datetime.now().strftime("%Y-%m-%d %H:%M"), msg)
    with open(INBOX, "a", encoding="utf-8") as f:
        f.write("\n" + line + "\n")
    print("넣었다: " + msg)

    if a.now:
        # 러너가 아니라 `claude` 자식만 죽인다 — 러너는 그것을 실패로 보고 다음 사이클을
        # 곧바로 시작하며, 그 프롬프트 맨 위에 이 말이 붙는다.
        killed = 0
        if os.name == "nt":
            r = subprocess.run(["wmic", "process", "where",
                                "name='claude.exe'", "get", "ProcessId"],
                               capture_output=True, text=True)
            for tok in r.stdout.split():
                if tok.isdigit():
                    subprocess.run(["taskkill", "/T", "/F", "/PID", tok], capture_output=True)
                    killed += 1
        else:
            killed = subprocess.run(["pkill", "-f", "claude -p"]).returncode == 0
        print("도는 사이클 끊음({}개). 다음 사이클이 1~2분 안에 이 말부터 읽는다.".format(killed))
    else:
        print("다음 사이클이 읽는다. 지금 도는 것을 끊으려면 --now")
    return 0


if __name__ == "__main__":
    sys.exit(main())

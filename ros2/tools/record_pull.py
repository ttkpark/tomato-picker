#!/usr/bin/env python3
"""젯슨에 남은 시험기록 줄을 저장소로 **가져온다** — 덮지 않고 붙인다.

왜 이 도구가 있는가 (2026-09-18, T58):
  실기는 젯슨의 도커 안에서 돈다. `move5_check`가 쓰는 자리는 컨테이너에
  물린 **젯슨 트리**(`~/tomato-picker/`)이고, 그 트리는 `git` 저장소가 아니다
  (archive+scp로 배포된 부분복사본이다 — 실측: `git rev-parse` → "not a git
  repository"). 그래서 거기 쓴 줄은 **커밋될 길이 없다.** 사이클42의 실기 5회가
  산문에만 남고 jsonl에 없던 것이 그 결과다.

  ⚠ 그냥 `scp`로 끌어오면 한쪽을 잃는다. 같은 이름의 파일이 두 곳에서 **따로**
  자란다 — 2026-09-18 실측으로 저장소 57줄 / 젯슨 20줄이고, 겹치는 줄은 사람이
  손으로 옮긴 5줄뿐이다. 그러니 이 도구는 **덮어쓰기를 하지 않는다**:
  저장소에 없는 줄만 골라 **끝에 붙인다**(append-only). 순서는 젯슨의 순서다.

쓰는 법 (PC에서):
    python ros2/tools/record_pull.py                       # 오늘 날짜 파일
    python ros2/tools/record_pull.py --date 2026-09-18 --dry-run
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
RECORD_REL = os.path.join("docs", "시험기록")

DEFAULT_HOST = "server@192.168.0.19"
DEFAULT_KEY = "C:/Users/parkg/.ssh/id_ed25519"
DEFAULT_REMOTE = "~/tomato-picker/" + RECORD_REL.replace(os.sep, "/")


# 한 줄의 신원을 이루는 칸. **사람이 나중에 덧붙인 칸(cycle, note 따위)은 뺀다** —
# 2026-09-18 실측: 사이클35의 10줄이 저장소에는 손으로 `cycle: 35`가 붙은 채,
# 젯슨에는 원본 그대로 남아 있었다. 글자 해시로만 보면 **같은 시험이 다른 줄로
# 보여** 10줄이 겹쳐 붙는다. `detail`을 넣는 이유는 그 반대쪽 위험 때문이다 —
# 같은 표적을 두 판 돌리면 앞칸이 전부 같아지는데, 거절 문장까지 같기는 어렵다.
IDENTITY = ("trial", "dry_run", "commanded", "ok", "stage", "error_mm", "detail", "t")


def _key(line: str) -> str:
    """한 줄의 신원. 같은 시험이면 같은 값이 나와야 한다.

    jsonl 한 줄은 그 자체로 한 번의 시험이다. 시험 줄이면 위의 칸만 보고,
    시험 줄이 아니면(note 따위) 글자 그대로 본다.
    """
    try:
        row = json.loads(line)
    except Exception:
        row = None
    if isinstance(row, dict) and "trial" in row:
        body = json.dumps({k: row.get(k) for k in IDENTITY},
                          ensure_ascii=False, sort_keys=True)
    else:
        body = line.strip()
    return hashlib.sha1(body.encode("utf-8")).hexdigest()


def merge_lines(local: list[str], remote: list[str]) -> tuple[list[str], list[str]]:
    """저장소에 없는 원격 줄만 골라 낸다. (붙일 줄, 이미 있던 줄)

    덮어쓰지 않는 것이 이 함수의 전부다 — 로컬 줄은 하나도 건드리지 않는다.
    """
    have = {_key(ln) for ln in local if ln.strip()}
    fresh, dup = [], []
    for ln in remote:
        if not ln.strip():
            continue
        k = _key(ln)
        if k in have:
            dup.append(ln.rstrip())
            continue
        have.add(k)  # 원격 안에 같은 줄이 두 번 있어도 한 번만 붙인다
        fresh.append(ln.rstrip())
    return fresh, dup


def _describe(line: str) -> str:
    """붙이는 줄이 무엇인지 한 토막으로 — 사람이 눈으로 확인하라고."""
    try:
        row = json.loads(line)
    except Exception:
        return line[:60]
    if "note" in row and "trial" not in row:
        return f"note: {str(row['note'])[:50]}"
    bits = [f"trial={row.get('trial')}", f"ok={row.get('ok')}"]
    if row.get("error_mm") is not None:
        bits.append(f"err={row['error_mm']}mm")
    if row.get("stage"):
        bits.append(f"stage={row['stage']}")
    return " ".join(bits)


def main() -> int:
    today = time.strftime("%Y-%m-%d")
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--key", default=DEFAULT_KEY)
    ap.add_argument("--remote-dir", default=DEFAULT_REMOTE)
    ap.add_argument("--date", default=today)
    ap.add_argument("--name", default="", help="파일 이름을 통째로 지정(--date 무시)")
    ap.add_argument("--dry-run", action="store_true", help="붙일 줄만 보여준다")
    args = ap.parse_args()

    name = args.name or f"move-to-point-{args.date}.jsonl"
    remote_path = f"{args.remote_dir}/{name}"
    local_path = os.path.join(REPO, RECORD_REL, name)

    cmd = ["ssh"]
    if args.key:
        cmd += ["-i", args.key]
    cmd += [args.host, f"cat {remote_path} 2>/dev/null || true"]
    proc = subprocess.run(cmd, capture_output=True)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr.decode("utf-8", "replace"))
        print(f"❌ 젯슨에서 못 읽었다: {remote_path}")
        return 1
    remote_lines = proc.stdout.decode("utf-8", "replace").splitlines()
    if not remote_lines:
        print(f"젯슨에 그 기록이 없다(또는 비었다): {remote_path}")
        return 0

    local_lines = []
    if os.path.exists(local_path):
        local_lines = open(local_path, encoding="utf-8").read().splitlines()

    fresh, dup = merge_lines(local_lines, remote_lines)
    print(f"젯슨 {len(remote_lines)}줄 · 저장소 {len(local_lines)}줄 "
          f"→ 새 줄 {len(fresh)} (이미 있던 줄 {len(dup)})")
    for ln in fresh:
        print(f"  + {_describe(ln)}")
    if not fresh:
        return 0
    if args.dry_run:
        print("(--dry-run — 아무것도 안 붙였다)")
        return 0

    os.makedirs(os.path.dirname(local_path), exist_ok=True)
    with open(local_path, "a", encoding="utf-8") as fh:
        for ln in fresh:
            fh.write(ln + "\n")
    print(f"✅ {len(fresh)}줄을 붙였다 → {local_path}")
    print("   (러너가 사이클 끝에 커밋한다)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

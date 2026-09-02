#!/usr/bin/env python3
"""대시보드 페이지의 **JS가 살아 있는지** 검사 — 브라우저 없이, node 없이.

    python tools/pages_check.py

왜 이게 필요한가 (2026-08-28 실사고):

    pages.py의 JS는 파이썬 `\"\"\"..."\"\"` 문자열 안에 들어 있다. 거기에
    `'...등록합니다.\\n'` 처럼 **백슬래시 하나짜리 \\n**을 쓰면, 파이썬이
    그걸 **진짜 줄바꿈으로 바꿔서** 내보낸다. 그러면 JS 쪽에서는

        confirm('지금 자세를 ... 등록합니다.
        '            ← 여기서 문자열이 끊긴다

    가 되어 **스크립트 전체가 파싱 실패**한다. 결과는 조용하다: 페이지는
    멀쩡히 뜨고, 버튼만 아무 반응이 없고, 상태 표시는 "확인 중..."에 멈춘다.
    실제로 `/settings`가 이 상태로 방치돼 **팔 영점을 한 번도 못 잡고 있었다**
    (그래서 그 위에 얹힌 손-눈 보정도 시작조차 못 했다).

    한 글자 때문에 화면 하나가 통째로 죽는데 아무도 모른다 — 그래서 검사한다.

무엇을 보나:
  ① pages.py 소스에 **해석되는 이스케이프**(\\n \\t \\r …)가 JS 안에 있는가
     → JS로 보내려면 `\\\\n`으로 두 개를 써야 한다
  ② 내보낸 <script>의 각 줄에서 따옴표 문자열이 **줄을 넘어가지 않는가**
     (백틱 템플릿 리터럴은 넘어가도 되므로 상태를 이어서 추적한다)
  ③ 괄호·중괄호·대괄호 짝이 맞는가
  ④ node가 있으면 **진짜로 파싱**까지 한다 (없으면 건너뛴다 — 젯슨엔 없다)
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import tempfile

for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8")

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from tomato_picker.voice import pages  # noqa: E402

BS = chr(92)
FAILED: list[str] = []
PASSED = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global PASSED
    if ok:
        PASSED += 1
        print(f"  ok   {name}" + (f"  ({detail})" if detail else ""))
    else:
        FAILED.append(name)
        print(f"  FAIL {name}  {detail}")


def scripts_of(html: str) -> list[str]:
    return re.findall(r"<script[^>]*>([\s\S]*?)</script>", html)


# ----------------------------------------------------------------------
# ① 소스의 이스케이프 — 이번 사고의 근본 원인
# ----------------------------------------------------------------------

def test_source_escapes() -> None:
    print("\n[소스] JS 안에 해석되는 이스케이프가 있는가")
    path = os.path.join(os.path.dirname(pages.__file__), "pages.py")
    src = open(path, encoding="utf-8").read()
    # 앞에 백슬래시가 더 있으면(\\n) 이미 안전하다.
    pat = re.compile("(?<![" + BS + BS + "])" + BS + BS + "([ntrbfav0])")

    bad = []
    for i, line in enumerate(src.split("\n"), 1):
        stripped = line.lstrip()
        # 파이썬이 HTML을 조립하는 줄(f"...\n")은 정상이다 — 그건 진짜 줄바꿈을
        # 원하는 곳이다. JS는 들여쓰기된 본문 안에 있으므로 그쪽만 본다.
        if stripped.startswith(("return (", 'f"', '"<', "boot =", '"<meta')):
            continue
        for m in pat.finditer(line):
            bad.append(f"{i}행 {BS}{m.group(1)} — {stripped[:70]}")

    check("JS로 나갈 이스케이프는 전부 두 번 써야 한다",
          not bad,
          "; ".join(bad) if bad else f"{BS}{BS}n 처럼 쓰면 JS가 {BS}n을 받는다")


# ----------------------------------------------------------------------
# ② 줄을 넘어가는 따옴표 문자열
# ----------------------------------------------------------------------

def unterminated_lines(js: str) -> list[tuple[int, str]]:
    """따옴표 문자열이 줄 끝에서 안 닫힌 줄을 찾는다.

    백틱(`)은 여러 줄이 정상이므로 줄을 넘어 상태를 이어간다. // 주석과
    /* */ 주석은 건너뛴다.
    """
    bad: list[tuple[int, str]] = []
    in_back = False
    in_block = False
    for n, line in enumerate(js.split("\n"), 1):
        i = 0
        quote = ""          # 이 줄에서 열려 있는 ' 또는 "
        while i < len(line):
            c = line[i]
            if in_block:
                if line.startswith("*/", i):
                    in_block = False
                    i += 2
                    continue
                i += 1
                continue
            if quote:
                if c == BS:
                    i += 2
                    continue
                if c == quote:
                    quote = ""
                i += 1
                continue
            if in_back:
                if c == BS:
                    i += 2
                    continue
                if c == "`":
                    in_back = False
                i += 1
                continue
            if line.startswith("//", i):
                break
            if line.startswith("/*", i):
                in_block = True
                i += 2
                continue
            if c in "'\"":
                quote = c
            elif c == "`":
                in_back = True
            i += 1
        if quote:
            bad.append((n, line.strip()[:80]))
    return bad


def test_strings_closed(name: str, html: str) -> None:
    for k, js in enumerate(scripts_of(html)):
        bad = unterminated_lines(js)
        check(f"{name}: 따옴표가 줄을 안 넘어간다",
              not bad,
              "; ".join(f"{n}행 {t}" for n, t in bad[:3]) if bad else f"{len(js)}자")


def test_brackets(name: str, html: str) -> None:
    """괄호 짝 — 문자열·주석 밖에서만 센다."""
    for js in scripts_of(html):
        depth = {"()": 0, "{}": 0, "[]": 0}
        in_back = in_block = False
        quote = ""
        i = 0
        for line in js.split("\n"):
            i = 0
            while i < len(line):
                c = line[i]
                if in_block:
                    if line.startswith("*/", i):
                        in_block = False
                        i += 2
                        continue
                elif quote or in_back:
                    if c == BS:
                        i += 2
                        continue
                    if (quote and c == quote) or (in_back and c == "`"):
                        quote, in_back = "", False
                elif line.startswith("//", i):
                    break
                elif line.startswith("/*", i):
                    in_block = True
                    i += 2
                    continue
                elif c in "'\"":
                    quote = c
                elif c == "`":
                    in_back = True
                elif c in "({[":
                    depth["()" if c == "(" else "{}" if c == "{" else "[]"] += 1
                elif c in ")}]":
                    depth["()" if c == ")" else "{}" if c == "}" else "[]"] -= 1
                i += 1
        off = {k: v for k, v in depth.items() if v != 0}
        check(f"{name}: 괄호 짝이 맞는다", not off, str(off) if off else "")


# ----------------------------------------------------------------------
# ④ node가 있으면 진짜 파싱
# ----------------------------------------------------------------------

def test_node_parse(name: str, html: str) -> None:
    node = shutil.which("node")
    if not node:
        return
    for js in scripts_of(html):
        fd, path = tempfile.mkstemp(suffix=".js")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(js)
        try:
            r = subprocess.run([node, "--check", path],
                               capture_output=True, text=True, timeout=30)
            check(f"{name}: node 파싱", r.returncode == 0,
                  (r.stderr or "").strip().split("\n")[0][:110])
        finally:
            os.unlink(path)


def main() -> int:
    print("대시보드 JS 검사 — 브라우저 없이")
    test_source_escapes()

    built = {
        "/control": pages.control_page(140, 720),
        "/settings": pages.settings_page(),
        "/diag": pages.diag_page(),
    }
    for name, html in built.items():
        print(f"\n[{name}]")
        check(f"{name}: 스크립트가 있다", bool(scripts_of(html)), f"{len(html)}자")
        test_strings_closed(name, html)
        test_brackets(name, html)
        test_node_parse(name, html)

    print()
    if FAILED:
        print(f"❌ {len(FAILED)}개 실패 / {PASSED + len(FAILED)}개 중")
        for n in FAILED:
            print(f"   - {n}")
        return 1
    print(f"✅ 전부 통과 ({PASSED}개)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

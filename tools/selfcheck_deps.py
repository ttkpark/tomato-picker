#!/usr/bin/env python3
"""자체검증 4종이 **빈 환경에서 무엇이 없는지 말하고** 죽게 하는 한 곳.

왜 이걸 만들었나 — 2026-09-17, `.venv`에 `pyyaml`이 없어
`ros2/tools/ros_selfcheck.py`가 `ModuleNotFoundError`로 죽고 있었는데 그게
**"통과"로 오독될 뻔했다.** 파이썬 기본 역추적은 스무 줄이고 모듈 이름은 맨
마지막 한 줄에만 나온다 — 출력을 파이프로 넘겨 끝만 보는 습관(이 저장소가
그렇게 본다)에서는 "무엇이 없다"가 눈에 안 들어오고, 파이프 뒤 `$?`는 마지막
명령 것이라 0이 되기도 한다. 졸업 기준 4번(175종 통과)은 이 넷이 **실제로
도는 것**을 전제하므로, 안 돌았을 때는 그 사실이 한 줄로 보여야 한다.

쓰는 법 — 서드파티를 import하기 **전에** 부른다:

    from selfcheck_deps import require
    require("numpy", "yaml")
    import numpy as np
    import yaml

종료코드 규약: **2 = 환경이 없다**(설치하면 된다), 1 = 검사가 실패했다,
0 = 통과. 둘을 같은 1로 내면 "고쳐야 할 코드"와 "깔아야 할 패키지"가
구분이 안 된다.
"""

from __future__ import annotations

import importlib.util
import sys

# import 이름 → pip 이름. 둘이 다른 것(yaml↔PyYAML)이 바로 이 표가 필요한 이유다.
PIP_NAME = {
    "numpy": "numpy",
    "yaml": "PyYAML",
    "cv2": "opencv-python",
}

# 자체검증 4종이 쓰는 전부. requirements.txt에 이 셋이 다 있어야 한다
# (ros_selfcheck의 [의존성] 검사가 강제한다).
#
# ⚠ cv2는 **module-level import가 아니다** — eye_check가 부르는
#   `Eye.fruits()`가 함수 안에서 `..vision.color_detect`를 끌어오고 그것이
#   cv2를 쓴다(eye.py:496). 그래서 파일 머리만 훑어서는 안 보이고, 빈 venv에서
#   검사 **도중에** 죽는다(2026-09-17 실측: 60종 중 뒤쪽 test_two_cameras에서
#   ModuleNotFoundError, RC=1 — "검사 실패"로 오독되는 자리다).
#   여기 표는 파일 머리가 아니라 **실제로 돌려 본 결과**를 적는 곳이다.
SELFCHECK_MODULES = ("numpy", "yaml", "cv2")


def missing(*modules: str) -> list[str]:
    """없는 모듈 이름만 골라 돌려준다 — import는 하지 않는다(부작용 없이 묻는다)."""
    out = []
    for name in modules:
        try:
            found = importlib.util.find_spec(name) is not None
        except (ImportError, ValueError):
            found = False
        if not found:
            out.append(name)
    return out


def require(*modules: str) -> None:
    """없으면 **설치 한 줄**과 함께 종료코드 2로 죽는다."""
    gone = missing(*modules)
    if not gone:
        return
    pips = " ".join(PIP_NAME.get(m, m) for m in gone)
    print(f"❌ 자체검증을 돌릴 수 없다 — 없는 모듈: {', '.join(gone)}", file=sys.stderr)
    print(f"   설치하라:  {sys.executable} -m pip install {pips}", file=sys.stderr)
    print("   (이 스크립트는 실패가 아니라 **안 돌았다**. 통과로 세지 마라.)",
          file=sys.stderr)
    raise SystemExit(2)

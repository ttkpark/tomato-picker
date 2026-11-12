#!/usr/bin/env python3
"""**디지털 트윈 조작대** — 실기 조작대(click_server.py)를 가짜 팔(twin_io)에 물려 PC에서 **그대로** 띄운다.

    python ros2/tools/twin_server.py --port 8090
    → http://localhost:8090/  (같은 화면·같은 3D 미리보기)
    → curl -s -XPOST localhost:8090/run -d '{"job":"stage","args":{"target":"0,45,60,-20,0"}}'
    → curl -s localhost:8090/state   # arm_pose · tcp · tcp_err_mm · twin:true

주소·JSON·409·로그가 실기와 **구조적으로** 같다(같은 코드다). 보내는 쪽은 상대가 가짜인지 몰라도 된다.
가짜 팔 상태 = ~/twin_arm.json, 보정 사본 = ~/twin/{cal,cart}.json (docs/디지털트윈-구현방침-2026-11-08.md).
가짜 눈 = twin_cam.py(~/twin/kincal.json이 있을 때): 손목 D405의 컬러·깊이를 depth-cam과 같은 파일(/dev/shm, 윈도우는 C: 드라이브의 /dev/shm)로 낸다.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "tomato_bridge"))
sys.path.insert(0, HERE)

os.environ["TOMATO_TWIN"] = "1"
os.environ.setdefault("TOMATO_PY", sys.executable)
from tomato_bridge import twin_io  # noqa: E402

twin_io.ensure_calib()
import twin_cam  # noqa: E402

# 트윈의 눈: 역보정 모형(~/twin/kincal.json)이 있으면 가짜 손목 카메라가 depth-cam과 같은 파일을 낸다.
# 종이의 인쇄 배율도 실기와 같게 맞춰 둔다(검출기가 ~/target_print.json을 읽는다) — 이미 있으면 안 건드린다.
if os.path.exists(twin_cam.MODEL):
    import json
    _tp = os.path.expanduser("~/target_print.json")
    if not os.path.exists(_tp):
        json.dump({"scale": json.load(open(twin_cam.MODEL, encoding="utf-8"))["print_scale"], "_note": "twin_server가 모형에서 옮김"},
                  open(_tp, "w"))
    twin_cam.start()
import click_server  # noqa: E402

if __name__ == "__main__":
    sys.exit(click_server.main())

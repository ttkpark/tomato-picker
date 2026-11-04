#!/usr/bin/env python3
"""ArUco 인쇄물 두 장(A4, **실제 크기 100%로** 인쇄) → docs/aruco/*.pdf

    python tools/aruco_print.py

① charuco_wall.pdf — 벽에 붙이는 ChArUco 보드(5×7칸, 칸 35mm·마커 26mm, DICT_4X4_50).
   회전판 + 손목 D405 사슬 보정용: 차체를 세워 두고 회전판을 돌리며 같은 보드를 여러 각에서 본다.
② home_floor.pdf — 바닥에 붙이는 HOME 마커(ID 49, 160mm — 벽 보드의 ID 0~16과 안 겹치게). 화살표 = 로봇이 서 있을 정면.
두 장 모두 아래쪽 100mm 눈금자로 인쇄 배율을 확인한다(자로 재서 100mm가 아니면 다시 뽑는다).
"""
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DPI = 300
MM = DPI / 25.4
A4 = (round(210 * MM), round(297 * MM))
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
HOME_ID = 49                   # 벽 ChArUco가 0~16을 쓴다 — 같은 사전에서 겹치면 HOME을 보드로 착각한다
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "aruco")


def font(px: int):
    for f in ("malgun.ttf", "NanumGothic.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(f, px)
        except OSError:
            pass
    return ImageFont.load_default()


def ruler(d: ImageDraw.ImageDraw, x_mm: float, y_mm: float) -> None:
    """100mm 눈금자(10mm마다 긴 눈금)."""
    x0, y0 = x_mm * MM, y_mm * MM
    d.line([(x0, y0), (x0 + 100 * MM, y0)], fill=0, width=4)
    for i in range(0, 101, 5):
        h = 6 if i % 10 == 0 else 3
        d.line([(x0 + i * MM, y0), (x0 + i * MM, y0 - h * MM)], fill=0, width=3)
    d.text((x0 + 103 * MM, y0 - 6 * MM), "100 mm — 자로 재서 맞는지 확인", fill=0, font=font(round(4 * MM)))


def save(img: Image.Image, name: str) -> str:
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    img.save(path, "PDF", resolution=DPI)
    return path


def charuco_wall():
    sq, mk, nx, ny = 35.0, 26.0, 5, 7
    board = cv2.aruco.CharucoBoard((nx, ny), sq / 1000, mk / 1000, DICT)
    w, h = round(nx * sq * MM), round(ny * sq * MM)
    pat = board.generateImage((w, h), marginSize=0, borderBits=1)
    page = Image.new("L", A4, 255)
    ox, oy = round((A4[0] - w) / 2), round(18 * MM)
    page.paste(Image.fromarray(pat), (ox, oy))
    d = ImageDraw.Draw(page)
    d.text((ox, oy + h + 6 * MM), "ChArUco 5x7 · 칸 35mm · 마커 26mm · DICT_4X4_50 · 벽 보정용(회전판+손목 D405)",
           fill=0, font=font(round(4 * MM)))
    d.text((ox, oy + h + 13 * MM), "벽에 평평하게(구김 없이), 바닥에서 보드 가운데까지 30~40cm 높이로 붙인다",
           fill=0, font=font(round(4 * MM)))
    ruler(d, 20, 285)
    return page, save(page, "charuco_wall.pdf")


def home_floor():
    side = 160.0
    px = round(side * MM)
    mk = cv2.aruco.generateImageMarker(DICT, HOME_ID, px, borderBits=1)
    page = Image.new("L", A4, 255)
    ox, oy = round((A4[0] - px) / 2), round(55 * MM)
    page.paste(Image.fromarray(mk), (ox, oy))
    d = ImageDraw.Draw(page)
    cx = A4[0] / 2
    # 로봇 정면 화살표(마커 위쪽 = 앞)
    d.polygon([(cx, 15 * MM), (cx - 12 * MM, 35 * MM), (cx + 12 * MM, 35 * MM)], fill=0)
    d.line([(cx, 35 * MM), (cx, 50 * MM)], fill=0, width=round(3 * MM))
    d.text((cx + 16 * MM, 22 * MM), "로봇 정면", fill=0, font=font(round(6 * MM)))
    d.text((ox, oy + px + 8 * MM), "HOME · ArUco ID %d · 160mm · DICT_4X4_50" % HOME_ID, fill=0, font=font(round(6 * MM)))
    d.text((ox, oy + px + 18 * MM), "바닥에 테이프로 평평하게. 화살표가 로봇이 출발할 때 볼 방향",
           fill=0, font=font(round(4 * MM)))
    ruler(d, 20, 285)
    return page, save(page, "home_floor.pdf")


def check(page: Image.Image) -> list:
    """인쇄할 그림에서 마커가 다시 읽히는가 → 읽힌 ID들(사전·ID가 틀린 종이를 뽑지 않게)."""
    small = np.array(page.resize((page.width // 3, page.height // 3)))
    _, ids, _ = cv2.aruco.ArucoDetector(DICT).detectMarkers(small)
    return sorted(int(i) for i in ids.ravel()) if ids is not None else []


if __name__ == "__main__":
    (wall, pw), (home, ph) = charuco_wall(), home_floor()
    assert len(check(wall)) == 17, check(wall)          # 5×7 ChArUco = 마커 17개(ID 0~16)
    assert check(home) == [HOME_ID], check(home)
    print("만들었다(마커 다시 읽힘 확인):", os.path.normpath(pw), os.path.normpath(ph))

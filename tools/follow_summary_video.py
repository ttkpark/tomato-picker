"""0차 따라가기 요약 영상: 전경(PC) 위 + 앞·손목(젯슨) 아래, 4배속, 회마다 제목·결과 자막. 두 영상은 끝을 맞춘다.
    python tools/follow_summary_video.py avc1  →  out_avc1.mp4 (크다 — imageio-ffmpeg의 ffmpeg로 -crf 27 다시 인코딩해 16MB)
젯슨 영상(~/follow_trials/follow-*-H?.mp4)은 실행 폴더의 jv/에 먼저 받아 둔다. RUNS·자막은 벌마다 고친다."""
import sys, cv2, numpy as np
from PIL import Image, ImageDraw, ImageFont
REPO = r"C:/Users/parkg/Desktop/AIWorkspace/tomato-picker/docs/시험기록/영상/"
JV = "jv/"
RUNS = [("H1", "follow-20261109-205556-H1.mp4", "1회 — 성공 · 148초 · 끝 434mm · +4.0°", True),
        ("H2", "follow-20261109-210328-H2.mp4", "2회 — 성공 · 143초 · 끝 435mm · +3.2°", True),
        ("H3", "follow-20261109-211541-H3.mp4", "3회 — 실패(접근 실패) · 출발이 화분 약 2m, 조건 밖", False)]
SPEED, FPS, W, H = 4, 20, 1280, 1080
F = lambda n: ImageFont.truetype("C:/Windows/Fonts/malgunbd.ttf", n)

def text(img, lines, bar=None):
    im = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)); d = ImageDraw.Draw(im, "RGBA")
    for (x, y, s, size, col) in lines:
        b = d.textbbox((x, y), s, font=F(size)); d.rectangle((b[0]-8, b[1]-6, b[2]+8, b[3]+6), fill=(0, 0, 0, 170))
        d.text((x, y), s, font=F(size), fill=col)
    return cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)

def card(lines, secs):
    img = np.full((H, W, 3), (28, 24, 22), np.uint8)
    im = Image.fromarray(img); d = ImageDraw.Draw(im)
    y = 300
    for s, size, col in lines:
        w = d.textlength(s, font=F(size)); d.text(((W - w) / 2, y), s, font=F(size), fill=col); y += size + 28
    out = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
    return [out] * int(secs * FPS)

four = sys.argv[1]
vw = cv2.VideoWriter("out_%s.mp4" % four, cv2.VideoWriter_fourcc(*four), FPS, (W, H))
assert vw.isOpened(), "codec"
WHITE, RED, GREEN, GREY = (255, 255, 255), (239, 90, 60), (95, 192, 138), (200, 195, 185)
for f in card([("토마토 로봇 · 체크포인트 0차", 54, RED), ("따라가기: 스캔 → 찾기 → 앞으로 → 손목 근접", 40, WHITE),
               ("2026-11-09 · 코드 고정 3회 · 사람 개입 0 · 4배속", 30, GREY),
               ("결과 2 / 3   (기준: 끝 거리 450±50mm · 가로 ±15° · 5분 안)", 34, GREEN)], 4): vw.write(f)
for tag, jf, cap, ok in RUNS:
    a = cv2.VideoCapture(REPO + "follow_here_20261109-%s.mp4" % tag); b = cv2.VideoCapture(JV + jf)
    na, nb = int(a.get(7)), int(b.get(7))                 # 10fps / 5fps
    dur = min(na / 10.0, nb / 5.0)                       # 끝을 맞춘다 — 둘 다 같은 순간에 멈췄다
    t0a, t0b = na / 10.0 - dur, nb / 5.0 - dur
    step = SPEED / FPS; n = int(dur / step); last_b = None; ib = -1; ia = -1; fa = None
    for k in range(n):
        t = k * step
        ta = int((t0a + t) * 10)
        while ia < ta:
            ok_a, x = a.read(); ia += 1
            if ok_a: fa = x
        tb = int((t0b + t) * 5)
        while ib < tb:
            ok_b, x = b.read(); ib += 1
            if ok_b: last_b = x
        if fa is None or last_b is None: continue
        frame = np.zeros((H, W, 3), np.uint8)
        frame[:720] = cv2.resize(fa, (W, 720)); frame[720:] = cv2.resize(last_b, (W, 360))
        frame = text(frame, [(20, 14, "전경", 26, GREY), (20, 730, "앞 카메라", 24, GREY), (660, 730, "손목 D405", 24, GREY),
                             (20, 660, cap, 30, GREEN if ok else RED), (W - 250, 14, "%s · %d초 · ×%d" % (tag, int(t), SPEED), 24, WHITE)])
        vw.write(frame)
    for _ in range(int(1.5 * FPS)): vw.write(frame)
for f in card([("0차 따라가기 2 / 3", 54, GREEN), ("남은 일: 잡기 20회 규약 측정 · 손-눈 재보정 · 접근 끝과 잡기 잇기", 30, WHITE)], 3): vw.write(f)
vw.release(); print("done", four)

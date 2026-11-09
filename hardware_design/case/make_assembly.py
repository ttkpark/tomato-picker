"""케이스 v2 조립도 생성기 — tomato_case.scad의 part="step" 장면을 OpenSCAD로 뽑아 HTML 한 장에 넣는다.

    python hardware_design/case/make_assembly.py      → docs/design/case-assembly-guide.html

scad를 고쳤으면 다시 돌린다. 그림은 data: URI로 박혀 파일 하나로 오프라인에서 열린다.
"""
import base64
import concurrent.futures as cf
import html
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCAD = HERE / "tomato_case.scad"
OUT = HERE.parents[1] / "docs" / "design" / "case-assembly-guide.html"
OPENSCAD = shutil.which("openscad") or r"C:\Program Files\OpenSCAD\openscad.exe"

C = (126, 125)  # 회전 중심 = 몸통 가운데 (scad의 rot_center)


def cam(center, direction, dist):
    """OpenSCAD --camera=eye,center. direction은 중심에서 눈 쪽."""
    n = sum(d * d for d in direction) ** 0.5
    eye = [c + d / n * dist * 1.7 for c, d in zip(center, direction)]   # OpenSCAD 시야각 22.5°라 멀리 둔다
    return ",".join(f"{v:.0f}" for v in eye + list(center))


FL, FR, BR = (-0.5, -0.7, 0.6), (0.5, -0.7, 0.6), (0.5, 0.75, 0.45)
TOP_FL = (-0.45, -0.6, 0.8)

# (번호, 제목, 카메라, 무엇을, [나사], [할 일], 주의) — 번호는 scad steps()의 it(i) 와 같다
STEPS = [
    (1, "인서트 12개", cam((*C, 55), FL, 560), "트레이 기둥 6개의 위·아래 구멍",
     ["M3 열압입 인서트 (외경 4 × 길이 5) × 12"],
     ["기둥 위 구멍 6곳에 인서트 — 뚜껑이 여기에 조인다.",
      "트레이를 뒤집어 기둥 아래 구멍 6곳에 인서트 — 치마가 여기에 조인다.",
      "인두 끝을 인서트에 대고 수직으로 천천히 누른다. 윗면이 기둥 면과 같아지면 멈춘다."],
     "구멍은 Ø4.2, 깊이 7. 기울게 박히면 뚜껑 나사가 안 맞으니 하나 박을 때마다 직각을 본다."),
    (2, "치마 → 트레이", cam((*C, -20), (-0.45, -0.7, -0.35), 640), "트레이 밑 둘레",
     ["M3×12 × 6 — 밑에서 위로, 기둥 아래 인서트"],
     ["치마의 사다리꼴 홈이 바퀴 자리(좌우 4·앞뒤 4)에 오도록 방향을 맞춘다.",
      "치마 귀 6곳을 트레이 기둥 밑에 대고 위로 조인다."],
     "치마는 배선 가림·겉모습용이라 없어도 동작한다. 나중에 달아도 되지만 차체에 올린 뒤에는 바퀴가 손을 막는다."),
    (3, "받침 기둥 + 트레이 → 차체", cam((*C, 0), (-0.5, -0.75, 0.18), 700), "차체 상판의 50×30 나사 패턴 두 벌",
     ["M3×25 + 너트 × 8 — 상판 밑에서 위로, 너트는 트레이 바닥 안쪽"],
     ["지금 달린 케이스(TOPLeft·TOPRight)와 그 나사를 뗀다.",
      "상판으로 내려갈 선(모터·엔코더·전원)을 먼저 트레이 바닥의 선 구멍 4곳과 기둥 가운데 홈으로 뺀다.",
      "기둥 2개를 패턴 위에 놓고 트레이를 얹은 뒤, 나사 4개씩 상판 밑에서 끼워 트레이 안쪽에서 너트로 조인다."],
     "기둥 하나가 나사 4개를 다 받는다. 팔을 옆으로 뻗어도 흔들리지 않게 하려는 것이니 8개를 다 쓴다."),
    (4, "벅부스트 2장", cam((*C, 40), (0.45, -0.6, 0.8), 520), "오른쪽, 긴 쪽이 앞뒤",
     ["M3×6~8 셀프탭 × 8"],
     ["12V(팔)·7.5V(바퀴) 두 장을 나란히 보스 위에 얹고 조인다.",
      "출력 선은 젯슨 밑(받침 12mm 틈)으로, 입력 선은 왼쪽 배터리로 보낸다."],
     "오른벽은 전 높이가 환기창이다. 판 위아래가 트여 있어 나중에도 손이 들어간다."),
    (5, "제어 기판", cam((*C, 40), TOP_FL, 520), "앞 가운데, Astra 크래들 밑",
     ["M3×6 × 4"],
     ["Uno + Moebius 쉴드를 보스 4개에 얹고 조인다. STM32 보드(100×70)도 구멍이 같아 같은 자리에 앉는다.",
      "USB 선은 젯슨 쪽으로 뺀다."],
     "크래들 밑과 기판 사이는 35mm다. 쉴드 위 부품(30)은 들어가지만 세로로 꽂는 커넥터는 피한다."),
    (6, "IMU", cam((*C, 30), (0.4, -0.55, 0.85), 420), "젯슨과 벅부스트 사이, 앞뒤 가운데",
     ["M2×5 × 2"],
     ["MPU6050(GY-521)을 칩 면이 위로 오게 놓고, 핀 쪽 가장자리 구멍 2개를 조인다.",
      "I²C 선(버스 7)은 젯슨 40핀으로 짧게 뺀다."],
     "진동이 크면 나사 대신 양면 폼으로 붙인다. 회전 짐작은 Z축(요)만 쓴다."),
    (7, "배터리", cam((*C, 40), FL, 520), "왼쪽, 세워서 긴 쪽이 앞뒤",
     ["나사 없음 — 벨크로 밴드 2줄"],
     ["4S3P 배터리를 턱 안에 세운다. 왼벽과 25mm 틈이 선 자리다.",
      "바닥의 밴드 슬롯 두 쌍으로 벨크로를 통과시켜 묶는다."],
     "배터리(약 560g)가 가운데서 76mm 왼쪽이라 무게 중심이 왼쪽으로 쏠린다. 직진이 틀어지면 오른쪽 예비 칸에 추를 얹는다."),
    (8, "젯슨", cam((*C, 40), FL, 520), "정중앙, 단자가 뒤로",
     ["나사 없음 — 레일 위에 얹고 네 모서리 턱"],
     ["밑으로 지나갈 선(벅부스트·기판)을 먼저 정리한다.",
      "Orin Nano를 좌우 레일 위에 얹어 네 모서리 턱 안에 넣는다. USB·LAN 단자가 뒤 칸을 본다."],
     "WiFi 안테나 판에는 고정 구멍이 없어 턱으로만 막는다. 뒤 턱은 단자를 안 막게 2mm만 올라와 있다."),
    (9, "전원 패널", cam((60, 55, 50), (-0.85, -0.4, 0.35), 460), "왼벽 앞쪽",
     ["OLED M2×5 × 4", "스위치·전압계는 끼움, 잭은 자기 너트"],
     ["전압계를 위 창에, 로커 스위치(KCD1)를 아래 왼쪽 창에 밖에서 눌러 끼운다.",
      "DC 충전 잭을 아래 오른쪽 구멍에 넣고 안쪽에서 너트로 조인다.",
      "OLED(SSD1306 0.96\")를 안쪽 기둥 4개에 조인다. 화면이 가운데 창에 맞는다."],
     "OLED는 젯슨 I²C 버스 7(0x3C)에 IMU와 같이 물린다. 주소가 안 겹친다. 띄우는 코드는 아직 없다."),
    (10, "Astra + 캡", cam((126, 30, 110), (-0.45, -0.75, 0.6), 480), "앞 윗면 크래들",
     ["M3×20 × 4 — 양 끝벽에 2개씩"],
     ["접이식 받침대를 접은 채 크래들 바닥 구멍(62×26)으로 넣고, 본체를 홈에 눕힌다. 렌즈가 앞이다.",
      "USB 선도 같은 구멍으로 트레이 안에 넣는다.",
      "캡을 덮고 끝벽 나사 4개를 조인다."],
     "크래들 앞면은 앞벽과 한 면이다. 앞으로 튀어나오지 않는다. 캡은 11/09 판을 새로 뽑아야 맞는다."),
    (11, "회전 서보 → 뚜껑", cam((*C, 95), (-0.5, -0.6, -0.55), 420), "뚜껑 밑 서보 걸이 (작업대)",
     ["케이블 타이 2줄 — 걸이 긴 면의 슬롯"],
     ["뚜껑을 뒤집어 작업대에 놓는다. 여기부터 14단계까지 뚜껑만 따로 조립한다.",
      "STS3215(ID 7)를 출력축이 회전 중심에 오게 걸이에 밀어 넣는다. 몸통이 뚜껑 판 속으로 3mm 올라간다.",
      "걸이 긴 면의 슬롯 두 줄로 케이블 타이를 둘러 조인다. 선은 걸이 끝 홈으로 뺀다."],
     "서보는 팔과 같은 12V 버스에 한 줄 더 물린다. 0/4095 경계가 가동 구간 밖(뒤쪽)에 있는지 조립 전에 한 번 읽어 본다."),
    (12, "베어링 + 누름 링", cam((*C, 140), (-0.5, -0.6, 0.7), 440), "뚜껑 윗면 베어링 받침",
     ["M3×12 셀프탭 × 4"],
     ["6816-2RS 베어링을 받침에 손으로 꽉 눌러 넣는다.",
      "누름 링을 얹고 나사 4개를 조인다. 회전판을 올리기 전에 해야 한다."],
     "바깥 링을 위아래로 붙잡아야 팔을 뻗을 때 한쪽이 들리지 않는다."),
    (13, "서보 혼", cam((*C, 115), (-0.5, -0.6, 0.75), 360), "베어링 가운데로 보이는 서보 출력축",
     ["혼 가운데 M3 고정 나사 × 1"],
     ["원판 혼(Ø20)을 25T 스플라인에 끼우고 가운데 나사로 고정한다."],
     "지금 로봇에서 떼기 전에 혼과 회전판에 같은 선을 그어 둔다. 같은 자리로 다시 끼워야 영점(정면 2048)이 안 바뀐다."),
    (14, "회전판", cam((*C, 160), (-0.5, -0.6, 0.7), 500), "베어링 안쪽 링",
     ["M3×20 × 4 — 회전판 위에서 혼으로"],
     ["회전판 허브를 베어링 안쪽 링에 밀어 넣는다. 딸깍 하고 걸쇠가 걸린다.",
      "13단계에서 그은 선을 맞추고, 혼 나사 4개를 위에서 조인다. 머리는 3mm 홈에 묻힌다."],
     "스토퍼 핀은 부러져 없다. 서보 EEPROM 한계(1024~3072, ±90°)가 유일한 멈춤이다."),
    (15, "뚜껑 → 트레이", cam((*C, 90), FL, 760), "트레이 위",
     ["M3×12 × 6 — 기둥 위 인서트"],
     ["서보 선을 젯슨 위 18mm 칸으로 내리고, 회전판 쪽 선은 180° 호형 슬롯으로 지나게 한다.",
      "뚜껑 앞 ㄷ자 홈이 Astra 캡을 비키도록 내려놓는다.",
      "기둥 6곳을 조인다."],
     "배선은 회전 중심을 못 지난다(서보 축이 있다). 회전판을 양 끝까지 돌려 보고 선이 당기지 않는지 본다."),
    (16, "뒷판", cam((126, 240, 50), BR, 620), "뒷벽 창",
     ["M3×12 셀프탭 × 4"],
     ["젯슨 단자에 USB·전원 선을 꽂고, 뒤 칸 케이블 타이 고리 3개에 정리한다.",
      "뒷판을 대고 4곳을 조인다. 선은 아래 120×16 출구로 나간다."],
     "뒷판만 떼면 뚜껑을 안 열고도 단자에 손이 닿는다."),
    (17, "SO-101 팔", cam((*C, 150), FL, 600), "회전판 윗면 인서트 4곳",
     ["SO-101 받침의 나사 그대로 × 4"],
     ["팔 받침을 앞 쌍(간격 56.7, 축 앞 15)·뒤 쌍(간격 64.4, 축 뒤 55)에 맞춰 얹는다.",
      "나사는 회전판 두께 6을 넘지 않는 길이로 조인다."],
     "인서트는 지금 회전판에 이미 박혀 있다."),
]

HERO = [
    ("front", cam((*C, 70), FL, 600), "-D step=18", "완성 — 앞에서"),
    ("back", cam((*C, 70), BR, 600), "-D step=18", "완성 — 뒤에서"),
    ("explode", cam((*C, 130), (-0.5, -0.7, 0.55), 900), "-D explode=true", "분해도"),
]

PRINTS = [  # README "최종 출력 목록"과 같다
    ("tray.stl", 1, "252 × 251 × 105", "그대로", "크래들 밑·뒷벽 창 위. 스커트·브림 끔"),
    ("lid.stl", 1, "252 × 250 × 49", "뒤집어서 서보 걸이가 위로", "없음"),
    ("astra_cap.stl", 1, "184 × 49 × 18", "그대로", "없음"),
    ("riser.stl", 2, "62 × 42 × 15", "그대로", "없음"),
    ("rear_cover.stl", 1, "188 × 96 × 2.5", "그대로", "없음"),
    ("skirt.stl", 1, "252 × 250 × 30", "그대로", "없음 (선택 부품)"),
]

SCREWS = [  # README "나사·인서트"와 같다
    ("상판 ↔ 기둥 ↔ 트레이 바닥", "M3×25 + 너트", 8, "새로 산다"),
    ("뚜껑 ↔ 트레이 기둥", "M3×12", 6, "기둥 위 인서트"),
    ("치마 ↔ 트레이 기둥", "M3×12", 6, "기둥 아래 인서트"),
    ("뒷판", "M3×12", 4, "셀프탭"),
    ("누름 링 ↔ 베어링 받침", "M3×12", 4, "셀프탭"),
    ("서보 혼 ↔ 회전판", "M3×20", 4, "혼의 암나사"),
    ("Astra 캡 ↔ 크래들", "M3×20", 4, "크래들 셀프탭"),
    ("제어 기판 · 벅부스트", "M3×6~8", "4 · 8", "셀프탭"),
    ("IMU · OLED", "M2×5", "2 · 4", "셀프탭"),
    ("열압입 인서트", "M3 외경4 × 길이5", "12 (+여분 3)", "구멍 Ø4.2"),
]

PARTS = [
    ("회전판 · 누름 링", "지금 로봇에서 떼어 옮겨 단다"),
    ("6816-2RS 베어링 · STS3215 회전 서보(ID 7)", "지금 것"),
    ("Astra Pro", "165 × 30 × 40, 받침대 접은 채"),
    ("Jetson Orin Nano 개발 키트", "103 × 90.5 × 35"),
    ("배터리 4S3P", "134 × 43 × 57, 벨크로 밴드 2줄"),
    ("벅부스트 2장", "76.6 × 45, 12V 팔 · 7.5V 바퀴"),
    ("Uno + Moebius 쉴드 또는 STM32 보드", "같은 구멍"),
    ("MPU6050 · SSD1306 OLED", "젯슨 I²C 버스 7"),
    ("전압계 · KCD1 로커 · DC 잭", "옛 판과 같은 구멍"),
    ("케이블 타이", "서보 걸이 2줄 + 뒤 칸 정리"),
]

AFTER = [
    ("회전판·Astra가 상판 기준 45mm 올라간다", "space_map selfcal로 ~/space_mount.json을 다시 잡는다."),
    ("Astra 손-눈 보정이 옛 자리 값이다", "~/arm_eye.json의 cameras.astra를 fixed 식으로 다시 푼다."),
    ("대기 자세·잡기 시작 자세의 높이", "팔 받침이 45mm 높아졌으니 바닥 쪽 자세를 다시 본다."),
    ("무게 중심이 왼쪽으로 약 10mm", "직진 시험에서 틀어지면 오른쪽 예비 칸에 추를 얹는다."),
]


def render(job):
    name, camera, extra, tmp = job
    out = Path(tmp) / f"{name}.png"
    cmd = [OPENSCAD, "-D", 'part="step"', *extra.split(), f"--camera={camera}",
           "--imgsize=1200,900", "--colorscheme=Tomorrow", "-o", str(out), str(SCAD)]
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode or not out.exists():
        sys.exit(f"{name} 렌더 실패:\n{r.stderr[-2000:]}")
    return name, "data:image/png;base64," + base64.b64encode(out.read_bytes()).decode()


CSS = """
/* 레이아웃: 공장 작업지시서 — 왼쪽 단계 레일 + 본문 한 단, 그림은 밝은 판 위에 */
:root{
  --bg:#f3f4f2; --paper:#ffffff; --plate:#f8f8f8; --ink:#1d2321; --muted:#5d6763; --line:#d5d9d6;
  --accent:#e2561a; --accent-ink:#b23d0b; --screw:#2c5d8f; --ok:#2f7d4f;
  --sans:"IBM Plex Sans KR","Apple SD Gothic Neo","Malgun Gothic",system-ui,sans-serif;
  --mono:"IBM Plex Mono",ui-monospace,Consolas,monospace;
}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){
  --bg:#141716; --paper:#1c201f; --plate:#f2f2f0; --ink:#e7ebe8; --muted:#9aa5a0; --line:#323836;
  --accent:#f07a3e; --accent-ink:#f59a68; --screw:#7fb0e0; --ok:#5cc489; color-scheme:dark}}
:root[data-theme="dark"]{
  --bg:#141716; --paper:#1c201f; --plate:#f2f2f0; --ink:#e7ebe8; --muted:#9aa5a0; --line:#323836;
  --accent:#f07a3e; --accent-ink:#f59a68; --screw:#7fb0e0; --ok:#5cc489; color-scheme:dark}
*{box-sizing:border-box}
body{background:var(--bg);color:var(--ink);font:15px/1.65 var(--sans);margin:0}
.wrap{max-width:1180px;margin:0 auto;padding-left:max(16px,env(safe-area-inset-left,0px));padding-right:max(16px,env(safe-area-inset-right,0px));padding-block:28px 80px;display:grid;grid-template-columns:190px minmax(0,1fr);gap:40px}
nav.toc{position:sticky;top:calc(env(safe-area-inset-top,0px) + 20px);align-self:start;font-size:13px;max-height:calc(100vh - 40px);overflow-y:auto}
nav.toc a{display:flex;gap:8px;color:var(--muted);text-decoration:none;padding:2px 0 2px 10px;border-left:2px solid var(--line)}
nav.toc a span{font:500 12px/1.9 var(--mono);color:var(--accent-ink);min-width:2ch}
nav.toc a:hover,nav.toc a:focus-visible{color:var(--ink);border-color:var(--accent)}
nav.toc .grp{font:500 11px var(--mono);letter-spacing:.08em;text-transform:uppercase;color:var(--muted);margin:16px 0 4px}
main{min-width:0}
h1{font-size:34px;line-height:1.2;margin:0 0 8px;text-wrap:balance;letter-spacing:-.01em}
h2{font-size:22px;margin:56px 0 14px;padding-top:14px;border-top:2px solid var(--ink);text-wrap:balance}
h3{font-size:18px;margin:0;text-wrap:balance}
p{margin:0 0 10px;max-width:68ch}
code{font:13px var(--mono);background:color-mix(in srgb,var(--accent) 12%,transparent);padding:1px 5px;border-radius:3px}
.lede{color:var(--muted);max-width:68ch}
.spec{display:flex;flex-wrap:wrap;gap:6px 22px;margin:16px 0 0;font:13px var(--mono);color:var(--muted)}
.spec b{color:var(--ink);font-weight:500}
.legend{display:flex;flex-wrap:wrap;gap:8px 20px;margin-top:14px;font-size:13px;color:var(--muted)}
.legend i{display:inline-block;width:12px;height:12px;border-radius:2px;margin-right:6px;vertical-align:-1px}
.plate{background:var(--plate);border:1px solid var(--line);border-radius:6px;overflow:hidden;margin:0}
.plate img{display:block;width:100%;height:auto}
.plate figcaption{font-size:12.5px;color:#5d6763;padding:4px 10px 8px}
.hero{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:22px}
.explode{margin-top:12px}
.explode img{max-width:900px;margin:0 auto}
.tbl{overflow-x:auto;border:1px solid var(--line);border-radius:6px;background:var(--paper)}
table{border-collapse:collapse;width:100%;font-size:13.5px;min-width:560px}
th,td{padding:7px 12px;text-align:left;border-bottom:1px solid var(--line);vertical-align:top}
tbody tr:last-child td,tbody tr:last-child th{border-bottom:0}
thead th{font:500 11.5px var(--mono);letter-spacing:.05em;color:var(--muted);background:color-mix(in srgb,var(--ink) 4%,transparent)}
td{font-variant-numeric:tabular-nums}
td.n{text-align:right;font-family:var(--mono)}
td.f{font-family:var(--mono);font-size:13px}
.two{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:16px;align-items:start}
.note{color:var(--muted);font-size:14px;margin-top:10px}
.step{display:grid;grid-template-columns:minmax(0,1.15fr) minmax(0,1fr);gap:20px;background:var(--paper);border:1px solid var(--line);border-radius:6px;padding:16px;margin-bottom:16px;scroll-margin-top:16px}
.step header{display:flex;align-items:baseline;gap:12px;margin-bottom:4px}
.num{font:500 28px/1 var(--mono);color:var(--accent);min-width:2ch}
.where{font-size:13px;color:var(--muted);margin-bottom:10px}
.screws{list-style:none;padding:0;margin:0 0 10px;display:grid;gap:4px}
.screws li{font:13px var(--mono);color:var(--screw);padding-left:16px;position:relative}
.screws li::before{content:"";position:absolute;left:0;top:.55em;width:8px;height:8px;border-radius:50%;background:var(--screw)}
.do{margin:0 0 10px;padding-left:20px}
.do li{margin:3px 0}
.why{color:var(--muted);font-size:14px;border-left:2px solid var(--line);padding-left:10px;margin:0}
.bench{font:500 11px var(--mono);letter-spacing:.06em;color:var(--accent-ink);border:1px solid var(--accent);border-radius:3px;padding:0 6px;margin-left:auto;white-space:nowrap}
.after{display:grid;gap:10px}
.after div{background:var(--paper);border:1px solid var(--line);border-radius:4px;padding:10px 14px}
.after b{display:block}
.after span{color:var(--muted);font-size:14px}
footer{margin-top:48px;font-size:13px;color:var(--muted)}
:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
@media (max-width:860px){
  .wrap{grid-template-columns:1fr;gap:0}
  nav.toc{display:none}
  .hero,.two,.step{grid-template-columns:1fr}
  h1{font-size:27px}
}
"""


def esc(s):
    return html.escape(str(s), quote=True)


def build(imgs):
    toc = ['<div class="grp">개요</div>', '<a href="#prints">출력물</a>', '<a href="#parts">부품·나사</a>',
           '<div class="grp">트레이</div>']
    for n, title, *_ in STEPS:
        if n == 11:
            toc.append('<div class="grp">뚜껑 작업대</div>')
        if n == 15:
            toc.append('<div class="grp">마무리</div>')
        toc.append(f'<a href="#s{n}"><span>{n:02d}</span>{esc(title)}</a>')
    toc.append('<a href="#after">조립 후 다시 잴 것</a>')

    steps = []
    for n, title, _, where, screws, todo, why in STEPS:
        bench = '<span class="bench">작업대</span>' if 11 <= n <= 14 else ""
        steps.append(f"""<section class="step" id="s{n}">
  <figure class="plate"><img src="{imgs[f's{n}']}" alt="{n}단계 {esc(title)}" loading="lazy" width="1200" height="900"></figure>
  <div>
    <header><span class="num">{n:02d}</span><h3>{esc(title)}</h3>{bench}</header>
    <div class="where">{esc(where)}</div>
    <ul class="screws">{''.join(f'<li>{esc(s)}</li>' for s in screws)}</ul>
    <ol class="do">{''.join(f'<li>{esc(t)}</li>' for t in todo)}</ol>
    <p class="why">{esc(why)}</p>
  </div>
</section>""")

    prints = "".join(f'<tr><td class="f">{esc(f)}</td><td class="n">{q}</td><td>{esc(sz)}</td><td>{esc(o)}</td><td>{esc(s)}</td></tr>'
                     for f, q, sz, o, s in PRINTS)
    screws = "".join(f'<tr><td>{esc(a)}</td><td class="f">{esc(b)}</td><td class="n">{esc(c)}</td><td>{esc(d)}</td></tr>'
                     for a, b, c, d in SCREWS)
    parts = "".join(f'<tr><td>{esc(a)}</td><td>{esc(b)}</td></tr>' for a, b in PARTS)
    after = "".join(f'<div><b>{esc(a)}</b><span>{esc(b)}</span></div>' for a, b in AFTER)

    return f"""<title>토마토 로봇 케이스 v2 조립도</title>
<meta charset="utf-8">
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans+KR:wght@400;500;700&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>{CSS}</style>
<div class="wrap">
<nav class="toc" aria-label="목차">{''.join(toc)}</nav>
<main>
<h1>토마토 로봇 케이스 v2 조립도</h1>
<p class="lede">출력물 6종 7개와 지금 로봇에서 옮겨 다는 부품이 어디로, 어떤 순서로 들어가는지 정리했다. 그림은 모두 <code>tomato_case.scad</code>에서 바로 뽑았다. 1~10단계는 트레이, 11~14단계는 뚜껑만 따로 작업대에서, 15단계에서 둘을 합친다.</p>
<div class="spec"><span>몸통 <b>252 × 250 × 104</b></span><span>상판 위 회전판 윗면 <b>141</b></span><span>바퀴 중심 <b>앞뒤 165 · 좌우 212</b></span><span>회전 범위 <b>±90°</b> (서보 EEPROM)</span></div>
<div class="legend"><span><i style="background:#e2561a"></i>이 단계에 붙이는 것</span><span><i style="background:#2c5d8f"></i>나사</span><span><i style="background:#c9a640"></i>인서트</span><span><i style="background:#3a3d3f"></i>차체·바퀴</span></div>
<div class="hero">
  <figure class="plate"><img src="{imgs['front']}" alt="완성 조립 앞" width="1200" height="900"><figcaption>완성 — 앞에서. Astra 크래들이 앞벽과 한 면이다.</figcaption></figure>
  <figure class="plate"><img src="{imgs['back']}" alt="완성 조립 뒤" width="1200" height="900"><figcaption>완성 — 뒤에서. 뒷판 아래가 케이블 출구다.</figcaption></figure>
</div>
<figure class="plate explode"><img src="{imgs['explode']}" alt="분해도" width="1200" height="900"><figcaption>분해도 — 아래부터 차체, 받침 기둥·치마, 트레이와 내용물, Astra와 캡, 뚜껑·서보, 베어링·누름 링, 회전판.</figcaption></figure>

<h2 id="prints">출력물</h2>
<div class="tbl"><table><thead><tr><th>파일 (hardware_design/case/stl)</th><th>개수</th><th>크기 mm</th><th>베드에 놓는 법</th><th>서포트</th></tr></thead><tbody>{prints}</tbody></table></div>
<p class="note">재질은 PETG 이상(젯슨 열). 베드 256 기준 트레이·뚜껑·치마는 여유가 2~6mm라 슬라이서에서 먼저 올려 본다. 회전판·누름 링은 뽑지 않는다.</p>

<h2 id="parts">부품·나사</h2>
<div class="two">
  <div class="tbl"><table style="min-width:0"><thead><tr><th>옮겨 다는 부품</th><th>메모</th></tr></thead><tbody>{parts}</tbody></table></div>
  <div class="tbl"><table style="min-width:0"><thead><tr><th>자리</th><th>나사</th><th>개수</th><th>받는 쪽</th></tr></thead><tbody>{screws}</tbody></table></div>
</div>

<h2>조립 순서</h2>
{''.join(steps)}

<h2 id="after">조립 후 다시 잴 것</h2>
<div class="after">{after}</div>

<footer>생성: <code>python hardware_design/case/make_assembly.py</code> · 원본 <code>hardware_design/case/tomato_case.scad</code> · 설계 판단은 <code>hardware_design/case/README.md</code>.</footer>
</main>
</div>
"""


def main():
    tmp = tempfile.mkdtemp(prefix="case_asm_")
    jobs = [(f"s{n}", camera, f"-D step={n}", tmp) for n, _, camera, *_ in STEPS]
    jobs += [(name, camera, extra, tmp) for name, camera, extra, _ in HERO]
    with cf.ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as ex:
        imgs = dict(ex.map(render, jobs))
    OUT.write_text(build(imgs), encoding="utf-8", newline="\n")
    shutil.rmtree(tmp, ignore_errors=True)
    print(f"{OUT} ({OUT.stat().st_size / 1e6:.1f} MB, 그림 {len(imgs)}장)")


if __name__ == "__main__":
    main()

# 메카넘 STM32 주행보드 — 부품·넷·배치를 한 곳에서 정한다 (KiCad 9 pcbnew).
# 실행: "C:/Program Files/KiCad/9.0/bin/python.exe" gen_pcb.py   → mecanum_stm32.kicad_pcb (배치 + 미리 깐 모터선)
#
# 회로도(.kicad_sch)는 없다. 이 파일이 회로도다 — 넷을 패드에 직접 물려 래츠네스트가 뜨고,
# 라우팅은 route.py(+Freerouting)가 한다. 부품을 고치면 여기 고치고 다시 돌린다.
# 펌웨어 핀표 = firmware/mecanum_stm32/src/board.h 와 1:1 (어긋나면 둘 중 하나가 틀렸다).
# 고정구멍 = 아두이노 우노와 같은 자리(왼쪽 아래 기준, M3 4개). 판 100×70(케이스 v2의 100×70 칸).
#
# 2026-11-09 개정(검토 반영): 좌표는 판 위 **절대 mm**(예전엔 Uno 비율로 늘려 디커플링이 최대 34mm 흩어졌다).
# IC마다 디커플링·타이밍 부품을 IC 패드 기준 상대 좌표로 묶어 FIXED로 박고, 배치 뒤 거리 검사(3mm)가 판을 거절한다.
# 전원(VBAT_IN/VBAT_R/VBAT_P/VM)은 F.Cu 동박 면(POWER_ZONES), 모터 출력은 1.0mm 선을 여기서 미리 깐다(PREROUTES).
import os, sys, math
import pcbnew

FP = r"C:/Program Files/KiCad/9.0/share/kicad/footprints"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mecanum_stm32.kicad_pcb")
W, H = 100.0, 70.0
ISO_X = 30.0               # 절연 경계 x = ADuM1201(U5) 중심. 왼쪽·위(y<ISO_Y)가 USB 칸(GNDU)
ISO_Y = 19.5               # USB 칸 아래 경계 mm
ISO_GAP = 1.5              # GNDU ↔ 나머지 동박 최소 간격(검사·해자 폭)
DECAP_MAX = 3.0            # 디커플링·타이밍 부품 패드 ↔ IC 핀 패드 최대 거리(가장자리 기준)

R06, C06, C08, C12, LED = ("Resistor_SMD", "R_0603_1608Metric"), ("Capacitor_SMD", "C_0603_1608Metric"), \
    ("Capacitor_SMD", "C_0805_2012Metric"), ("Capacitor_SMD", "C_1206_3216Metric"), ("LED_SMD", "LED_0603_1608Metric")
SOD123 = ("Diode_SMD", "D_SOD-123")
# DRV8874 HTSSOP: 비아 없는 풋프린트 + build()에서 0.3/0.6mm 방열 비아 12개를 넣는다(예전 0.2mm×48은 2층 공정 최소 미달)
HTSSOP = ("Package_SO", "HTSSOP-16-1EP_4.4x5mm_P0.65mm_EP3.4x5mm_Mask2.46x2.31mm")
THERMAL_VIAS = [(x, y) for x in (-1.0, 0.0, 1.0) for y in (-1.5, -0.5, 0.5, 1.5)]
PH6 = ("Connector_JST", "JST_PH_B6B-PH-K_1x06_P2.00mm_Vertical")
H254 = lambda n: ("Connector_PinHeader_2.54mm", f"PinHeader_1x{n:02d}_P2.54mm_Vertical")

parts = []      # (ref, (lib, fp), value, {pad: net}, x, y, rot)
FIXED = set()   # settle()가 절대 안 움직이는 부품
DECAPS = []     # (부품, IC, IC 핀) — 배치 뒤 거리 검사 대상
PREROUTES = []  # (net, width_mm, [(x, y), ...]) F.Cu — 모터 출력·접지 짧은 선
POWER_ZONES = {"VBAT_IN": [], "VBAT_R": [], "VBAT_P": [], "VM": []}   # net → 다각형 목록(절대 mm, 합쳐서 한 면)


def P(ref, fp, val, pads, x, y, rot=0, fixed=False):
    parts.append((ref, fp, val, {str(k): v for k, v in pads.items()}, x, y, rot % 360))
    if fixed:
        FIXED.add(ref)


def rot_xy(dx, dy, rot):
    """KiCad 회전(화면에서 반시계, y 아래)으로 국소 좌표를 돌린다."""
    a = math.radians(rot)
    return dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)


def part(ref):
    return next(p for p in parts if p[0] == ref)


def L(ic, dx, dy):
    """IC 국소 좌표(IC 회전 0 기준) → 절대 좌표."""
    _, _, _, _, x, y, r = part(ic)
    ox, oy = rot_xy(dx, dy, r)
    return x + ox, y + oy


def PL(ref, fp, val, pads, ic, dx, dy, drot=0, fixed=True):
    """IC 국소 좌표에 부품을 둔다(IC를 돌리면 묶음째 돈다)."""
    x, y = L(ic, dx, dy)
    P(ref, fp, val, pads, x, y, part(ic)[6] + drot, fixed)


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def lrect(ic, x0, y0, x1, y1):
    return [L(ic, x, y) for x, y in rect(x0, y0, x1, y1)]


# ================= 전원 입력 · 역접속 · 스위치 · 릴레이 =================
# 배터리 → Q1(역접속) → Q4(스위치) → VBAT_P ─┬─ 벌크 1000µF×2 · 배터리 감지(100k/10k)
#                                          ├─ [비상정지 릴레이 K1] → VM(모터 드라이버)
#                                          └─ D3 → 홀드업 470µF → 5V 벅 → 3V3 LDO
P("J1", ("TerminalBlock", "TerminalBlock_bornier-2_P5.08mm"), "BAT 7.4~12.6V", {1: "VBAT_IN", 2: "GND"}, 4.2, 61.5, 90, True)
# 전원 스위치 = Q1·Q4 소스 맞댄 P-FET 쌍. J8(외부 잠금 스위치)은 게이트만 GND로 당긴다.
P("Q1", ("Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm"), "AO4407A",
  {1: "VBAT_R", 2: "VBAT_R", 3: "VBAT_R", 4: "Q_G", 5: "VBAT_IN", 6: "VBAT_IN", 7: "VBAT_IN", 8: "VBAT_IN"}, 12.1, 57.5, 180, True)
P("Q4", ("Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm"), "AO4407A",
  {1: "VBAT_R", 2: "VBAT_R", 3: "VBAT_R", 4: "Q_G", 5: "VBAT_P", 6: "VBAT_P", 7: "VBAT_P", 8: "VBAT_P"}, 20.1, 57.5, 0, True)
# 게이트: R1 22k(소스) / R68 10k(스위치) → Vgs = −0.69·Vbat (7.4V → −5.1V, 12.6V → −8.7V, 24V는 D1이 −12V로 묶음)
P("R1", R06, "22k", {1: "VBAT_R", 2: "Q_G"}, 18.2, 62.9, 0, True)
P("R68", R06, "10k", {1: "Q_G", 2: "PWR_SW"}, 19.6, 61.05, 0, True)
P("D1", SOD123, "BZT52C12", {1: "VBAT_R", 2: "Q_G"}, 14.0, 62.6, 180, True)     # Vgs 클램프(AO4407A Vgs ±25V)
# 소프트 스타트(돌입 전류 ≈2.5mF×0.8V/ms ≈2A, ≈15ms): Q4 게이트-드레인 1µF를 **켤 때만** 다이오드로 건다.
# 끌 때는 D9가 막고 R71(100k)만 남아 밀러 효과가 거의 없다 → 부하 중에 꺼도 FET가 선형 구간에 머물지 않는다.
P("C15", C08, "1u 50V X7R", {1: "SS", 2: "VBAT_P"}, 21.6, 64.6, 0, True)
P("D9", SOD123, "1N4148W", {1: "Q_G", 2: "SS"}, 19.9, 67.4, 0, True)          # 1=K(게이트) 2=A(SS)
P("R71", R06, "100k", {1: "SS", 2: "Q_G"}, 23.1, 67.6, 270, True)   # 1번(SS) 위
P("J8", H254(2), "POWER SW", {1: "PWR_SW", 2: "GND"}, 2.0, 22.4, 0, True)        # 잠금형 스위치. 빠지면 꺼진다
P("C1", ("Capacitor_THT", "CP_Radial_D12.5mm_P5.00mm"), "1000u 35V", {1: "VBAT_P", 2: "GND"}, 26.0, 47.4, 0, True)
P("C14", ("Capacitor_THT", "CP_Radial_D12.5mm_P5.00mm"), "1000u 35V", {1: "VBAT_P", 2: "GND"}, 17.5, 47.4, 180, True)
# 비상정지: 5V → F1(PTC) → J6(외부 NC 버튼, 없으면 단락) → 릴레이 코일. 버튼이 코일을 끊으면 접점이 VM을 끊는다(MCU 무관).
P("K1", ("Relay_THT", "Relay_SPDT_SANYOU_SRD_Series_Form_C"), "SRD-05VDC-SL-C",
  {1: "VBAT_P", 2: "COIL", 5: "GND", 3: "VM", 4: None}, 25.5, 61.95, 0, True)    # 1=COM 3=NO 4=NC(안 씀) 2·5=코일
# 코일 역기전력: D7(SS34) + D8(12V 제너) 음극끼리 직렬 → 코일 쪽이 −12.5V까지 내려가 접점이 빨리 떨어진다(아크 짧게).
# PB13은 R60 10k 너머라 주입 전류 (12.5−0.4)/10k ≈ 1.2mA < F4 FT핀 음주입 한계 5mA.
P("D7", SOD123, "1N4148W", {1: "FLYK", 2: "GND"}, 64.8, 46.3, 0, True)      # 1=K. 코일 71mA라 SOD-123로 충분(SS34는 자리만 먹었다)
P("D8", SOD123, "BZT52C12", {1: "FLYK", 2: "COIL"}, 64.8, 49.0, 0, True)               # 1=K
P("F1", ("Fuse", "Fuse_1206_3216Metric"), "PTC 0.2A hold", {1: "5V", 2: "ESTOP5"}, 2.0, 35.6, 90, True)
P("J6", ("TerminalBlock", "TerminalBlock_bornier-2_P5.08mm"), "E-STOP (NC)", {1: "ESTOP5", 2: "COIL"}, 4.2, 49.4, 90, True)
P("R60", R06, "10k", {1: "COIL", 2: "ESTOP"}, 66.5, 42.0, 90)                  # FT 핀 PB13, 코일 5V를 직렬저항으로
P("R61", R06, "100k", {1: "ESTOP", 2: "GND"}, 68.0, 42.0, 90)
P("D2", ("Diode_SMD", "D_SMB"), "SMBJ15A", {1: "VM", 2: "GND"}, 93.0, 46.0, 180, True)   # K(VM)가 오른쪽 기둥에   # 역기전력. DRV8874 37V 안
# VM 방전(블리더) 2kΩ = 1k×2 직렬(0603 각 ≈40mW @12.6V; 24V면 1206으로)
P("R66", R06, "1k", {1: "VM", 2: "BLD"}, 71.0, 66.0, 90, True)     # 1번(VM)이 아래 띠 안
P("R67", R06, "1k", {1: "BLD", 2: "GND"}, 72.8, 63.6, 0, True)

# ================= 로직 전원: D3 → 470µF 홀드업 → TPS542025(5V) → AMS1117-3.3 =================
P("D3", ("Diode_SMD", "D_SMA"), "SS34", {1: "VLOG", 2: "VBAT_P"}, 19.0, 38.9, 0, True)
P("C2", ("Capacitor_THT", "CP_Radial_D10.0mm_P5.00mm"), "470u 25V", {1: "VLOG", 2: "GND"}, 9.5, 36.0, 90, True)
UX, UY = 30.5, 30.0
P("U2", ("Package_TO_SOT_SMD", "SOT-23-6"), "TPS542025 (5V)",
  {1: "GND", 2: "SW5", 3: "VLOG", 4: "5V", 5: None, 6: "BST5"}, UX, UY, 0, True)   # EN 띄움 = 켜짐(내부 풀업). FB=출력
PL("C32", C06, "100n 50V", {1: "VLOG", 2: "GND"}, "U2", -1.3, 3.3, 270)            # VIN 핀 바로 아래
PL("C3", C12, "10u 25V", {1: "VLOG", 2: "GND"}, "U2", 0.75, 4.1, 270)
PL("C4", C06, "100n", {1: "BST5", 2: "SW5"}, "U2", 2.95, -0.9, 90)
PL("L1", ("Inductor_SMD", "L_Bourns_SRP7028A_7.3x6.6mm"), "15uH SRP7028A-150M", {1: "SW5", 2: "5V"}, "U2", -7.0, 0.0, 180)
PL("C5", C12, "22u 10V", {1: "5V", 2: "GND"}, "U2", -12.9, 0.0, 90)
PL("C6", C12, "22u 10V", {1: "5V", 2: "GND"}, "U2", -9.7, 5.0, 0)
PL("C33", C12, "22u 10V", {1: "5V", 2: "GND"}, "U2", -9.7, -5.0, 0)
DECAPS += [("C32", "U2", 3), ("C3", "U2", 3), ("C4", "U2", 6), ("L1", "U2", 2),
           ("C5", "L1", 2), ("C6", "L1", 2), ("C33", "L1", 2)]   # 출력 캡은 인덕터 출력 패드 기준(FB=그 노드)
P("U3", ("Package_TO_SOT_SMD", "SOT-223-3_TabPin2"), "AMS1117-3.3", {1: "GND", 2: "3V3", 3: "5V"}, 40.5, 45.0, 0, True)
PL("C7", C12, "10u 25V", {1: "5V", 2: "GND"}, "U3", -2.5, 5.6, 0)
PL("C8", C12, "10u 25V", {1: "3V3", 2: "GND"}, "U3", 3.15, -5.0, 0)
PL("C34", ("Capacitor_Tantalum_SMD", "CP_EIA-3528-21_Kemet-B"), "22u 10V tant", {1: "3V3", 2: "GND"}, "U3", 6.4, 0.0, 90)   # ESR → AMS1117 안정
DECAPS += [("C7", "U3", 3), ("C8", "U3", 2), ("C34", "U3", 2)]
P("R2", R06, "1k", {1: "3V3", 2: "LEDP"}, 41.5, 25.6, 0)
P("D4", LED, "PWR 초록", {1: "GND", 2: "LEDP"}, 41.5, 23.9, 0)

# ================= USB 쪽(절연 너머, GNDU) : USB Mini-B → USBLC6 → CH340C → ADuM1201 =================
P("J2", ("Connector_USB", "USB_Mini-B_Wuerth_65100516121_Horizontal"), "USB Mini-B",
  {1: "VBUS", 2: "DM", 3: "DP", 4: None, 5: "GNDU", 6: "GNDU"}, 5.3, 11.8, 270, True)
P("U6", ("Package_TO_SOT_SMD", "SOT-23-6"), "USBLC6-2SC6",
  {1: "DP", 6: "DP", 3: "DM", 4: "DM", 2: "GNDU", 5: "VBUS"}, 12.2, 11.8, 0, True)   # USB ESD(GNDU 쪽)
P("U4", ("Package_SO", "SOIC-16_3.9x9.9mm_P1.27mm"), "CH340C",
  {1: "GNDU", 2: "U_TXC", 3: "U_RXC", 4: "U_V3", 5: "DP", 6: "DM", 7: None, 8: None, 9: None, 10: None,
   11: None, 12: None, 13: None, 14: None, 15: None, 16: "VBUS"}, 19.5, 10.3, 0, True)
PL("C9", C06, "100n", {1: "U_V3", 2: "GNDU"}, "U4", -4.9, -2.7, 90)
PL("C10", C06, "100n", {1: "VBUS", 2: "GNDU"}, "U4", 5.1, -3.3, 90)
PL("C11", C12, "10u 25V", {1: "VBUS", 2: "GNDU"}, "U4", 5.0, -7.7, 0)
DECAPS += [("C9", "U4", 4), ("C10", "U4", 16), ("C11", "U4", 16)]
P("U5", ("Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm"), "ADuM1201ARZ",                # 회전 0: 1~4(USB측)=왼쪽, 5~8(MCU측)=오른쪽
  {1: "ISO_V1", 2: "U_RX", 3: "U_TX", 4: "GNDU", 5: "GND", 6: "MCU_RX", 7: "MCU_TX", 8: "3V3"}, ISO_X, 12.0, 0, True)
PL("C12", C06, "100n", {1: "ISO_V1", 2: "GNDU"}, "U5", -3.0, -4.4, 0)
PL("C13", C06, "100n", {1: "3V3", 2: "GND"}, "U5", 3.0, -4.4, 0)
DECAPS += [("C12", "U5", 1), ("C13", "U5", 8)]
# 젯슨 40핀 UART 직결 대안 — 절연기 **앞**. USB로 쓸 땐 R62~R64(0Ω) 실장 + J7 비움.
P("R62", R06, "0", {1: "U_TXC", 2: "U_TX"}, 24.6, 10.6, 90, True)
P("R63", R06, "0", {1: "U_RX", 2: "U_RXC"}, 24.6, 13.7, 90, True)
P("R64", R06, "0", {1: "VBUS", 2: "ISO_V1"}, 24.6, 16.8, 90, True)
P("J7", ("Connector_PinHeader_2.54mm", "PinHeader_1x04_P2.54mm_Vertical"), "JETSON UART",
  {1: "GNDU", 2: "U_TX", 3: "U_RX", 4: "ISO_V1"}, 6.5, 2.2, 90, True)

# ================= MCU STM32F405RGT6 =================
mcu = {1: "3V3", 2: None, 3: None, 4: None, 5: "OSC_IN", 6: "OSC_OUT", 7: "NRST",
       8: "IPR_A", 9: "IPR_B", 10: "IPR_C", 11: "IPR_D", 12: "GND", 13: "3V3", 14: "ENC_D_A", 15: "ENC_D_B",
       16: "PWM_C1", 17: "PWM_C2", 18: "GND", 19: "3V3", 20: "SPI_CS", 21: "SPI_SCK", 22: "SPI_MISO", 23: "SPI_MOSI",
       24: "VB_SNS", 25: "VM_SNS", 26: "V5_SNS", 27: "LED_FLT", 28: "BOOT1", 29: "IMU_INT", 30: "LED_ST",
       31: "VCAP1", 32: "3V3", 33: "NSLEEP", 34: "ESTOP", 35: "PWM_D1", 36: "PWM_D2",
       37: "PWM_A1", 38: "PWM_A2", 39: "PWM_B1", 40: "PWM_B2", 41: None, 42: "MCU_TX", 43: "MCU_RX",
       44: None, 45: None, 46: "SWDIO", 47: "VCAP2", 48: "3V3", 49: "SWCLK", 50: "ENC_A_A",
       51: "FLT_A", 52: "FLT_B", 53: "FLT_C", 54: "FLT_D", 55: "ENC_A_B", 56: "ENC_B_A", 57: "ENC_B_B",
       58: "ENC_C_A", 59: "ENC_C_B", 60: "BOOT0", 61: "I2C_SCL", 62: "I2C_SDA", 63: "GND", 64: "3V3"}
MX, MY = 55.5, 30.5
P("U1", ("Package_QFP", "LQFP-64_10x10mm_P0.5mm"), "STM32F405RGT6", mcu, MX, MY, 0, True)
PL("C20", C06, "100n", {1: "3V3", 2: "GND"}, "U1", -3.0, 7.6, 0)     # VDD 19
PL("C21", C06, "100n", {1: "3V3", 2: "GND"}, "U1", 5.8, 8.3, 0)      # VDD 32
PL("C22", C06, "100n", {1: "3V3", 2: "GND"}, "U1", 8.0, -4.4, 90)    # VDD 48
PL("C23", C06, "100n", {1: "3V3", 2: "GND"}, "U1", -4.4, -7.7, 0)    # VDD 64
PL("C24", C06, "100n", {1: "3V3", 2: "GND"}, "U1", -8.3, 3.45, 180)     # VDDA 13 (VREF+ 내부 연결)
PL("C31", C06, "2.2u", {1: "3V3", 2: "GND"}, "U1", -8.3, 5.1, 180)     # VDDA 13 저주파
PL("C27", C06, "4.7u", {1: "3V3", 2: "GND"}, "U1", -7.6, -7.0, 90)   # VBAT 1 + 벌크
PL("C25", C06, "2.2u", {1: "VCAP1", 2: "GND"}, "U1", 2.6, 7.6, 0)
PL("C26", C06, "2.2u", {1: "VCAP2", 2: "GND"}, "U1", 8.0, -1.3, 90)
# 크리스털: 3번 패드 = OSC_IN(핀5), 1번 = OSC_OUT(핀6) — 둘은 대칭이라 가까운 쪽으로 물렸다
PL("Y1", ("Crystal", "Crystal_SMD_3225-4Pin_3.2x2.5mm"), "8MHz 3225 12pF", {1: "OSC_OUT", 2: "GND", 3: "OSC_IN", 4: "GND"}, "U1", -8.6, -1.5, 90)
PL("C28", C06, "15p", {1: "OSC_IN", 2: "GND"}, "U1", -11.8, -2.6, 90)   # CL 12pF: 2×(12−4 stray) ≈ 15pF
PL("C29", C06, "15p", {1: "OSC_OUT", 2: "GND"}, "U1", -8.4, 1.6, 0)
DECAPS += [("C20", "U1", 19), ("C21", "U1", 32), ("C22", "U1", 48), ("C23", "U1", 64), ("C24", "U1", 13),
           ("C31", "U1", 13), ("C27", "U1", 1), ("C25", "U1", 31), ("C26", "U1", 47),
           ("Y1", "U1", 5), ("Y1", "U1", 6), ("C28", "Y1", 3), ("C29", "Y1", 1)]
P("C30", C06, "100n", {1: "NRST", 2: "GND"}, 42.5, 33.0, 90)
P("R5", R06, "10k", {1: "BOOT0", 2: "GND"}, 52.3, 21.0, 90)
P("R6", R06, "10k", {1: "BOOT1", 2: "GND"}, 60.6, 42.4, 90)
P("SW1", ("Button_Switch_THT", "SW_PUSH_6mm"), "RESET", {1: "NRST", 2: "GND"}, 31.8, 19.8, 0, True)   # NRST(PC 핀 7)가 왼쪽이라 왼쪽 위
P("SW2", ("Button_Switch_THT", "SW_PUSH_6mm"), "BOOT0", {1: "BOOT0", 2: "3V3"}, 81.2, 39.5, 0, True)
P("R7", R06, "1k", {1: "LED_ST", 2: "LEDS"}, 74.5, 47.6, 0)
P("D5", LED, "상태 초록", {1: "GND", 2: "LEDS"}, 71.4, 47.6, 0)
P("R8", R06, "1k", {1: "LED_FLT", 2: "LEDF"}, 74.5, 49.4, 0)
P("D6", LED, "고장 빨강", {1: "GND", 2: "LEDF"}, 71.4, 49.4, 0)
# 계통 전압 감지 — 분압 + 100nF(ADC 샘플링 전하 공급, PWM 잡음 거름)
P("R9", R06, "100k", {1: "VBAT_P", 2: "VB_SNS"}, 24.6, 39.6, 0, True)    # 1번이 VBAT_P 면 안(전원 넷은 면으로만 잇는다)
P("R10", R06, "10k", {1: "VB_SNS", 2: "GND"}, 51.0, 42.6, 90)
P("C35", C06, "100n", {1: "VB_SNS", 2: "GND"}, 52.8, 42.6, 90)
P("R11", R06, "100k", {1: "VM", 2: "VM_SNS"}, 95.6, 52.0, 180, True)   # 1번이 오른쪽 VM 기둥 안
P("R12", R06, "10k", {1: "VM_SNS", 2: "GND"}, 56.4, 42.6, 90)
P("C36", C06, "100n", {1: "VM_SNS", 2: "GND"}, 58.2, 42.6, 90)
P("R69", R06, "10k", {1: "5V", 2: "V5_SNS"}, 49.2, 46.0, 90)
P("R70", R06, "10k", {1: "V5_SNS", 2: "GND"}, 51.0, 46.0, 90)
P("C37", C06, "100n", {1: "V5_SNS", 2: "GND"}, 52.8, 46.0, 90)

# ================= 모터 드라이버 4 × DRV8874 (PWM 모드) + 엔코더 커넥터 =================
# 국소 좌표(IC 회전 0): 1~8 왼쪽(로직), 9~16 오른쪽(VM 쪽), 출력(8·10)은 아래 끝. 커넥터는 출력 끝 쪽(판 안쪽)에 둔다.
# 위 줄 A·B = 회전 0(출력이 판 안쪽 아래), 아래 줄 C·D = 회전 180. VM 면은 위·아래 가장자리 띠 + 오른쪽 기둥으로 묶는다.
# VREF 1.06V → 1k IPROPI에서 ≈2.3A 제한. 위 줄(A·B)·아래 줄(C·D)에 분압을 하나씩 — 한 넷을 판 네 귀퉁이로 끌면 배선이 막혔다(11/09)
P("R13", R06, "100k", {1: "3V3", 2: "VREF"}, 62.6, 20.6, 90)
P("R14", R06, "47k", {1: "VREF", 2: "GND"}, 64.1, 20.6, 90)
P("R75", R06, "100k", {1: "3V3", 2: "VREF_CD"}, 68.0, 54.5, 90)
P("R76", R06, "47k", {1: "VREF_CD", 2: "GND"}, 69.5, 54.5, 90)
P("R15", R06, "100k", {1: "NSLEEP", 2: "GND"}, 63.8, 41.9, 90)              # 리셋·부트로더 중엔 드라이버가 잔다

drv = {"A": (44.0, 9.0, 0), "B": (80.0, 9.0, 0), "C": (54.5, 61.0, 180), "D": (86.0, 61.0, 180)}
enc_pins = {"A": ("ENC_A_A", "ENC_A_B"), "B": ("ENC_B_A", "ENC_B_B"), "C": ("ENC_C_A", "ENC_C_B"), "D": ("ENC_D_A", "ENC_D_B")}
for n, w in enumerate("ABCD"):
    x, y, r = drv[w]
    u = f"U1{n}"
    P(u, HTSSOP, "DRV8874PWPR",
      {1: f"PWM_{w}1", 2: f"PWM_{w}2", 3: "NSLEEP", 4: f"FLT_{w}", 5: "VREF" if w in "AB" else "VREF_CD", 6: f"IPR_{w}", 7: "GND",
       8: f"M{w}+", 9: "GND", 10: f"M{w}-", 11: "VM", 12: f"VCP_{w}", 13: f"CPH_{w}", 14: f"CPL_{w}",
       15: "GND", 16: f"PMODE_{w}", 17: "GND"}, x, y, r, True)
    c = 40 + n * 4
    # 오른쪽 열(9 PGND·10 OUT2·11 VM·12 VCP·13 CPH·14 CPL·15 GND·16 PMODE): 12~14는 오른쪽으로 나가 계단식으로 위로
    # (14가 안쪽 x≈3.95, 13 x≈4.5, 12 x≈5.75) → 그 위 C41(CPL|CPH)·C40(VCP/VM). VM은 핀 11에서 오른쪽 띠 → 탭으로 위 띠.
    PL(f"C{c + 2}", C06, "100n 50V", {1: "VM", 2: "GND"}, u, 6.0, 1.9, 270)           # 1번(VM)이 위 = VM 띠 위
    PL(f"C{c + 3}", C12, "10u 25V", {1: "VM", 2: "GND"}, u, 8.6, 1.3, 270)            # 벌크 — 핀 11에서 ≈4mm(3mm 검사에서 뺌)
    PL(f"C{c}", C06, "100n 50V", {1: f"VCP_{w}", 2: "VM"}, u, 6.5, -2.45, 90)         # 1번(VCP)이 아래
    PL(f"C{c + 1}", C06, "22n 50V", {1: f"CPL_{w}", 2: f"CPH_{w}"}, u, 4.15, -3.6, 0) # 1번(CPL) 왼쪽
    PL(f"R{20 + n * 4}", R06, "1k", {1: f"IPR_{w}", 2: "GND"}, u, -6.6, 0.6, 90)    # IPROPI 0.45V/A. 핀 끝에서 2.5mm 띄워 1~5번 핀이 빠져나갈 길
    PL(f"R{21 + n * 4}", R06, "10k", {1: "3V3", 2: f"FLT_{w}"}, u, -6.6, -2.6, 90)   # nFAULT 풀업
    PL(("R16", "R72", "R73", "R74")[n], R06, "0", {1: "3V3", 2: f"PMODE_{w}"}, u, 0.5, -4.2, 0)   # PMODE high = PWM(IN1/IN2). 칩마다 0Ω(옮길 수 있게)
    DECAPS += [(f"C{c + 2}", u, 11), (f"C{c}", u, 12), (f"C{c + 1}", u, 13), (f"R{20 + n * 4}", u, 6)]
    # 커넥터 = 기존 쉴드와 같은 6핀 순서: M− · GND · ENC_B · ENC_A · 5V · M+  (1번이 국소 x=7.9, 6번이 x=−2.1)
    j = f"J{10 + n}"
    PL(j, PH6, f"MOTOR {w}", {1: f"M{w}-", 2: "GND", 3: f"E{w}B", 4: f"E{w}A", 5: "5V", 6: f"M{w}+"}, u, 7.9, 7.0, 180)
    ea, eb = enc_pins[w]
    # 엔코더: 모터 안 10k(5V) 오픈컬렉터 → 1k 직렬 → FT 핀, 10k 풀업(3V3). 커넥터 바로 안쪽에 한 줄로
    PL(f"R{40 + n * 4}", R06, "1k", {1: f"E{w}A", 2: ea}, u, 1.9, 10.9, 90)
    PL(f"R{41 + n * 4}", R06, "1k", {1: f"E{w}B", 2: eb}, u, 3.9, 10.9, 90)
    PL(f"R{42 + n * 4}", R06, "10k", {1: "3V3", 2: ea}, u, -0.1, 10.9, 90)
    PL(f"R{43 + n * 4}", R06, "10k", {1: "3V3", 2: eb}, u, 5.85, 10.9, 90)
    # 모터 출력 1.0mm(패드에서 0.75mm까지만 0.4mm 넥) + 패드→노출패드 접지 짧은 선
    PREROUTES.append((f"M{w}+", 0.4, [L(u, -2.86, 2.27), L(u, -4.4, 2.27)]))
    PREROUTES.append((f"M{w}+", 1.0, [L(u, -4.4, 2.27), L(u, -4.4, 5.2), L(u, -2.1, 5.2), L(u, -2.1, 7.0)]))
    PREROUTES.append((f"M{w}-", 0.4, [L(u, 2.86, 1.62), L(u, 4.0, 1.62), L(u, 4.6, 2.5)]))   # 넥 ≈1.4mm(PGND 핀 비켜 사선)
    PREROUTES.append((f"M{w}-", 1.0, [L(u, 4.6, 2.5), L(u, 4.6, 5.2), L(u, 7.9, 5.2), L(u, 7.9, 7.0)]))
    for px, py in ((2.86, 2.27), (2.86, -1.62), (-2.86, 1.62)):          # PGND 9 · GND 15 · IMODE 7 → EP
        PREROUTES.append(("GND", 0.4, [L(u, px, py), L(u, math.copysign(1.2, px), py)]))
    # VM 면: 핀 11 + C42/C43 VM 패드 → 탭 → 가장자리 띠. C40의 VM 패드도 띠에 붙인다.
    POWER_ZONES["VM"] += [lrect(u, 3.3, 0.75, 7.6, 1.2),       # 핀 11 입구(≈1.5mm까지 0.45mm 넥 — 핀 12·10과 0.22mm)
                          lrect(u, 5.3, 0.75, 7.6, 1.95),      # 넥 지나 1.2mm로 — C42 VM 패드 포함
                          lrect(u, 7.5, -6.0, 10.5, 1.7),      # 탭(C43 VM 패드 포함) → 가장자리 띠
                          lrect(u, 6.0, -6.0, 7.6, -2.75)]     # C40의 VM 패드

POWER_ZONES["VM"] += [rect(31.5, 0.5, 99.5, 3.4),      # 위 띠(A·B)
                      rect(95.6, 0.5, 99.5, 69.5),     # 오른쪽 기둥
                      rect(37.8, 66.4, 99.5, 69.5)]    # 아래 띠(K1 NO → C·D)
# 게이트·소프트 스타트 망은 전원 면 사이 좁은 틈이라 손으로 깐다(자동배선이 Q1.4·D1·C15를 못 이었다, 11/09)
PREROUTES += [("Q_G", 0.25, [(12.35, 62.6), (12.35, 55.59), (14.58, 55.59)]),           # D1 → Q1 게이트(Q1 몸통 밑)
              ("Q_G", 0.25, [(14.58, 55.59), (14.58, 54.55), (20.1, 54.55), (20.1, 59.41), (17.62, 59.41)]),   # Q1 → Q4 게이트(Q4 몸통 밑)
              ("Q_G", 0.25, [(17.62, 59.41), (18.82, 60.6), (18.82, 61.05), (18.98, 62.9), (18.25, 67.4)]),  # R68·R1·D9
              ("Q_G", 0.25, [(18.25, 67.4), (18.25, 68.6), (22.8, 68.6), (23.1, 68.38)]),  # → R71
              ("SS", 0.25, [(20.65, 64.6), (21.55, 66.3), (21.55, 67.4), (23.1, 66.82)])]  # C15 → D9 → R71
POWER_ZONES["VBAT_IN"] += [[(2.4, 63.4), (2.4, 59.2), (8.3, 59.2), (8.3, 54.9), (11.1, 54.9), (11.1, 63.4)]]
POWER_ZONES["VBAT_R"] += [[(13.4, 56.2), (15.9, 56.2), (15.9, 55.0), (18.8, 55.0), (18.8, 58.8), (16.3, 58.8), (16.3, 60.0), (13.4, 60.0)],
                          rect(14.8, 59.0, 16.4, 63.4), rect(14.8, 62.2, 17.95, 63.4)]   # D1·R1의 VBAT_R 패드까지
POWER_ZONES["VBAT_P"] += [[(16.0, 46.2), (27.4, 46.2), (27.4, 48.6), (24.4, 48.6), (24.4, 60.2), (27.3, 60.2),
                           (27.3, 63.7), (21.2, 63.7), (21.2, 48.6), (16.0, 48.6)],
                          rect(20.2, 38.0, 24.0, 46.5),     # D3 애노드·R9(배터리 감지)까지
                          rect(21.9, 63.5, 23.3, 65.6)]     # C15(소프트 스타트) VBAT_P 패드

# ================= 헤더: SWD · I2C(IMU 이전용) · SPI =================
P("J3", H254(5), "SWD", {1: "3V3", 2: "SWDIO", 3: "SWCLK", 4: "NRST", 5: "GND"}, 89.0, 23.5, 0, True)
P("J4", H254(4), "I2C (MPU6050)", {1: "3V3", 2: "GND", 3: "I2C_SCL", 4: "I2C_SDA"}, 85.0, 23.5, 0, True)
P("R17", R06, "10k", {1: "3V3", 2: "I2C_SCL"}, 82.3, 24.1, 90)
P("R18", R06, "10k", {1: "3V3", 2: "I2C_SDA"}, 82.3, 27.3, 90)
P("J5", H254(7), "SPI IMU", {1: "3V3", 2: "GND", 3: "SPI_SCK", 4: "SPI_MISO", 5: "SPI_MOSI", 6: "SPI_CS", 7: "IMU_INT"}, 93.0, 23.5, 0, True)

UNO_HOLES = [(14.0, 2.54), (15.24, 50.8), (66.04, 7.62), (66.04, 35.56)]   # Uno 도면(왼쪽아래 원점) → 아래에서 y 뒤집음
FULL_PADS = {"J1", "C1", "C14", "D2", "K1", "Q1", "Q4"} | {f"U1{n}" for n in range(4)} | {f"C{40 + n * 4 + 2}" for n in range(4)} \
    | {f"C{40 + n * 4 + 3}" for n in range(4)}   # 이 부품들의 전원·접지 패드는 동박에 통째로(방열 스포크 없이) 붙인다


def build():
    b = pcbnew.BOARD()
    ds = b.GetDesignSettings()   # 2층 일반 공정: 선 ≥0.127, 드릴 ≥0.3
    ds.m_TrackMinWidth = pcbnew.FromMM(0.127); ds.m_MinThroughDrill = pcbnew.FromMM(0.3)
    ds.m_ViasMinSize = pcbnew.FromMM(0.6); ds.m_HoleClearance = pcbnew.FromMM(0.25); ds.m_MinClearance = pcbnew.FromMM(0.15)
    nets = {}
    def net(name):
        if name not in nets:
            nets[name] = pcbnew.NETINFO_ITEM(b, name); b.Add(nets[name])
        return nets[name]
    for ref, (lib, fpn), val, pads, x, y, rot in parts:
        fp = pcbnew.FootprintLoad(f"{FP}/{lib}.pretty", fpn)
        if fp is None:
            sys.exit(f"풋프린트 없음: {lib}:{fpn}")
        fp.SetReference(ref); fp.SetValue(val)
        fp.Reference().SetTextSize(pcbnew.VECTOR2I_MM(0.8, 0.8)); fp.Reference().SetTextThickness(pcbnew.FromMM(0.15))
        names = {p.GetNumber() for p in fp.Pads() if p.GetNumber()}
        missing = set(pads) - names
        if missing:
            sys.exit(f"{ref}: 패드 {missing} 가 {fpn}에 없다")
        if fpn == HTSSOP[1]:   # 방열 비아 0.3/0.6mm × 12 (노출 패드 번호 17 = GND)
            for vx, vy in THERMAL_VIAS:
                v = pcbnew.PAD(fp); v.SetNumber("17"); v.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
                v.SetShape(pcbnew.PAD_SHAPE_CIRCLE); v.SetSize(pcbnew.VECTOR2I_MM(0.6, 0.6)); v.SetDrillSize(pcbnew.VECTOR2I_MM(0.3, 0.3))
                ls = pcbnew.LSET(); ls.AddLayer(pcbnew.F_Cu); ls.AddLayer(pcbnew.B_Cu); v.SetLayerSet(ls)
                v.SetPosition(pcbnew.VECTOR2I_MM(vx, vy)); fp.Add(v)
        for p in fp.Pads():
            n = pads.get(p.GetNumber())
            if n:
                p.SetNet(net(n))
                if ref in FULL_PADS and n in ("GND", "VM", "VBAT_IN", "VBAT_R", "VBAT_P"):
                    p.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
        b.Add(fp)
        fp.SetPosition(pcbnew.VECTOR2I_MM(x, y))
        fp.SetOrientationDegrees(rot)
    for i, (hx, hy) in enumerate(UNO_HOLES):
        h = pcbnew.FootprintLoad(f"{FP}/MountingHole.pretty", "MountingHole_3.2mm_M3")
        h.SetReference(f"H{i + 1}"); b.Add(h); h.SetPosition(pcbnew.VECTOR2I_MM(hx, H - hy))
        FIXED.add(f"H{i + 1}")
    for (x0, y0), (x1, y1) in [((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))]:
        s = pcbnew.PCB_SHAPE(b); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetLayer(pcbnew.Edge_Cuts)
        s.SetStart(pcbnew.VECTOR2I_MM(x0, y0)); s.SetEnd(pcbnew.VECTOR2I_MM(x1, y1)); s.SetWidth(pcbnew.FromMM(0.1))
        b.Add(s)
    for netname, wd, pts in PREROUTES:   # 미리 깐 선(F.Cu) — route.py가 Freerouting에 장애물로 넘긴다
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            t = pcbnew.PCB_TRACK(b); t.SetStart(pcbnew.VECTOR2I_MM(x0, y0)); t.SetEnd(pcbnew.VECTOR2I_MM(x1, y1))
            t.SetWidth(pcbnew.FromMM(wd)); t.SetLayer(pcbnew.F_Cu); t.SetNet(net(netname)); b.Add(t)
    return b


def iso_line(b):
    """절연 경계(실크) — ADuM1201 한가운데 세로선 + USB 칸 아래 가로선."""
    for (x0, y0), (x1, y1) in [((ISO_X, 0.5), (ISO_X, ISO_Y)), ((0.5, ISO_Y), (ISO_X, ISO_Y))]:
        s = pcbnew.PCB_SHAPE(b); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetLayer(pcbnew.F_SilkS)
        s.SetStart(pcbnew.VECTOR2I_MM(x0, y0)); s.SetEnd(pcbnew.VECTOR2I_MM(x1, y1)); s.SetWidth(pcbnew.FromMM(0.2)); b.Add(s)


def crt(fp, gap=0.25):
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
    bb.Inflate(pcbnew.FromMM(gap)); return bb


def overlaps(b):
    """F.CrtYd 사각형끼리 겹치는 쌍 + 판 밖으로 나간 부품. 0이어야 배치 끝."""
    boxes = [(fp.GetReference(), crt(fp, 0)) for fp in b.GetFootprints()]
    bad = []
    for i in range(len(boxes)):
        r, a = boxes[i]
        if a.GetLeft() < 0 or a.GetTop() < 0 or a.GetRight() > pcbnew.FromMM(W) or a.GetBottom() > pcbnew.FromMM(H):
            if not r.startswith(("J1", "J2", "J6", "J8", "H")):          # 가장자리 커넥터는 걸치는 게 정상
                bad.append(f"{r} 판 밖")
        for j in range(i + 1, len(boxes)):
            if a.Intersects(boxes[j][1]):
                bad.append(f"{r}↔{boxes[j][0]}")
    return bad


def settle(b, max_r=3.0):
    """FIXED가 아닌 작은 부품만, 적힌 자리에서 max_r mm 안의 가장 가까운 빈자리로 옮긴다(0/90° 둘 다).
    ponytail: 탐욕 나선 탐색 — 못 찾으면 그 자리에 두고 이름을 찍는다(겹침 검사가 판을 거절한다)."""
    fps = sorted(b.GetFootprints(), key=lambda f: (f.GetReference() not in FIXED, -crt(f).GetArea()))
    placed = []
    def ok(f):
        a = crt(f, 0.0 if f.GetReference() in FIXED else 0.2)
        if a.GetLeft() < 0 or a.GetTop() < 0 or a.GetRight() > pcbnew.FromMM(W) or a.GetBottom() > pcbnew.FromMM(H):
            return False
        return not any(a.Intersects(p) for p in placed)
    for f in fps:
        p0, r0 = f.GetPosition(), f.GetOrientationDegrees()
        found = f.GetReference() in FIXED or ok(f)
        rad = 0.0
        while not found and rad < max_r:
            rad += 0.25
            k_n = max(8, int(rad * 8))
            for k in range(k_n):
                t = 2 * math.pi * k / k_n
                for rot in (r0, r0 + 90):
                    f.SetOrientationDegrees(rot)
                    f.SetPosition(pcbnew.VECTOR2I(p0.x + pcbnew.FromMM(rad * math.cos(t)), p0.y + pcbnew.FromMM(rad * math.sin(t))))
                    if ok(f):
                        found = True; break
                if found: break
        if not found:
            f.SetPosition(p0); f.SetOrientationDegrees(r0); print("자리 못 찾음:", f.GetReference())
        placed.append(crt(f, 0.0))


def _dist(sa, sb):
    """두 동박 모양 사이 최소 거리(mm) — Collide 이분 탐색."""
    if sa.Collide(sb, 0): return 0.0
    lo, hi = 0.0, 50.0
    if not sa.Collide(sb, pcbnew.FromMM(hi)): return hi
    for _ in range(20):
        mid = (lo + hi) / 2
        if sa.Collide(sb, pcbnew.FromMM(mid)): hi = mid
        else: lo = mid
    return hi


def decap_check(b):
    """DECAPS의 부품 패드(IC 핀과 같은 넷이면 그 패드, 아니면 가장 가까운 패드) ↔ IC 핀 패드 가장자리 거리 ≤ DECAP_MAX."""
    bad, worst = [], 0.0
    for ref, ic, pin in DECAPS:
        ips = [p for p in b.FindFootprintByReference(ic).Pads() if p.GetNumber() == str(pin)]   # SOT-223 탭처럼 같은 번호가 여럿
        cps = list(b.FindFootprintByReference(ref).Pads())
        same = [p for p in cps if p.GetNetname() and p.GetNetname() == ips[0].GetNetname()]
        d = min(_dist(p.GetEffectiveShape(pcbnew.F_Cu), ip.GetEffectiveShape(pcbnew.F_Cu)) for p in (same or cps) for ip in ips)
        worst = max(worst, d)
        if d > DECAP_MAX:
            bad.append(f"{ref}→{ic}.{pin} {d:.2f}mm")
    return bad, worst


USB_NETS = {"GNDU", "VBUS", "U_TXC", "U_RXC", "ISO_V1", "DP", "DM", "U_V3", "U_TX", "U_RX"}


def iso_check(b, copper=False):
    """USB 칸(x<ISO_X, y<ISO_Y)엔 USB 쪽 부품만, USB 쪽 부품은 칸 안에만(U5는 경계 위).
    copper=True면 USB 넷 동박(패드·선·비아·동박면) ↔ 다른 넷 동박 최소 거리도 잰다(≥ ISO_GAP)."""
    bad = []
    for f in b.GetFootprints():
        r = f.GetReference()
        if r == "U5" or r.startswith("H"): continue
        bb = f.GetBoundingBox(False)
        inside = pcbnew.ToMM(bb.GetLeft()) < ISO_X and pcbnew.ToMM(bb.GetTop()) < ISO_Y
        usb = bool({p.GetNetname() for p in f.Pads()} & USB_NETS)
        if inside != usb: bad.append(f"{r} {'칸 침범' if inside else 'USB 부품이 칸 밖'}")
    if not copper:
        return bad, None
    worst = (99.0, "")
    for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
        A, B = [], []
        for t in b.GetTracks():
            if t.IsOnLayer(lay):
                (A if t.GetNetname() in USB_NETS else B).append((t.GetEffectiveShape(lay), f"{t.GetNetname()} 선"))
        for p in b.GetPads():
            if p.IsOnLayer(lay) and p.GetNetname():
                (A if p.GetNetname() in USB_NETS else B).append((p.GetEffectiveShape(lay), f"{p.GetParentFootprint().GetReference()}.{p.GetNumber()}"))
        for z in b.Zones():
            if z.GetIsRuleArea() or not z.IsOnLayer(lay): continue
            fl = z.GetFilledPolysList(lay)
            for i in range(fl.OutlineCount()):
                (A if z.GetNetname() in USB_NETS else B).append((pcbnew.SHAPE_POLY_SET(fl.Outline(i)), f"{z.GetNetname()} 동박"))
        for sa, na in A:
            ba = sa.BBox(); ba.Inflate(pcbnew.FromMM(ISO_GAP + 0.5))
            for sb, nb in B:
                if not ba.Intersects(sb.BBox()): continue
                if sa.Collide(sb, pcbnew.FromMM(ISO_GAP) - 1):
                    d = _dist(sa, sb)
                    worst = min(worst, (d, f"{na}↔{nb}"))
                    bad.append(f"{pcbnew.LayerName(lay) if hasattr(pcbnew, 'LayerName') else lay}: {na}↔{nb} {d:.2f}mm")
    if worst[0] == 99.0:
        worst = (ISO_GAP, "≥ 기준")
    return bad, worst


if __name__ == "__main__":
    b = build()
    area = sum(crt(f, 0).GetArea() for f in b.GetFootprints()) / 1e12
    print(f"코트야드 면적 합 {area:.0f} mm² / 판 {W * H:.0f} mm² ({100 * area / (W * H):.0f}%)")
    settle(b)
    iso_line(b)
    b.BuildConnectivity()
    bad = overlaps(b)
    pcbnew.SaveBoard(OUT, b)
    print(f"부품 {len(parts) + len(UNO_HOLES)}개, 넷 {b.GetNetCount()}개 → {OUT}")
    print("겹침/이탈:", len(bad)); print("\n".join(bad))
    iso, _ = iso_check(b)
    print("절연 칸 위반:", len(iso)); print("\n".join(iso))
    dbad, worst = decap_check(b)
    print(f"디커플링 거리 위반(>{DECAP_MAX}mm): {len(dbad)} (최악 {worst:.2f}mm)"); print("\n".join(dbad))
    if bad or iso or dbad:
        sys.exit(1)

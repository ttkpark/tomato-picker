# 메카넘 STM32 주행보드 — 부품·넷·배치를 한 곳에서 정한다 (KiCad 9 pcbnew).
# 실행: "C:/Program Files/KiCad/9.0/bin/python.exe" gen_pcb.py   → mecanum_stm32.kicad_pcb
#
# 회로도(.kicad_sch)는 없다. 이 파일이 회로도다 — 넷을 패드에 직접 물려 래츠네스트가 뜨고,
# 라우팅은 pcbnew(또는 Freerouting)에서 한다. 부품을 고치면 여기 고치고 다시 돌린다.
# 펌웨어 핀표 = firmware/mecanum_stm32/README.md 와 1:1 (어긋나면 둘 중 하나가 틀렸다).
# 고정구멍 = 아두이노 우노와 같은 자리(왼쪽 아래 기준, M3 4개) — 지금 Uno 스탠드오프에 그대로 앉는다.
# 판은 오른쪽·위로 Uno보다 크다(케이스 트레이에 들어가는지 실측 필요).
import os, sys
import pcbnew

FP = r"C:/Program Files/KiCad/9.0/share/kicad/footprints"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mecanum_stm32.kicad_pcb")
W, H = 100.0, 70.0         # Uno(68.58×53.34)보다 크다 — 2층 배선 밀도 때문. 명세서 v0.1 반영(릴레이·벌크)으로 90×64에서 키움
UW, UH = 68.58, 53.34      # 아래 좌표는 Uno 판 기준 힌트 → 비율로 늘려 쓴다
ISO_Y = 30.0               # USB 절연 칸(GNDU) 아래 경계 mm — 이 칸엔 GNDU 쪽 부품만(route.py가 동박을 여기서 가른다)

R06, C06, C12, LED = ("Resistor_SMD", "R_0603_1608Metric"), ("Capacitor_SMD", "C_0603_1608Metric"), \
    ("Capacitor_SMD", "C_1206_3216Metric"), ("LED_SMD", "LED_0603_1608Metric")
HTSSOP = ("Package_SO", "HTSSOP-16-1EP_4.4x5mm_P0.65mm_EP3.4x5mm_Mask2.46x2.31mm_ThermalVias")
PH6 = ("Connector_JST", "JST_PH_B6B-PH-K_1x06_P2.00mm_Vertical")
H254 = lambda n: ("Connector_PinHeader_2.54mm", f"PinHeader_1x{n:02d}_P2.54mm_Vertical")

parts = []   # (ref, (lib, fp), value, {pad: net}, x, y, rot)
def P(ref, fp, val, pads, x, y, rot=0):
    parts.append((ref, fp, val, {str(k): v for k, v in pads.items()}, x, y, rot))

# ---------------- 전원 (명세서 v0.1 반영) ----------------
# 배터리 → 역접속 P-FET → 전원 스위치 P-FET → VBAT_P ─┬─ 벌크 1000µF×2 · 배터리 전압 감지(100k/10k)
#                                 ├─ [비상정지 릴레이 K1] → VM(모터 드라이버)   ← 버튼이 코일을 끊으면 VM이 물리적으로 끊긴다
#                                 └─ 다이오드 → 홀드업 → 5V 벅 → 3V3 LDO       ← 로직은 릴레이와 무관하게 산다
P("J1", ("TerminalBlock", "TerminalBlock_bornier-2_P5.08mm"), "BAT 7.4~12.6V", {1: "VBAT_IN", 2: "GND"}, 4.5, 44.0, 90)
# 전원 스위치 = Q1·Q4 소스 맞댄 P-FET 쌍. 바깥 스위치(J8)는 게이트만 GND로 당긴다 — 배터리 전류가 스위치를
# 지나지 않고(작은 패널 스위치로 충분), 꺼지면 대기전류 0. FET 하나로는 몸체다이오드로 새서 못 끈다.
# 역접속이면 Q1 몸체다이오드가 막고 게이트(GND=+)가 꺼진 쪽이라 둘 다 안 켜진다.
P("Q1", ("Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm"), "AO4407A",            # D=배터리, S=공통점: 역접속이면 몸체다이오드가 막는다
  {1: "VBAT_R", 2: "VBAT_R", 3: "VBAT_R", 4: "Q_G", 5: "VBAT_IN", 6: "VBAT_IN", 7: "VBAT_IN", 8: "VBAT_IN"}, 12.0, 47.0, 0)
P("Q4", ("Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm"), "AO4407A",            # S=공통점, D=VBAT_P: 꺼지면 몸체다이오드가 배터리→부하를 막는다
  {1: "VBAT_R", 2: "VBAT_R", 3: "VBAT_R", 4: "Q_G", 5: "VBAT_P", 6: "VBAT_P", 7: "VBAT_P", 8: "VBAT_P"}, 12.0, 41.0, 0)
P("R1", R06, "100k", {1: "VBAT_R", 2: "Q_G"}, 12.0, 51.0)                 # 스위치 열림 → Vgs 0 → 꺼짐
P("R68", R06, "10k", {1: "Q_G", 2: "PWR_SW"}, 8.0, 51.0)                  # 닫힘 → Vgs ≈ −0.9·Vbat(12V 클램프)
P("D1", ("Diode_SMD", "D_SOD-123"), "BZT52C12", {1: "VBAT_R", 2: "Q_G"}, 16.5, 51.0)   # Vgs 클램프(Vgs 최대 20V)
P("J8", H254(2), "POWER SW", {1: "PWR_SW", 2: "GND"}, 2.0, 38.0, 90)      # 잠금형 스위치(토글·래칭 푸시). 빠지면 꺼진다
P("D2", ("Diode_SMD", "D_SMB"), "SMBJ15A", {1: "VM", 2: "GND"}, 19.0, 46.0, 90)      # 역기전력. DRV8874 37V 안
P("C1", ("Capacitor_THT", "CP_Radial_D12.5mm_P5.00mm"), "1000u 35V", {1: "VBAT_P", 2: "GND"}, 27.0, 46.5)   # 명세서: 벌크 1000µF×2 이상
P("C14", ("Capacitor_THT", "CP_Radial_D12.5mm_P5.00mm"), "1000u 35V", {1: "VBAT_P", 2: "GND"}, 35.0, 47.5)
# 비상정지: J6(외부 NC 버튼, 없으면 단락) 직렬로 5V → 릴레이 코일. 버튼을 누르면 코일이 꺼지고 접점이 VM을 끊는다(MCU 무관).
# 코일 쪽 전압을 MCU(PB13)가 읽어 젯슨에 보고한다(estop=1). 펌웨어의 S·데드맨은 그대로 보조 정지.
P("K1", ("Relay_THT", "Relay_SPDT_SANYOU_SRD_Series_Form_C"), "SRD-05VDC-SL-C",
  {1: "VBAT_P", 2: "COIL", 5: "GND", 3: "VM", 4: None}, 33.0, 38.0, 0)               # 1=COM 3=NO 4=NC(안 씀) 2·5=코일
P("D7", ("Diode_SMD", "D_SMA"), "SS34", {1: "COIL", 2: "GND"}, 40.0, 41.0, 90)        # 코일 역기전력
P("J6", ("TerminalBlock", "TerminalBlock_bornier-2_P5.08mm"), "E-STOP (NC)", {1: "5V", 2: "COIL"}, 4.5, 34.0, 90)
P("R60", R06, "10k", {1: "COIL", 2: "ESTOP"}, 46.0, 36.0)                            # FT 핀 PB13, 코일 5V를 직렬저항으로
P("R61", R06, "100k", {1: "ESTOP", 2: "GND"}, 46.0, 38.0)
P("D3", ("Diode_SMD", "D_SMA"), "SS34", {1: "VLOG", 2: "VBAT_P"}, 6.5, 30.0, 90)          # 모터가 VM을 끌어내려도 로직은 홀드업으로 버틴다
P("C2", ("Capacitor_THT", "CP_Radial_D10.0mm_P5.00mm"), "470u 25V", {1: "VLOG", 2: "GND"}, 7.0, 29.5, 90)   # 절연 칸(y<ISO_Y) 밖
P("U2", ("Package_TO_SOT_SMD", "SOT-23-6"), "TPS542025 (5V)",
  {1: "GND", 2: "SW5", 3: "VLOG", 4: "5V", 5: None, 6: "BST5"}, 14.0, 29.0)          # EN 띄움 = 켜짐
P("C3", C12, "10u 25V", {1: "VLOG", 2: "GND"}, 14.0, 33.0)
P("C4", C06, "100n", {1: "BST5", 2: "SW5"}, 14.0, 25.5)
P("L1", ("Inductor_SMD", "L_Bourns_SRP7028A_7.3x6.6mm"), "10uH SRP7028A-100M", {1: "SW5", 2: "5V"}, 20.5, 29.0)
P("C5", C12, "22u 10V", {1: "5V", 2: "GND"}, 20.5, 34.5)
P("C6", C12, "22u 10V", {1: "5V", 2: "GND"}, 20.5, 37.5)
P("U3", ("Package_TO_SOT_SMD", "SOT-223-3_TabPin2"), "AMS1117-3.3", {1: "GND", 2: "3V3", 3: "5V"}, 27.0, 36.0)
P("C7", C12, "10u 25V", {1: "5V", 2: "GND"}, 23.5, 40.5)
P("C8", C12, "10u 25V", {1: "3V3", 2: "GND"}, 30.5, 40.5)
P("R2", R06, "1k", {1: "3V3", 2: "LEDP"}, 33.0, 40.5)
P("D4", LED, "PWR 초록", {1: "GND", 2: "LEDP"}, 35.5, 40.5)

# ---------------- USB 쪽(절연 너머, GNDU) : USB Mini-B → CH340C → ADuM1201 ----------------
P("J2", ("Connector_USB", "USB_Mini-B_Wuerth_65100516121_Horizontal"), "USB Mini-B",   # 0.8mm 피치 — USB-C(0.5mm)는 손땜이 어렵다
  {1: "VBUS", 2: "DM", 3: "DP", 4: None, 5: "GNDU", 6: "GNDU"}, 3.62, 9.0, 270)   # 고정탭이 판 끝에서 0.5mm 안에 들게
P("U4", ("Package_SO", "SOIC-16_3.9x9.9mm_P1.27mm"), "CH340C",
  {1: "GNDU", 2: "U_TXC", 3: "U_RXC", 4: "U_V3", 5: "DP", 6: "DM", 7: None, 8: None, 9: None, 10: None,
   11: None, 12: None, 13: None, 14: None, 15: None, 16: "VBUS"}, 12.5, 11.0)
P("C9", C06, "100n", {1: "U_V3", 2: "GNDU"}, 9.5, 17.5)
P("C10", C06, "100n", {1: "VBUS", 2: "GNDU"}, 12.5, 17.5)
P("C11", C12, "10u 25V", {1: "VBUS", 2: "GNDU"}, 9.5, 20.0)
P("U5", ("Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm"), "ADuM1201ARZ",                # 1~4=USB측, 5~8=MCU측
  {1: "ISO_V1", 2: "U_RX", 3: "U_TX", 4: "GNDU", 5: "GND", 6: "MCU_RX", 7: "MCU_TX", 8: "3V3"}, 20.0, 9.0, 90)
# 젯슨 40핀 UART 직결 대안(명세서: 3.3V UART, 레벨시프터 없음) — 절연기 **앞**에 붙여 절연은 그대로 둔다.
# USB로 쓸 땐 R62~R64(0Ω) 실장 + J7 비움. J7로 쓸 땐 R62~R64를 빼고 젯슨 TX→2, RX←3, 3.3V→4, GND→1.
P("R62", R06, "0", {1: "U_TXC", 2: "U_TX"}, 16.0, 6.0, 90)
P("R63", R06, "0", {1: "U_RX", 2: "U_RXC"}, 16.0, 9.0, 90)
P("R64", R06, "0", {1: "VBUS", 2: "ISO_V1"}, 16.0, 12.0, 90)
P("J7", ("Connector_PinHeader_2.54mm", "PinHeader_1x04_P2.54mm_Vertical"), "JETSON UART",
  {1: "GNDU", 2: "U_TX", 3: "U_RX", 4: "ISO_V1"}, 9.0, 1.8, 90)
P("C12", C06, "100n", {1: "ISO_V1", 2: "GNDU"}, 17.5, 3.0)
P("C13", C06, "100n", {1: "3V3", 2: "GND"}, 23.0, 3.0)

# ---------------- MCU STM32F405RGT6 ----------------
mcu = {1: "3V3", 2: None, 3: None, 4: None, 5: "OSC_IN", 6: "OSC_OUT", 7: "NRST",
       8: "IPR_A", 9: "IPR_B", 10: "IPR_C", 11: "IPR_D", 12: "GND", 13: "3V3", 14: "ENC_D_A", 15: "ENC_D_B",
       16: "PWM_C1", 17: "PWM_C2", 18: "GND", 19: "3V3", 20: "SPI_CS", 21: "SPI_SCK", 22: "SPI_MISO", 23: "SPI_MOSI",
       24: "VB_SNS", 25: "VM_SNS", 26: "V5_SNS", 27: "LED_FLT", 28: "BOOT1", 29: "IMU_INT", 30: "LED_ST",
       31: "VCAP1", 32: "3V3", 33: "NSLEEP", 34: "ESTOP", 35: "PWM_D1", 36: "PWM_D2",
       37: "PWM_A1", 38: "PWM_A2", 39: "PWM_B1", 40: "PWM_B2", 41: None, 42: "MCU_TX", 43: "MCU_RX",
       44: None, 45: None, 46: "SWDIO", 47: "VCAP2", 48: "3V3", 49: "SWCLK", 50: "ENC_A_A",
       51: "FLT_A", 52: "FLT_B", 53: "FLT_C", 54: "FLT_D", 55: "ENC_A_B", 56: "ENC_B_A", 57: "ENC_B_B",
       58: "ENC_C_A", 59: "ENC_C_B", 60: "BOOT0", 61: "I2C_SCL", 62: "I2C_SDA", 63: "GND", 64: "3V3"}
MX, MY = 36.0, 26.5   # 100×70 판에서 H4 고정구멍을 피하는 자리
P("U1", ("Package_QFP", "LQFP-64_10x10mm_P0.5mm"), "STM32F405RGT6", mcu, MX, MY, 0)
for i, (x, y) in enumerate([(MX - 7.5, MY - 3), (MX - 7.5, MY + 3), (MX + 7.5, MY - 3), (MX + 7.5, MY + 3), (MX, MY - 7.6)]):
    P(f"C{20 + i}", C06, "100n", {1: "3V3", 2: "GND"}, x, y, 90)              # VDD×4 + VDDA
P("C25", C06, "2.2u", {1: "VCAP1", 2: "GND"}, MX - 3, MY + 7.6)
P("C26", C06, "2.2u", {1: "VCAP2", 2: "GND"}, MX + 3, MY - 7.6)
P("C27", C06, "4.7u", {1: "3V3", 2: "GND"}, MX + 3, MY + 7.6)
P("Y1", ("Crystal", "Crystal_SMD_3225-4Pin_3.2x2.5mm"), "8MHz 3225 12pF", {1: "OSC_IN", 2: "GND", 3: "OSC_OUT", 4: "GND"}, MX - 10.5, MY - 7.5)
P("C28", C06, "20p", {1: "OSC_IN", 2: "GND"}, MX - 13.5, MY - 9.5)
P("C29", C06, "20p", {1: "OSC_OUT", 2: "GND"}, MX - 13.5, MY - 5.5)
P("C30", C06, "100n", {1: "NRST", 2: "GND"}, MX - 10.0, MY + 8.0)
P("R5", R06, "10k", {1: "BOOT0", 2: "GND"}, MX - 3.0, MY + 10.0)
P("R6", R06, "10k", {1: "BOOT1", 2: "GND"}, MX - 6.0, MY + 10.0)
P("SW1", ("Button_Switch_THT", "SW_PUSH_6mm"), "RESET", {1: "NRST", 2: "GND"}, 45.5, 39.0)
P("SW2", ("Button_Switch_THT", "SW_PUSH_6mm"), "BOOT0", {1: "BOOT0", 2: "3V3"}, 54.0, 39.0)
P("R7", R06, "1k", {1: "LED_ST", 2: "LEDS"}, MX + 9.5, MY + 9.0)
P("D5", LED, "상태 초록", {1: "GND", 2: "LEDS"}, MX + 12.0, MY + 9.0)
P("R8", R06, "1k", {1: "LED_FLT", 2: "LEDF"}, MX + 9.5, MY + 11.0)
P("D6", LED, "고장 빨강", {1: "GND", 2: "LEDF"}, MX + 12.0, MY + 11.0)
P("R9", R06, "100k", {1: "VBAT_P", 2: "VB_SNS"}, MX - 9.5, MY + 3.5)            # 배터리/11 → 36V까지(명세서 100k/10k, 24V 배터리도 OK)
P("R10", R06, "10k", {1: "VB_SNS", 2: "GND"}, MX - 9.5, MY + 5.5)
P("R69", R06, "10k", {1: "5V", 2: "V5_SNS"}, MX - 14.5, MY + 3.5)               # 5V/2 → PB0. 3V3는 핀 없이 VREFINT로 잰다
P("R70", R06, "10k", {1: "V5_SNS", 2: "GND"}, MX - 14.5, MY + 5.5)
P("R11", R06, "100k", {1: "VM", 2: "VM_SNS"}, MX - 12.0, MY + 3.5)             # 릴레이 **뒤** VM 감지(÷11) — E-stop인데 VM이 살아 있으면 접점 용착
P("R12", R06, "10k", {1: "VM_SNS", 2: "GND"}, MX - 12.0, MY + 5.5)
# VM 방전(블리더) 2kΩ = 1k×2 직렬(0603 각 ≈40mW @12.6V). 없으면 드라이버가 잘 때 VM(≈40µF)이 110k로만 빠져
# 시정수 4.4초 → E-stop마다 "VM이 안 죽었다 = 접점 용착"으로 오판한다. 2k면 ≈80ms. 멈춘 뒤 저장 에너지도 없앤다.
P("R66", R06, "1k", {1: "VM", 2: "BLD"}, 40.0, 40.0)
P("R67", R06, "1k", {1: "BLD", 2: "GND"}, 42.0, 40.0)

# ---------------- 모터 드라이버 4 × DRV8874 (PWM 모드) + 엔코더 커넥터 ----------------
# 커넥터 = 기존 쉴드와 같은 6핀 순서: M− · GND · ENC_B · ENC_A · VCC50 · M+   (⚠ 피치·방향 실물 확인)
# 바퀴 = A/RR B/RL C/FR D/FL (펌웨어 인덱스 0..3). 위쪽 가장자리 = A,B / 아래 = C,D 는 배치일 뿐.
P("R13", R06, "100k", {1: "3V3", 2: "VREF"}, 50.0, 22.0, 90)                   # VREF 1.06V → 1k IPROPI에서 ≈2.3A 제한
P("R14", R06, "47k", {1: "VREF", 2: "GND"}, 52.0, 22.0, 90)
P("R15", R06, "100k", {1: "NSLEEP", 2: "GND"}, 50.0, 31.0, 90)                 # 부팅 중엔 드라이버가 잔다
P("R16", R06, "0", {1: "3V3", 2: "PMODE"}, 52.0, 31.0, 90)                    # PMODE high = PWM(IN1/IN2) — 데이터시트 표로 확인, 옮길 수 있게 0Ω

drv_pos = {"A": (31.5, 7.5, 0), "B": (52.5, 7.5, 0), "C": (31.5, 44.0, 180), "D": (52.5, 44.0, 180)}
conn_pos = {"A": (24.0, 2.6, 0), "B": (45.0, 2.6, 0), "C": (38.5, 50.7, 180), "D": (63.5, 50.7, 180)}
enc_pins = {"A": ("ENC_A_A", "ENC_A_B"), "B": ("ENC_B_A", "ENC_B_B"), "C": ("ENC_C_A", "ENC_C_B"), "D": ("ENC_D_A", "ENC_D_B")}
for n, w in enumerate("ABCD"):
    x, y, r = drv_pos[w]; s = 1 if r == 0 else -1   # 아래줄은 거울로
    P(f"U1{n}", HTSSOP, "DRV8874PWPR",
      {1: f"PWM_{w}1", 2: f"PWM_{w}2", 3: "NSLEEP", 4: f"FLT_{w}", 5: "VREF", 6: f"IPR_{w}", 7: "GND",
       8: f"M{w}+", 9: "GND", 10: f"M{w}-", 11: "VM", 12: f"VCP_{w}", 13: f"CPH_{w}", 14: f"CPL_{w}",
       15: "GND", 16: "PMODE", 17: "GND"}, x, y, r)
    P(f"C{40 + n * 4}", C06, "100n 50V", {1: f"VCP_{w}", 2: "VM"}, x + 4.6, y - 2.0 * s, 90)
    P(f"C{41 + n * 4}", C06, "22n 50V", {1: f"CPH_{w}", 2: f"CPL_{w}"}, x + 6.2, y - 2.0 * s, 90)
    P(f"C{42 + n * 4}", C06, "100n 50V", {1: "VM", 2: "GND"}, x + 4.6, y + 2.6 * s, 90)
    P(f"C{43 + n * 4}", C12, "10u 25V", {1: "VM", 2: "GND"}, x + 8.3, y + 0.3 * s, 90)
    P(f"R{20 + n * 4}", R06, "1k", {1: f"IPR_{w}", 2: "GND"}, x - 4.6, y + 2.6 * s, 90)     # IPROPI 0.45V/A
    P(f"R{21 + n * 4}", R06, "10k", {1: "3V3", 2: f"FLT_{w}"}, x - 6.2, y + 2.6 * s, 90)   # nFAULT 풀업
    ea, eb = enc_pins[w]
    cx, cy, cr = conn_pos[w]
    P(f"J{10 + n}", PH6, f"MOTOR {w}", {1: f"M{w}-", 2: "GND", 3: f"E{w}B", 4: f"E{w}A", 5: "5V", 6: f"M{w}+"}, cx, cy, cr)
    # 엔코더: 5V 홀센서 → 1k 직렬(MCU 꺼졌을 때 전류 제한) → FT 핀, 10k 풀업(오픈컬렉터 대응)
    P(f"R{40 + n * 4}", R06, "1k", {1: f"E{w}A", 2: ea}, x - 4.6, y - 2.0 * s, 90)
    P(f"R{41 + n * 4}", R06, "1k", {1: f"E{w}B", 2: eb}, x - 6.2, y - 2.0 * s, 90)
    P(f"R{42 + n * 4}", R06, "10k", {1: "3V3", 2: ea}, x - 7.8, y - 2.0 * s, 90)
    P(f"R{43 + n * 4}", R06, "10k", {1: "3V3", 2: eb}, x - 7.8, y + 2.6 * s, 90)

# ---------------- 헤더: SWD · I2C(IMU 이전용) · SPI ----------------
P("J3", H254(5), "SWD", {1: "3V3", 2: "SWDIO", 3: "SWCLK", 4: "NRST", 5: "GND"}, 64.5, 21.5)
P("J4", H254(4), "I2C (MPU6050)", {1: "3V3", 2: "GND", 3: "I2C_SCL", 4: "I2C_SDA"}, 60.5, 21.5)
P("R17", R06, "10k", {1: "3V3", 2: "I2C_SCL"}, 56.5, 22.0, 90)
P("R18", R06, "10k", {1: "3V3", 2: "I2C_SDA"}, 58.0, 30.0, 90)
P("J5", H254(7), "SPI IMU", {1: "3V3", 2: "GND", 3: "SPI_SCK", 4: "SPI_MISO", 5: "SPI_MOSI", 6: "SPI_CS", 7: "IMU_INT"}, 66.5, 4.0, 0)   # 오른쪽 헤더 줄 — 절연 칸 밖

UNO_HOLES = [(14.0, 2.54), (15.24, 50.8), (66.04, 7.62), (66.04, 35.56)]   # Uno 도면(왼쪽아래 원점) → 아래에서 y 뒤집음


def build():
    b = pcbnew.BOARD()
    ds = b.GetDesignSettings()   # DRV8874 방열 비아가 0.2mm 드릴 — 보통 PCB 업체 최소치(JLC 0.2/0.3)
    ds.m_MinThroughDrill = pcbnew.FromMM(0.2); ds.m_HoleClearance = pcbnew.FromMM(0.2)
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
        fp.Reference().SetTextSize(pcbnew.VECTOR2I_MM(0.8, 0.8)); fp.Reference().SetTextThickness(pcbnew.FromMM(0.15))   # 실크 겹침 줄이기(0.8mm = 업체 최소 가독)
        names = {p.GetNumber() for p in fp.Pads() if p.GetNumber()}
        missing = set(pads) - names
        if missing:
            sys.exit(f"{ref}: 패드 {missing} 가 {fpn}에 없다")
        for p in fp.Pads():
            n = pads.get(p.GetNumber())
            if n:
                p.SetNet(net(n))
        b.Add(fp)
        fp.SetPosition(pcbnew.VECTOR2I_MM(x * W / UW, y * H / UH))
        fp.SetOrientationDegrees(rot)
    for i, (hx, hy) in enumerate(UNO_HOLES):
        h = pcbnew.FootprintLoad(f"{FP}/MountingHole.pretty", "MountingHole_3.2mm_M3")
        h.SetReference(f"H{i + 1}"); b.Add(h); h.SetPosition(pcbnew.VECTOR2I_MM(hx, H - hy))
    for (x0, y0), (x1, y1) in [((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))]:
        s = pcbnew.PCB_SHAPE(b); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetLayer(pcbnew.Edge_Cuts)
        s.SetStart(pcbnew.VECTOR2I_MM(x0, y0)); s.SetEnd(pcbnew.VECTOR2I_MM(x1, y1)); s.SetWidth(pcbnew.FromMM(0.1))
        b.Add(s)
    # 절연 경계 표시(실크) — GNDU 쪽 동박이 이 선을 넘지 않게 라우팅할 것
    return b


def iso_line(b):
    """절연 경계(실크) — ADuM1201 한가운데를 지나는 세로선. GNDU 동박이 이 선을 넘으면 절연이 없다."""
    u5 = b.FindFootprintByReference("U5"); x = pcbnew.ToMM(u5.GetPosition().x)
    s = pcbnew.PCB_SHAPE(b); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetLayer(pcbnew.F_SilkS)
    s.SetStart(pcbnew.VECTOR2I_MM(x, 0.5)); s.SetEnd(pcbnew.VECTOR2I_MM(x, ISO_Y)); s.SetWidth(pcbnew.FromMM(0.2)); b.Add(s)


def overlaps(b):
    """F.CrtYd 사각형끼리 겹치는 쌍 + 판 밖으로 나간 부품. 0이어야 배치 끝."""
    boxes = []
    for fp in b.GetFootprints():
        bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox() if fp.GetCourtyard(pcbnew.F_CrtYd).OutlineCount() else fp.GetBoundingBox(False)
        boxes.append((fp.GetReference(), bb))
    bad = []
    for i in range(len(boxes)):
        r, a = boxes[i]
        if a.GetLeft() < 0 or a.GetTop() < 0 or a.GetRight() > pcbnew.FromMM(W) or a.GetBottom() > pcbnew.FromMM(H):
            if not r.startswith(("J1", "J2", "H")):          # 커넥터는 가장자리에 걸치는 게 정상
                bad.append(f"{r} 판 밖")
        for j in range(i + 1, len(boxes)):
            if a.Intersects(boxes[j][1]):
                bad.append(f"{r}↔{boxes[j][0]}")
    return bad


def crt(fp, gap=0.25):
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
    bb.Inflate(pcbnew.FromMM(gap)); return bb


fixed = {"H1", "H2", "H3", "H4", "J1", "J2", "J10", "J11", "J12", "J13", "U1"}


def settle(b):
    """적힌 자리는 '여기 근처'라는 뜻이다. 큰 것부터 그 자리에 두고, 겹치면 나선으로 가장 가까운
    빈자리(0/90° 둘 다)를 찾는다. ponytail: 탐욕 배치라 최적은 아니다 — 손으로 다듬을 출발점."""
    import math
    fps = sorted(b.GetFootprints(), key=lambda f: (f.GetReference() not in fixed, -crt(f).GetArea()))
    placed, lim = [], (pcbnew.FromMM(0), pcbnew.FromMM(0), pcbnew.FromMM(W), pcbnew.FromMM(H))
    def ok(f):
        a = crt(f, 0.0 if f.GetReference().startswith("H") else 0.25)
        edge = f.GetReference() in ("J1", "J2") or f.GetReference().startswith("J1")
        if not edge and (a.GetLeft() < lim[0] or a.GetTop() < lim[1] or a.GetRight() > lim[2] or a.GetBottom() > lim[3]):
            return False
        return not any(a.Intersects(p) for p in placed)
    for f in fps:
        p0, r0 = f.GetPosition(), f.GetOrientationDegrees()
        found = f.GetReference() in fixed or ok(f)
        rad = 0.0
        while not found and rad < 30:
            rad += 0.5
            for k in range(max(8, int(rad * 6))):
                t = 2 * math.pi * k / max(8, int(rad * 6))
                for rot in (r0, r0 + 90):
                    f.SetOrientationDegrees(rot)
                    f.SetPosition(pcbnew.VECTOR2I(p0.x + pcbnew.FromMM(rad * math.cos(t)), p0.y + pcbnew.FromMM(rad * math.sin(t))))
                    if ok(f):
                        found = True; break
                if found: break
        if not found:
            f.SetPosition(p0); f.SetOrientationDegrees(r0); print("자리 못 찾음:", f.GetReference())
        placed.append(crt(f, 0.0 if f.GetReference().startswith("H") else 0.25))


USB_NETS = {"GNDU", "VBUS", "U_TXC", "U_RXC", "ISO_V1", "DP", "DM", "U_V3", "U_TX", "U_RX"}


def iso_check(b):
    """절연 칸(U5 중심 왼쪽, y<ISO_Y)엔 USB 쪽 부품만, USB 쪽 부품은 칸 안에만. 어기면 GND 동박이 칸을 침범한다."""
    ix = pcbnew.ToMM(b.FindFootprintByReference("U5").GetPosition().x)
    bad = []
    for f in b.GetFootprints():
        r = f.GetReference()
        if r == "U5" or r.startswith("H"): continue
        bb = f.GetBoundingBox(False)
        inside = pcbnew.ToMM(bb.GetLeft()) < ix and pcbnew.ToMM(bb.GetTop()) < ISO_Y
        usb = bool({p.GetNetname() for p in f.Pads()} & USB_NETS)
        if inside != usb: bad.append(f"{r} {'칸 침범' if inside else 'USB 부품이 칸 밖'}")
    return bad


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
    iso = iso_check(b)
    print("절연 칸 위반:", len(iso)); print(chr(10).join(iso))

/*
 * ps2_toggle — Moebius MecanumRobot(Uno) 키트의 PS2 L1 버튼을 눌러
 * "미러링 토글" 신호를 젯슨에 보낸다.
 *
 *   L1 버튼이 "새로" 눌릴 때마다  ->  Serial: "TOGGLE\n"
 *
 * 젯슨의 tools/mirror_toggle.py 가 이 줄을 받아 리더->팔로워 미러링을 ON/OFF 토글한다.
 *
 * 프로토콜 (115200 baud, 줄단위):
 *   Arduino -> "TOGGLE\n" : L1 버튼이 막 눌림 (손상필터+디바운스 통과)
 *   Arduino -> "READY\n"  : PS2 패드 인식 완료
 *   Arduino -> "ERR\n"    : PS2 패드 인식 실패
 *
 * ── 이 보드(확정 배선, [[moebius-mecanum-hardware]]) ──────────────────────
 *   PS2 핀: CLK=12, CMD=11, SEL(ATT)=10, DAT=13   (config_gamepad(12,11,10,13))
 *   모터는 PCA9685(I2C, A4/A5) 경유라 이 핀들과 무관 — 여기선 모터 구동 안 함.
 *
 *   ⚠️ 손상프레임 판정은 아날로그값으로 하면 안 된다:
 *      컨트롤러가 "디지털 모드"(아날로그 LED 꺼짐)면 4축이 정상적으로 0xFF(255)다.
 *      그래서 손상은 "동시에 7개 이상 버튼 눌림"으로만 판정한다(0x00 깨짐 프레임).
 *      디지털/아날로그 모드 둘 다에서 L1이 읽힌다.
 * ────────────────────────────────────────────────────────────────────────
 * 의존: PS2X_lib
 */

#include <PS2X_lib.h>

#define PS2_CLK 12
#define PS2_CMD 11
#define PS2_SEL 10
#define PS2_DAT 13
#define TOGGLE_BUTTON PSB_L1   // L2로 바꾸려면 PSB_L2

PS2X ps2x;
bool padOk = false;
unsigned long lastPs2 = 0;

// 손상프레임 검사용 전체 버튼 목록
const uint16_t ALL_BTN[] = {
  PSB_L1, PSB_L2, PSB_R1, PSB_R2, PSB_L3, PSB_R3, PSB_START, PSB_SELECT,
  PSB_PAD_UP, PSB_PAD_DOWN, PSB_PAD_LEFT, PSB_PAD_RIGHT,
  PSB_TRIANGLE, PSB_CIRCLE, PSB_CROSS, PSB_SQUARE
};

// 토글 디바운스 상태
bool    armed = true;   // 다음 누름에 토글 발사 가능?
uint8_t relCnt = 0;     // 연속 "뗌" 프레임 수

bool initPad() {
  return ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false) == 0;
}

void pollPs2() {
  if (millis() - lastPs2 < 50) return;   // PS2는 ~50ms 간격 폴링
  lastPs2 = millis();

  if (!padOk) {
    padOk = initPad();
    if (padOk) Serial.println("READY");
    return;
  }

  ps2x.read_gamepad();

  // 손상프레임: 동시에 7개 이상 버튼이 눌렸다고 나오면 잡음(0x00 깨짐). 버린다.
  uint8_t nb = 0;
  for (uint8_t i = 0; i < 16; i++) if (ps2x.Button(ALL_BTN[i])) nb++;
  if (nb >= 7) return;

  bool lNow = ps2x.Button(TOGGLE_BUTTON);
  if (lNow) {
    relCnt = 0;
    if (armed) {                 // 뗐다가 다시 누른 첫 프레임 = 토글
      Serial.println("TOGGLE");
      armed = false;
    }
  } else {
    if (relCnt < 10) relCnt++;
    if (relCnt >= 2) armed = true;   // 유효 2연속 뗌 = 재무장
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);                    // PS2 전원 안정화
  padOk = initPad();
  Serial.println(padOk ? "READY" : "ERR");
}

void loop() {
  pollPs2();
}

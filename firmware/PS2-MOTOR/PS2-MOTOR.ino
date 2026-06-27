#include <PS2X_lib.h>  //for MOEBIUS
#include "FaBoPWM_PCA9685.h"
#include <avr/wdt.h>   // 워치독: 루프가 행되면 자동 리셋 → 모터 폭주 방지

//#include "servo.hpp"

FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;   // 电机速度限制 motor speed
int MIN_VALUE = 300;

//PS2手柄引脚；PS2 handle Pin
#define PS2_DAT        13
#define PS2_CMD        11
#define PS2_SEL        10
#define PS2_CLK        12

//MOTOR CONTROL Pin
#define DIRA1 0
#define DIRA2 1
#define DIRB1 2
#define DIRB2 3
#define DIRC1 4
#define DIRC2 5
#define DIRD1 6
#define DIRD2 7

char speed;
#define pressures   false
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

int error = 0;
byte type = 0;
byte vibrate = 0;

void (* resetFunc) (void) = 0;

//电机控制，前进、后退、停止   motor control advance\back\stop
#define MOTORA_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRA1,pwm);faboPWM.set_channel_value(DIRA2, 0);}while(0)
#define MOTORA_STOP(x)         do{faboPWM.set_channel_value(DIRA1,0);faboPWM.set_channel_value(DIRA2, 0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRA1,0);faboPWM.set_channel_value(DIRA2, pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRB1,pwm);faboPWM.set_channel_value(DIRB2, 0);}while(0)
#define MOTORB_STOP(x)         do{faboPWM.set_channel_value(DIRB1,0);faboPWM.set_channel_value(DIRB2, 0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRB1,0);faboPWM.set_channel_value(DIRB2, pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRC1,pwm);faboPWM.set_channel_value(DIRC2, 0);}while(0)
#define MOTORC_STOP(x)         do{faboPWM.set_channel_value(DIRC1,0);faboPWM.set_channel_value(DIRC2, 0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRC1,0);faboPWM.set_channel_value(DIRC2, pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRD1,pwm);faboPWM.set_channel_value(DIRD2, 0);}while(0)
#define MOTORD_STOP(x)         do{faboPWM.set_channel_value(DIRD1,0);faboPWM.set_channel_value(DIRD2, 0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRD1,0);faboPWM.set_channel_value(DIRD2, pwm);}while(0)

#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 600;   // 속도 30% (최대 스케일 2000 기준). 전류↓ → brownout/행 완화.

//    ↑A-----B↑
//     |  ↑  |
//    ↑C-----D↑
void ADVANCE(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
void BACK()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_STOP(Motor_PWM);
}
void LEFT_2()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
void RIGHT_2()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_STOP(Motor_PWM);
}
void rotate_1()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
void rotate_2()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
void STOP()
{
  MOTORA_STOP(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

void IO_init()
{
  STOP();
}

void setup()
{
  wdt_disable();   // 워치독 리셋 직후 부팅루프 방지(부팅 시 즉시 해제)
  IO_init();
  Serial.begin(115200);   // 젯슨 mirror_toggle.py(115200)와 맞춤
  if(faboPWM.begin())
  {
    Serial.println("Find PCA9685");
    faboPWM.init(300);
  }
  faboPWM.set_hz(50);
  Serial.print("Start");

  delay(300); // 무선 PS2 모듈 기동 대기
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (error == 0) {
    Serial.println("Found Controller, configured successful");
  }
  else if (error == 1) {
    Serial.println("No controller found (will retry, no reboot)");
    // resetFunc() 제거: 노이즈로 한 번 실패해도 재부팅하지 않고 loop에서 재시도한다.
  }
  else if (error == 2)
    Serial.println("Controller found but not accepting commands");
  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode");

  type = ps2x.readType();
  Serial.print("Controller_type: "); Serial.println(type);

  wdt_enable(WDTO_250MS);   // 루프가 ~0.25초 이상 멈추면 자동 리셋(→setup의 STOP). 폭주 최소화.
  // 주: 120ms는 PS2X 재동기화 지연으로 오리셋 발생 → 250ms가 안정+짧은 폭주컷의 절충.
}

// 손상프레임 판정용 전체 버튼 목록 (핀13 노이즈 대응)
const uint16_t ALLB[] = {
  PSB_PAD_UP, PSB_PAD_DOWN, PSB_PAD_LEFT, PSB_PAD_RIGHT,
  PSB_PINK, PSB_RED, PSB_BLUE, PSB_GREEN, PSB_START, PSB_SELECT,
  PSB_L1, PSB_L2, PSB_R1, PSB_R2, PSB_L3, PSB_R3
};

void loop()
{
  wdt_reset();   // 루프 정상 동작 중 — 워치독 타이머 리셋

  // 아직 컨트롤러 미인식이면 재부팅 없이 조용히 재시도(노이즈로 한 번 실패해도 버팀).
  if (error != 0) {
    STOP();                 // 미인식 상태에선 모터 정지 보장
    wdt_reset();
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    wdt_reset();
    if (error == 0) type = ps2x.readType();
    delay(50);     // 120ms 워치독 미만 (컨트롤러 미인식 시 모터는 STOP 상태라 안전)
    wdt_reset();
    return;
  }
  if (type == 2) return;        // Guitar Hero 무시

  ps2x.read_gamepad(false, vibrate);

  // 손상프레임 가드(핀13 DAT 노이즈): 아날로그 4축이 모두 255거나 모두 0,
  // 또는 버튼이 7개 이상 동시에 눌림으로 나오면 손상 프레임 → 모터 상태 유지하고 스킵.
  // (이게 "가끔 멈춤"의 원인 — 손상 프레임에서 방향키가 '뗌'으로 잘못 읽혀 STOP되던 것)
  int lx = ps2x.Analog(PSS_LX), ly = ps2x.Analog(PSS_LY);
  int rx = ps2x.Analog(PSS_RX), ry = ps2x.Analog(PSS_RY);
  bool extreme = (lx == 255 && ly == 255 && rx == 255 && ry == 255) ||
                 (lx == 0   && ly == 0   && rx == 0   && ry == 0);
  uint8_t nb = 0;
  for (uint8_t i = 0; i < 16; i++) if (ps2x.Button(ALLB[i])) nb++;
  if (extreme || nb >= 7) { delay(20); return; }

  // L1 -> 젯슨 미러링 토글: 뗐다 다시 누르는 순간 1회 "TOGGLE" 전송.
  // (mirror_toggle.py가 받아 리더->팔로워 미러링 ON/OFF. L1은 주행에 안 쓰임.)
  static bool l1armed = true;
  static unsigned long lastTog = 0;
  if (ps2x.Button(PSB_L1)) {
    if (l1armed && millis() - lastTog > 300) {   // 300ms 리프랙토리: 노이즈 중복발사 방지
      Serial.println("TOGGLE");
      l1armed = false;
      lastTog = millis();
    }
  } else {
    l1armed = true;
  }

  // R1 + 면버튼(△○✕□) => 프리셋 1~4 명령(엣지). 젯슨이 미러링 상태에 따라 저장/재생.
  bool r1 = ps2x.Button(PSB_R1);
  const uint16_t FACE[4] = {PSB_TRIANGLE, PSB_CIRCLE, PSB_CROSS, PSB_SQUARE};
  static bool presetArmed[4] = {true, true, true, true};
  for (uint8_t i = 0; i < 4; i++) {
    bool face = ps2x.Button(FACE[i]);
    if (r1 && face && presetArmed[i]) {
      Serial.print("PRESET "); Serial.println(i + 1);   // 1=△ 2=○ 3=✕ 4=□
      presetArmed[i] = false;
    }
    if (!face) presetArmed[i] = true;   // 면버튼 떼면 재무장
  }

  // 누르고 있는 동안만 이동. (R1 누른 중엔 □○ 평행이동 무시 — 프리셋 모디파이어 우선)
  bool moving = true;
  if      (ps2x.Button(PSB_PAD_UP)   || ps2x.Button(PSB_START)) ADVANCE(500,500,500,500);
  else if (ps2x.Button(PSB_PAD_DOWN))    BACK();
  else if (ps2x.Button(PSB_PAD_LEFT))    rotate_2();   // 좌회전
  else if (ps2x.Button(PSB_PAD_RIGHT))   rotate_1();   // 우회전
  else if (!r1 && ps2x.Button(PSB_PINK)) LEFT_2();     // □ 좌평행(strafe)
  else if (!r1 && ps2x.Button(PSB_RED))  RIGHT_2();    // ○ 우평행(strafe)
  else moving = false;

  // 뗌 디바운스: 깨끗한 프레임에서 2회 연속 '아무 방향키도 안 눌림'일 때만 정지
  // → 손상 프레임 하나로 인한 순간 끊김 방지.
  static uint8_t noBtn = 0;
  if (moving) noBtn = 0;
  else if (++noBtn >= 2) STOP();
  delay(20);
}

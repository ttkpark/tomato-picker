/*
 * mecanum_serial — 젯슨(Orin)이 USB 시리얼로 직선 거리 명령을 보내면
 * 메카넘 4륜을 굴려 엔코더 틱만큼 이동 후 정지하고 DONE을 답하는 전용 펌웨어.
 *
 * 데모 목적의 "직선 이동 전용" 최소 구현이다. (회전/스트레이프 미포함 —
 * tomato_picker의 MobileBase.drive_to가 직선만 쓰기 때문.)
 *
 * 프로토콜 (115200 baud, 줄단위):
 *   젯슨 → "G <ticks>\n"  : ticks(부호=방향)만큼 직선 이동
 *   Arduino → "DONE\n"    : 도착해 정지 완료
 *   젯슨 → "P\n" → "OK\n" : 핑
 *   젯슨 → "S\n"          : 즉시 정지
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ ⚠️ 핀 번호는 Moebius 메카넘 쉴드 + HR8833 배선에 맞춰 반드시 확인!  │
 * │   아래 값은 일반적인 배치의 "예시"다. 보드 실크/회로도와 대조하고,   │
 * │   첫 테스트는 바퀴를 공중에 띄운(잭업) 상태로 할 것.                 │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * Uno는 하드웨어 인터럽트가 D2, D3 뿐이라 엔코더 4개를 전부 못 읽는다.
 * 직선 거리에는 좌/우 대표 한 개씩(2개)이면 충분하므로 D2=좌, D3=우만 읽는다.
 */

// ---- 모터 핀 (HR8833: 모터당 IN1/IN2. 한쪽 PWM, 다른쪽 방향) ----
// 메카넘 배치:  FL  FR
//               RL  RR
struct Motor { uint8_t in1; uint8_t in2; };
Motor FL = { 4,  5};   // 앞-좌
Motor FR = { 7,  6};   // 앞-우
Motor RL = { 8,  9};   // 뒤-좌
Motor RR = {12, 11};   // 뒤-우
// PWM 가능 핀(5,6,9,11)을 in2 쪽에 둬서 속도 제어에 쓴다.

// ---- 엔코더 핀 (대표 2개만 인터럽트) ----
const uint8_t ENC_LEFT  = 2;   // 좌측 대표 모터 엔코더 A상
const uint8_t ENC_RIGHT = 3;   // 우측 대표 모터 엔코더 A상

const int DRIVE_SPEED = 150;   // 0~255. 데모용 저속.

volatile long encLeft = 0;
volatile long encRight = 0;

void onEncLeft()  { encLeft++; }
void onEncRight() { encRight++; }

// motor: speed -255..255 (부호=방향). HR8833 한 핀 PWM + 한 핀 LOW/HIGH.
void setMotor(const Motor &m, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {            // 전진
    digitalWrite(m.in1, LOW);
    analogWrite(m.in2, speed);
  } else {                     // 후진
    analogWrite(m.in2, 0);
    digitalWrite(m.in1, HIGH);
    analogWrite(m.in2, 255 + speed);  // speed가 음수
  }
}

void driveAll(int speed) {
  setMotor(FL, speed);
  setMotor(FR, speed);
  setMotor(RL, speed);
  setMotor(RR, speed);
}

void brakeAll() {
  driveAll(0);
  digitalWrite(FL.in1, LOW); digitalWrite(FR.in1, LOW);
  digitalWrite(RL.in1, LOW); digitalWrite(RR.in1, LOW);
}

void setup() {
  Serial.begin(115200);
  uint8_t pins[] = {FL.in1, FL.in2, FR.in1, FR.in2,
                    RL.in1, RL.in2, RR.in1, RR.in2};
  for (uint8_t p : pins) pinMode(p, OUTPUT);
  brakeAll();

  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), onEncLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), onEncRight, RISING);
}

// ticks만큼(부호=방향) 직선 이동 후 정지. 좌/우 평균 틱으로 판정.
void moveTicks(long ticks) {
  long target = labs(ticks);
  int dir = (ticks >= 0) ? 1 : -1;
  noInterrupts(); encLeft = 0; encRight = 0; interrupts();

  driveAll(dir * DRIVE_SPEED);
  for (;;) {
    long l, r;
    noInterrupts(); l = encLeft; r = encRight; interrupts();
    if ((l + r) / 2 >= target) break;
    // 이동 중에도 비상정지(S) 명령은 받는다.
    if (Serial.available() && Serial.peek() == 'S') { Serial.read(); break; }
  }
  brakeAll();
  Serial.println("DONE");
}

void loop() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  switch (cmd) {
    case 'P': Serial.println("OK"); break;
    case 'S': brakeAll(); break;
    case 'G': moveTicks(Serial.parseInt()); break;
    default: break;  // 개행 등 무시
  }
}

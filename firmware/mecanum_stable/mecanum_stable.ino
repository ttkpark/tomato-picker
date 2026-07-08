// mecanum_stable — 젯슨 시리얼 주행용 안정 펌웨어 v1 (PS2 없음).
// 목표: 무슨 일이 있어도 폭주/행 없이, 젯슨과 안정적으로 통신.
// 방어: (1) 데드맨(명령 끊기면 자동정지) (2) HW 워치독(행 자동복구)
//       (3) 논블로킹 loop (4) PS2 완전분리.
//
// 프로토콜 (라인 단위, '\n' 종료):
//   V <vx> <vy> <w>   속도지령. vx=전진+, vy=우평행+, w=시계회전+. 각 -255..255
//   S                 즉시 정지
//   T <i> <d>         모터 i(0=A/FR 1=B/FL 2=C/RR 3=D/RL) 방향 d(1/-1) 단독 1초 (캘리브레이션)
//   P <n>             최대 PWM 스케일 설정 (기본 2000)
//   ?                 상태 1줄 출력
// 응답: 명령 처리 시 "ok ...", 주기적 "hb ..." 하트비트.
#include <avr/wdt.h>
#include "FaBoPWM_PCA9685.h"

FaBoPWM faboPWM;

// PCA9685 채널 (모터 A/B/C/D 각 2채널)
#define A1 0
#define A2 1
#define B1 2
#define B2 3
#define C1 4
#define C2 5
#define D1 6
#define D2 7

// 실측 매핑(2026-07-02): A=FR, B=FL, C=RR, D=RL
// 각 바퀴 "물리적 전진"이 어느 전기방향인지 부호로 보정 (캘리브레이션으로 확정).
// idx: 0=A(FR) 1=B(FL) 2=C(RR) 3=D(RL)
// 2026-07-03 실측: V140,0,0에서 좌(B,D)전진/우(A,C)후진 → 우측 반전으로 전진 정합.
int8_t POL[4] = {-1, 1, -1, 1};   // A(FR)반전 B(FL)정 C(RR)반전 D(RL)정. 런타임 'L'로 변경가능
const uint8_t CH1[4] = {A1, B1, C1, D1};
const uint8_t CH2[4] = {A2, B2, C2, D2};

int   MAX_PWM = 2000;             // PWM 최대 스케일
const unsigned long CMD_TIMEOUT = 400;  // ms: 이 시간 내 새 명령 없으면 데드맨 STOP
unsigned long lastCmd = 0;
bool  moving = false;

// 한 바퀴에 부호있는 속도 s(-255..255) 인가. wheel=0..3
void driveWheel(uint8_t w, int s){
  s = (int)s * POL[w];
  int mag = abs(s);
  if(mag > 255) mag = 255;
  int pwm = (long)mag * MAX_PWM / 255;   // 0..MAX_PWM
  if(s > 0){ faboPWM.set_channel_value(CH1[w], pwm); faboPWM.set_channel_value(CH2[w], 0); }
  else if(s < 0){ faboPWM.set_channel_value(CH1[w], 0); faboPWM.set_channel_value(CH2[w], pwm); }
  else { faboPWM.set_channel_value(CH1[w], 0); faboPWM.set_channel_value(CH2[w], 0); }
}

void STOP(){
  for(uint8_t w=0; w<4; w++) driveWheel(w, 0);
  moving = false;
}

// 메카넘 mixing. 표준 X-롤러 가정 (부호는 캘리브레이션으로 확정):
//  FL = vx - vy - w ; FR = vx + vy + w ; RL = vx + vy - w ; RR = vx - vy + w
// 바퀴→채널: A=FR, B=FL, C=RR, D=RL
void setVelocity(int vx, int vy, int w){
  int FL = vx - vy - w;
  int FR = vx + vy + w;
  int RL = vx + vy - w;
  int RR = vx - vy + w;
  driveWheel(0, FR);  // A
  driveWheel(1, FL);  // B
  driveWheel(2, RR);  // C
  driveWheel(3, RL);  // D
  moving = (vx || vy || w);
}

// ---- 시리얼 라인 버퍼 ----
char buf[48];
uint8_t blen = 0;

int nextInt(char*& p){
  while(*p==' ') p++;
  int sign=1;
  if(*p=='-'){ sign=-1; p++; } else if(*p=='+'){ p++; }
  int v=0; bool any=false;
  while(*p>='0' && *p<='9'){ v=v*10+(*p-'0'); p++; any=true; }
  return any ? sign*v : 0;
}

unsigned long testUntil = 0;  // T 명령 자동정지 시각

void handleLine(char* line){
  lastCmd = millis();
  char c = line[0];
  char* p = line+1;
  if(c=='V'){
    int vx=nextInt(p), vy=nextInt(p), w=nextInt(p);
    setVelocity(vx,vy,w);
    Serial.print(F("ok V ")); Serial.print(vx); Serial.print(' ');
    Serial.print(vy); Serial.print(' '); Serial.println(w);
  } else if(c=='S'){
    STOP(); Serial.println(F("ok S"));
  } else if(c=='T'){
    int i=nextInt(p), d=nextInt(p);
    if(i>=0 && i<4){
      STOP();
      driveWheel(i, d>=0 ? 200 : -200);
      moving = true;
      testUntil = millis()+1000;
      Serial.print(F("ok T ")); Serial.print(i); Serial.print(' '); Serial.println(d);
    }
  } else if(c=='P'){
    int n=nextInt(p); if(n>0&&n<=2000){ MAX_PWM=n; }
    Serial.print(F("ok P ")); Serial.println(MAX_PWM);
  } else if(c=='L'){
    for(uint8_t i=0;i<4;i++){ int v=nextInt(p); POL[i] = (v<0)?-1:1; }
    STOP();
    Serial.print(F("ok L POL=")); for(uint8_t i=0;i<4;i++){Serial.print((int)POL[i]);Serial.print(',');} Serial.println();
  } else if(c=='?'){
    Serial.print(F("st moving=")); Serial.print(moving);
    Serial.print(F(" MAX_PWM=")); Serial.print(MAX_PWM);
    Serial.print(F(" POL=")); for(uint8_t i=0;i<4;i++){Serial.print(POL[i]);Serial.print(',');}
    Serial.println();
  }
}

void setup(){
  wdt_disable();
  STOP();
  Serial.begin(115200);
  if(faboPWM.begin()){
    Serial.println(F("Find PCA9685 OK"));
    faboPWM.init(300);
  } else {
    Serial.println(F("PCA9685 begin FALSE"));
  }
  faboPWM.set_hz(50);
  STOP();
  lastCmd = millis();
  Serial.println(F("mecanum_stable v1 ready. proto: V vx vy w | S | T i d | P n | ?"));
  wdt_enable(WDTO_250MS);   // HW 워치독 (행 자동복구)
}

unsigned long lastHb = 0;

void loop(){
  wdt_reset();

  // 시리얼 수신 (논블로킹)
  while(Serial.available()){
    char ch = Serial.read();
    if(ch=='\n' || ch=='\r'){
      if(blen>0){ buf[blen]=0; handleLine(buf); blen=0; }
    } else if(blen < sizeof(buf)-1){
      buf[blen++]=ch;
    }
  }

  // T 테스트 자동정지
  if(testUntil && (long)(millis()-testUntil) >= 0){ STOP(); testUntil=0; Serial.println(F("T auto-stop")); }

  // 데드맨: 명령 끊기면 자동정지 (T 진행 중엔 예외)
  if(moving && !testUntil && (millis()-lastCmd) > CMD_TIMEOUT){
    STOP();
    Serial.println(F("deadman STOP"));
  }

  // 하트비트 1s
  if(millis()-lastHb > 1000){ lastHb=millis(); Serial.print(F("hb ")); Serial.println(millis()); }
}

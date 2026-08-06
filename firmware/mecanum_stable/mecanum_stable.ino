// mecanum_stable — 젯슨 시리얼 주행 펌웨어 v2 (PS2 없음).
// 목표: 무슨 일이 있어도 폭주/행 없이, 젯슨과 안정적으로 통신. 그리고 **조용히** 굴러갈 것.
//
// v2에서 바뀐 것 (v1의 "우우웅" 소음 + 주행 끊김을 잡는다):
//  (1) PWM 50Hz → 1500Hz.  ★ "우우웅"의 진짜 원인. 50Hz는 사람이 소리로 듣는 대역이라
//      모터가 초당 50번 끊기며 도는 게 그대로 저주파 울림이 된다. PCA9685의 상한(1526Hz)
//      근처로 올리면 가청 저역을 벗어나 훨씬 조용하고, 토크 리플도 줄어 저속이 부드러워진다.
//  (2) 슬루(가감속) 제한.  V 지령을 그대로 때리지 않고 5ms 틱마다 목표로 서서히 접근한다.
//      → 기동 인러시 피크가 낮아져 12V 레일 붕괴/젯슨 리셋 위험이 준다([[battery-power-system]]),
//        젯슨 쪽 지령이 한두 번 밀려도 속도가 튀지 않는다.
//  (3) 소프트 데드맨.  명령이 끊기면 즉시 STOP(=속도 0을 때림)이 아니라 목표를 0으로 두고
//      슬루로 감속한다. v1에서 "주웅 주웅 주우웅"으로 들리던 끊김이 이 하드 컷이었다.
//      완전 침묵이 HARD_TIMEOUT을 넘으면 그때는 즉시 0(진짜 안전 정지).
//  (4) 체크섬.  라인 끝에 "*HH"(앞부분 전체의 XOR, 2자리 16진수)를 붙이면 검증한다.
//      한 번이라도 올바른 체크섬을 받으면 그 뒤로는 체크섬 없는 V를 거부한다(strict 모드
//      자동 전환) — USB 노이즈로 깨진 "V 255 0 0"이 그대로 실행되는 사고를 막는다.
//  (5) V 응답 침묵.  50Hz로 들어오는 지령마다 "ok"를 되쏘면 TX 버퍼가 차서 loop가 막힌다.
//      상태는 1초 하트비트에 실어 보낸다(수신/오류 카운터 포함 = 링크 품질 지표).
//
// 프로토콜 (라인 단위, '\n' 종료. 모든 라인에 선택적으로 "*HH" 체크섬을 붙일 수 있다):
//   V <vx> <vy> <w>   속도지령. vx=전진+, vy=우평행+, w=시계회전+. 각 -255..255. (무응답)
//   S                 즉시 정지 (슬루 무시, 하드)
//   T <i> <d>         모터 i(0=A/FR 1=B/FL 2=C/RR 3=D/RL) 방향 d(1/-1) 단독 1초 (캘리브레이션)
//   P <n>             최대 PWM 스케일 설정 (기본 2000 / 4095)
//   L <a><b><c><d>    바퀴 극성 (각 1/-1)
//   R <accel> <decel> 슬루 스텝(틱당 변화량 상한). 기본 6 / 12
//   F <hz>            PWM 주파수 변경 (24~1526). 소음 튜닝용
//   ?                 상태 1줄 출력
// 응답: 처리 확인 "ok ...", 체크섬 오류 "nak crc", 1초 주기 "hb <ms> rx=<n> bad=<n> v=<vx,vy,w>"
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

int   MAX_PWM = 2000;             // PWM 최대 스케일 (PCA9685는 12bit=4095)

// ★ 소음의 핵심. 50Hz는 그대로 "우우웅"으로 들린다. PCA9685 상한이 1526Hz라
//   초음파(>20kHz)까지는 못 올리지만, 1500Hz면 저주파 울림은 사라지고 얇은 고음만 남는다.
//   드라이버가 이 주파수를 못 따라가 발열하면 F 명령으로 내려가며 찾을 것(1000/700/400...).
const uint16_t PWM_HZ_DEFAULT = 1500;

// 데드맨: 소프트(감속 시작) → 하드(즉시 0). v1은 400ms에서 곧바로 하드였다.
const unsigned long CMD_TIMEOUT  = 300;   // ms: 새 명령 없으면 목표를 0으로 (슬루로 감속)
const unsigned long HARD_TIMEOUT = 1000;  // ms: 이만큼 완전 침묵이면 즉시 0

// 슬루(틱당 변화량 상한). TICK_MS=5ms이므로 6 → 약 1200/초 = 0→255에 210ms.
const unsigned long TICK_MS = 5;
int ACCEL = 6;    // 속도를 "올릴" 때 (인러시 억제)
int DECEL = 12;   // 속도를 "내릴" 때 (정지는 빠르게 = 안전)

unsigned long lastCmd = 0;
unsigned long lastTick = 0;
unsigned long lastHb = 0;
unsigned long testUntil = 0;      // T 명령 자동정지 시각

int tgtVx = 0, tgtVy = 0, tgtW = 0;   // 젯슨이 원하는 속도
int curVx = 0, curVy = 0, curW = 0;   // 실제로 인가 중인 속도(슬루 결과)

// 링크 품질 카운터 — 하트비트로 젯슨에 보고한다.
uint16_t rxOk = 0, rxBad = 0;
bool strictCrc = false;   // 올바른 체크섬을 한 번 받으면 true → 이후 무체크섬 V 거부

bool testing = false;     // T 캘리브레이션 진행 중(슬루/데드맨 우회)

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

// 메카넘 mixing. 표준 X-롤러 가정 (부호는 캘리브레이션으로 확정):
//  FL = vx - vy - w ; FR = vx + vy + w ; RL = vx + vy - w ; RR = vx - vy + w
// 바퀴→채널: A=FR, B=FL, C=RR, D=RL
void applyCurrent(){
  int FL = curVx - curVy - curW;
  int FR = curVx + curVy + curW;
  int RL = curVx + curVy - curW;
  int RR = curVx - curVy + curW;
  driveWheel(0, FR);  // A
  driveWheel(1, FL);  // B
  driveWheel(2, RR);  // C
  driveWheel(3, RL);  // D
}

// 즉시 정지 — 슬루를 건너뛴다(비상/안전 경로).
void hardStop(){
  tgtVx = tgtVy = tgtW = 0;
  curVx = curVy = curW = 0;
  testing = false;
  applyCurrent();
}

// cur을 tgt 쪽으로 한 틱만큼 이동. 크기가 커지는 방향이면 ACCEL, 줄어드는 방향이면 DECEL.
// (부호가 바뀌는 구간은 "일단 0으로 줄어드는" 국면이라 DECEL이 걸려 자연히 빠르게 지난다.)
int approach(int cur, int tgt){
  if(cur == tgt) return cur;
  int step = (abs(tgt) > abs(cur)) ? ACCEL : DECEL;
  if(tgt > cur){ cur += step; if(cur > tgt) cur = tgt; }
  else         { cur -= step; if(cur < tgt) cur = tgt; }
  return cur;
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

int8_t hexVal(char c){
  if(c>='0'&&c<='9') return c-'0';
  if(c>='a'&&c<='f') return c-'a'+10;
  if(c>='A'&&c<='F') return c-'A'+10;
  return -1;
}

// 라인 끝의 "*HH"를 검사하고 잘라낸다.
//  1  = 체크섬이 있고 일치      0 = 체크섬 없음      -1 = 있는데 불일치/형식오류
int8_t verifyCrc(char* line){
  char* star = NULL;
  for(char* p=line; *p; p++) if(*p=='*') star = p;
  if(!star) return 0;
  int8_t hi = hexVal(star[1]);
  int8_t lo = star[1] ? hexVal(star[2]) : -1;
  if(hi < 0 || lo < 0) return -1;
  uint8_t want = (uint8_t)((hi<<4) | lo);
  uint8_t got = 0;
  for(char* p=line; p<star; p++) got ^= (uint8_t)(*p);
  *star = 0;                       // 페이로드만 남긴다
  return (got == want) ? 1 : -1;
}

void handleLine(char* line){
  int8_t crc = verifyCrc(line);
  if(crc < 0){
    rxBad++;
    Serial.println(F("nak crc"));
    return;                        // 깨진 라인은 절대 실행하지 않는다
  }
  if(crc > 0) strictCrc = true;    // 정상 체크섬을 받은 순간부터 strict
  else if(strictCrc && line[0]=='V'){
    // strict 전환 뒤 무체크섬 속도지령 = 노이즈이거나 옛 클라이언트. 주행만 막고 알린다.
    rxBad++;
    Serial.println(F("nak nocrc"));
    return;
  }

  rxOk++;
  lastCmd = millis();
  char c = line[0];
  char* p = line+1;
  if(c=='V'){
    int vx=nextInt(p), vy=nextInt(p), w=nextInt(p);
    if(vx>255)vx=255; if(vx<-255)vx=-255;
    if(vy>255)vy=255; if(vy<-255)vy=-255;
    if(w >255)w =255; if(w <-255)w =-255;
    tgtVx=vx; tgtVy=vy; tgtW=w;
    testing = false;
    // 응답 없음 — 50Hz로 들어오므로 되쏘면 TX 버퍼가 차서 loop가 막힌다.
  } else if(c=='S'){
    hardStop(); Serial.println(F("ok S"));
  } else if(c=='T'){
    int i=nextInt(p), d=nextInt(p);
    if(i>=0 && i<4){
      hardStop();
      driveWheel(i, d>=0 ? 200 : -200);
      testing = true;
      testUntil = millis()+1000;
      Serial.print(F("ok T ")); Serial.print(i); Serial.print(' '); Serial.println(d);
    }
  } else if(c=='P'){
    int n=nextInt(p); if(n>0&&n<=4095){ MAX_PWM=n; }
    Serial.print(F("ok P ")); Serial.println(MAX_PWM);
  } else if(c=='L'){
    for(uint8_t i=0;i<4;i++){ int v=nextInt(p); POL[i] = (v<0)?-1:1; }
    hardStop();
    Serial.print(F("ok L POL=")); for(uint8_t i=0;i<4;i++){Serial.print((int)POL[i]);Serial.print(',');} Serial.println();
  } else if(c=='R'){
    int a=nextInt(p), d=nextInt(p);
    if(a>0 && a<=255) ACCEL=a;
    if(d>0 && d<=255) DECEL=d;
    Serial.print(F("ok R ")); Serial.print(ACCEL); Serial.print(' '); Serial.println(DECEL);
  } else if(c=='F'){
    int hz=nextInt(p);
    if(hz>=24 && hz<=1526){ faboPWM.set_hz(hz); applyCurrent(); }
    Serial.print(F("ok F ")); Serial.println(hz);
  } else if(c=='?'){
    Serial.print(F("st cur=")); Serial.print(curVx); Serial.print(','); Serial.print(curVy);
    Serial.print(','); Serial.print(curW);
    Serial.print(F(" MAX_PWM=")); Serial.print(MAX_PWM);
    Serial.print(F(" slew=")); Serial.print(ACCEL); Serial.print('/'); Serial.print(DECEL);
    Serial.print(F(" strict=")); Serial.print(strictCrc);
    Serial.print(F(" POL=")); for(uint8_t i=0;i<4;i++){Serial.print(POL[i]);Serial.print(',');}
    Serial.println();
  }
}

void setup(){
  wdt_disable();
  Serial.begin(115200);
  if(faboPWM.begin()){
    Serial.println(F("Find PCA9685 OK"));
    faboPWM.init(0);              // 부팅 순간 채널 전부 0 — v1의 init(300)은 기동 시 미세 통전
  } else {
    Serial.println(F("PCA9685 begin FALSE"));
  }
  faboPWM.set_hz(PWM_HZ_DEFAULT); // ★ 50 → 1500Hz. 저주파 "우우웅" 제거
  hardStop();
  lastCmd = lastTick = millis();
  Serial.println(F("mecanum_stable v2 ready. proto: V vx vy w | S | T i d | P n | L .. | R a d | F hz | ? (+*CRC)"));
  wdt_enable(WDTO_250MS);   // HW 워치독 (행 자동복구)
}

void loop(){
  wdt_reset();

  // 시리얼 수신 (논블로킹)
  while(Serial.available()){
    char ch = Serial.read();
    if(ch=='\n' || ch=='\r'){
      if(blen>0){ buf[blen]=0; handleLine(buf); blen=0; }
    } else if(blen < sizeof(buf)-1){
      buf[blen++]=ch;
    } else {
      blen = 0;   // 오버플로 라인은 통째로 버린다(깨진 프레임을 실행하지 않게)
    }
  }

  // T 테스트 자동정지
  if(testing && (long)(millis()-testUntil) >= 0){ hardStop(); Serial.println(F("T auto-stop")); }

  unsigned long now = millis();

  // 데드맨: 소프트(감속) → 하드(즉시 0)
  if(!testing){
    unsigned long silence = now - lastCmd;
    if(silence > HARD_TIMEOUT){
      if(curVx || curVy || curW){ hardStop(); Serial.println(F("deadman HARD")); }
    } else if(silence > CMD_TIMEOUT){
      tgtVx = tgtVy = tgtW = 0;   // 슬루로 부드럽게 선다 — v1의 "주웅 주웅" 끊김 제거
    }
  }

  // 슬루 틱: 목표로 조금씩 이동
  if(!testing && (now - lastTick) >= TICK_MS){
    lastTick = now;
    int nx = approach(curVx, tgtVx);
    int ny = approach(curVy, tgtVy);
    int nw = approach(curW,  tgtW);
    if(nx!=curVx || ny!=curVy || nw!=curW){
      curVx=nx; curVy=ny; curW=nw;
      applyCurrent();
    }
  }

  // 하트비트 1s — 링크 품질(수신/오류 카운터)과 실제 인가 속도를 젯슨에 보고
  if(now - lastHb > 1000){
    lastHb = now;
    Serial.print(F("hb ")); Serial.print(now);
    Serial.print(F(" rx=")); Serial.print(rxOk);
    Serial.print(F(" bad=")); Serial.print(rxBad);
    Serial.print(F(" v=")); Serial.print(curVx); Serial.print(',');
    Serial.print(curVy); Serial.print(','); Serial.println(curW);
  }
}

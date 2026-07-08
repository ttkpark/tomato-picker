// motor_test — PS2를 완전히 배제하고 시리얼로 메카넘 모터를 직접 구동하는 진단 스케치.
// 목적: 12V/PCA9685/모터드라이버/배선이 실제로 바퀴를 돌리는지 확인 (PS2 문제와 분리).
// baud 9600. 명령은 단일 문자. 모든 구동은 안전을 위해 일정 시간 후 자동 STOP(데드맨).
#include "FaBoPWM_PCA9685.h"

FaBoPWM faboPWM;

// 모터 채널 (Moebius 보드: PCA9685 채널 0~7)
#define DIRA1 0
#define DIRA2 1
#define DIRB1 2
#define DIRB2 3
#define DIRC1 4
#define DIRC2 5
#define DIRD1 6
#define DIRD2 7

int PWM = 800;                 // 기본 40% (최대 2000). +/- 로 조절
unsigned long autoStopAt = 0;  // 이 시각 지나면 자동 STOP (데드맨)

#define SET(ch,v) faboPWM.set_channel_value((ch),(v))
void mFwd(int a,int b){ SET(a,PWM); SET(b,0); }
void mBack(int a,int b){ SET(a,0); SET(b,PWM); }
void mStop(int a,int b){ SET(a,0); SET(b,0); }

void STOP(){ mStop(DIRA1,DIRA2); mStop(DIRB1,DIRB2); mStop(DIRC1,DIRC2); mStop(DIRD1,DIRD2); autoStopAt=0; }

// 원본 매핑 기준 전진(ADVANCE): A후진 B전진 C후진 D전진
void ADVANCE(){ mBack(DIRA1,DIRA2); mFwd(DIRB1,DIRB2); mBack(DIRC1,DIRC2); mFwd(DIRD1,DIRD2); }
void BACK(){    mFwd(DIRA1,DIRA2); mBack(DIRB1,DIRB2); mFwd(DIRC1,DIRC2); mBack(DIRD1,DIRD2); }

void arm(unsigned long ms){ autoStopAt = millis()+ms; }  // ms 후 자동정지 예약

void help(){
  Serial.println();
  Serial.println("=== motor_test (PS2 없음) ===");
  Serial.print("PWM="); Serial.println(PWM);
  Serial.println("f=전진  b=후진  s=정지");
  Serial.println("1/2/3/4 = 모터 A/B/C/D 단독 전진(1.2s)");
  Serial.println("!/@/#/$ = 모터 A/B/C/D 단독 후진(1.2s)");
  Serial.println("+/- = PWM ±200   h=도움말");
  Serial.println("(모든 구동은 안전상 시간 후 자동정지)");
}

void setup(){
  STOP();
  Serial.begin(9600);
  delay(50);
  Serial.println("boot: begin PCA9685...");
  if(faboPWM.begin()){
    Serial.println("Find PCA9685 OK");
    faboPWM.init(300);
  } else {
    Serial.println("PCA9685 begin() returned FALSE (I2C 문제?)");
  }
  faboPWM.set_hz(50);
  STOP();
  help();
}

void loop(){
  // 데드맨: 예약된 자동정지 시각 지나면 STOP
  if(autoStopAt && (long)(millis()-autoStopAt) >= 0){
    STOP();
    Serial.println("auto-STOP");
  }

  if(Serial.available()){
    char c = Serial.read();
    switch(c){
      case 'f': PWM=PWM; ADVANCE(); arm(1500); Serial.println("ADVANCE"); break;
      case 'b': BACK(); arm(1500); Serial.println("BACK"); break;
      case 's': STOP(); Serial.println("STOP"); break;
      case '1': STOP(); mBack(DIRA1,DIRA2); arm(1200); Serial.println("motor A fwd"); break;
      case '2': STOP(); mFwd(DIRB1,DIRB2);  arm(1200); Serial.println("motor B fwd"); break;
      case '3': STOP(); mBack(DIRC1,DIRC2); arm(1200); Serial.println("motor C fwd"); break;
      case '4': STOP(); mFwd(DIRD1,DIRD2);  arm(1200); Serial.println("motor D fwd"); break;
      case '!': STOP(); mFwd(DIRA1,DIRA2);  arm(1200); Serial.println("motor A back"); break;
      case '@': STOP(); mBack(DIRB1,DIRB2); arm(1200); Serial.println("motor B back"); break;
      case '#': STOP(); mFwd(DIRC1,DIRC2);  arm(1200); Serial.println("motor C back"); break;
      case '$': STOP(); mBack(DIRD1,DIRD2); arm(1200); Serial.println("motor D back"); break;
      case '+': PWM+=200; if(PWM>2000)PWM=2000; Serial.print("PWM="); Serial.println(PWM); break;
      case '-': PWM-=200; if(PWM<0)PWM=0; Serial.print("PWM="); Serial.println(PWM); break;
      case 'h': help(); break;
      default: break;
    }
  }
}

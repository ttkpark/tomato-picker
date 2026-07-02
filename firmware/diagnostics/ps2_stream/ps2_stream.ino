#include <PS2X_lib.h>
PS2X ps2x;
const uint16_t ALL[]={PSB_L1,PSB_L2,PSB_R1,PSB_R2,PSB_L3,PSB_R3,PSB_START,PSB_SELECT,
  PSB_PAD_UP,PSB_PAD_DOWN,PSB_PAD_LEFT,PSB_PAD_RIGHT,PSB_TRIANGLE,PSB_CIRCLE,PSB_CROSS,PSB_SQUARE};
bool ok=false; unsigned long last=0; long cnt=0;
void setup(){Serial.begin(115200);delay(400);
  ok=(ps2x.config_gamepad(12,11,10,13,false,false)==0);
  Serial.println(ok?"READY-STREAM":"ERR");}
void loop(){
  if(millis()-last<100)return; last=millis();
  if(!ok){ok=(ps2x.config_gamepad(12,11,10,13,false,false)==0); return;}
  ps2x.read_gamepad();
  uint8_t nb=0; for(uint8_t i=0;i<16;i++) if(ps2x.Button(ALL[i])) nb++;
  Serial.print("cnt=");Serial.print(cnt++);
  Serial.print(" L1=");Serial.print(ps2x.Button(PSB_L1));
  Serial.print(" L2=");Serial.print(ps2x.Button(PSB_L2));
  Serial.print(" nb=");Serial.print(nb);
  Serial.print(" LX=");Serial.print(ps2x.Analog(PSS_LX));
  Serial.print(" RX=");Serial.println(ps2x.Analog(PSS_RX));
}

#include <PS2X_lib.h>
PS2X ps2x;
struct B { uint16_t mask; const char* name; };
B buttons[] = {
  {PSB_L1,"L1"},{PSB_L2,"L2"},{PSB_R1,"R1"},{PSB_R2,"R2"},
  {PSB_PAD_UP,"UP"},{PSB_PAD_DOWN,"DOWN"},{PSB_PAD_LEFT,"LEFT"},{PSB_PAD_RIGHT,"RIGHT"},
  {PSB_TRIANGLE,"TRI"},{PSB_CIRCLE,"CIR"},{PSB_CROSS,"CRX"},{PSB_SQUARE,"SQR"},
  {PSB_SELECT,"SELECT"},{PSB_START,"START"},{PSB_L3,"L3"},{PSB_R3,"R3"}
};
bool padOk=false; unsigned long last=0, hb=0;
bool corrupt(){int a=ps2x.Analog(PSS_LX),b=ps2x.Analog(PSS_LY),c=ps2x.Analog(PSS_RX),d=ps2x.Analog(PSS_RY);
  return (a==255&&b==255&&c==255&&d==255)||(a==0&&b==0&&c==0&&d==0);}
void setup(){Serial.begin(115200);delay(400);
  padOk=(ps2x.config_gamepad(12,11,10,13,false,false)==0);
  Serial.println(padOk?"READY (press the L button)":"ERR");}
void loop(){
  if(millis()-last<70)return; last=millis();
  if(!padOk){return;}
  ps2x.read_gamepad();
  bool bad=corrupt();
  String s="";
  for(auto&bt:buttons) if(ps2x.Button(bt.mask)){ s+=bt.name; s+=" "; }
  if(s.length()) { Serial.print(bad?"[corrupt?] PRESSED: ":"PRESSED: "); Serial.println(s); }
  if(millis()-hb>1500){hb=millis();
    Serial.print("hb analogs LX=");Serial.print(ps2x.Analog(PSS_LX));
    Serial.print(" LY=");Serial.print(ps2x.Analog(PSS_LY));
    Serial.print(" corrupt=");Serial.println(bad);}
}

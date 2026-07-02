/*
 * boot_probe — setup() 각 단계마다 시리얼 표식을 찍어, PCA9685/I2C 초기화가
 * 어디서 멈추는지(hang) 확인하기 위한 최소 진단 스케치.
 *
 * 이 AVR 코어(arduino:avr 1.8.8)의 Wire 라이브러리는 I2C 버스 타임아웃이 없어서,
 * PCA9685가 전원이 안 들어왔거나 배선이 안 맞으면 faboPWM.begin()이 무한 대기한다.
 * BOOT2 이후 줄이 안 찍히면 그게 원인이다 — PCA9685 전원(VCC/V+)과 I2C(A4=SDA,
 * A5=SCL) 배선을 확인할 것.
 */
#include <PS2X_lib.h>
#include "FaBoPWM_PCA9685.h"
FaBoPWM faboPWM;
PS2X ps2x;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("BOOT0-serial-ok");

  Serial.println("BOOT1-before-faboPWM.begin");
  bool found = faboPWM.begin();
  Serial.print("BOOT2-faboPWM.begin-returned="); Serial.println(found);

  if (found) {
    faboPWM.init(300);
    Serial.println("BOOT3-faboPWM.init-done");
  }
  faboPWM.set_hz(50);
  Serial.println("BOOT4-set_hz-done");

  int e = ps2x.config_gamepad(12,11,10,13,false,false);
  Serial.print("BOOT5-ps2-config-err="); Serial.println(e);
  Serial.println("BOOT6-setup-complete");
}
void loop() {
  static unsigned long last=0;
  if (millis()-last>1000) { last=millis(); Serial.println("alive"); }
}

#include <PS2X_lib.h>
PS2X ps2x;
struct Combo { uint8_t clk, cmd, sel, dat; };
// (CLK, CMD, SEL/ATT, DAT) 흔한 배선 후보들
Combo combos[] = {
  {13,11,10,12}, {10,11,12,13}, {8,9,10,11}, {11,10,9,8},
  {13,12,11,10}, {12,13,14,15}, {14,15,16,17}, {17,16,15,14},
  {2,4,7,8},     {8,7,4,2},     {9,8,7,6},     {6,7,8,9},
  {3,5,6,9},     {7,8,12,13},   {13,12,8,7},   {10,12,11,13},
  {18,17,16,19}, {15,14,13,12}, {12,11,8,7},   {9,10,11,12}
};
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("=== PS2 PIN PROBE ===");
  for (auto &c : combos) {
    int e = ps2x.config_gamepad(c.clk, c.cmd, c.sel, c.dat, false, false);
    Serial.print("CLK="); Serial.print(c.clk);
    Serial.print(" CMD="); Serial.print(c.cmd);
    Serial.print(" SEL="); Serial.print(c.sel);
    Serial.print(" DAT="); Serial.print(c.dat);
    Serial.print(" -> err="); Serial.print(e);
    if (e == 0) Serial.print("   <<< FOUND");
    Serial.println();
    delay(120);
  }
  Serial.println("=== DONE ===");
}
void loop() {}

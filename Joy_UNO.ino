#include "ABU_67_2.h"

ABU_Joy Joy;

// Joy PS2;
byte error_PS = 1;
byte type_PS = 0;

#define clk 7
#define cmd 5
#define att 4
#define dat 6

#define pin_G1 A0
#define pin_G2 A1
#define pin_G3 A2

#define button_speedUP 0
#define button_speedDOWN 2

#ifdef ARDUINO_AVR_UNO

void setup() {
  Serial.begin(115200);
  delay(150);
  Joy.Setup_Joy_nRF24(8, 9, 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL);
  Joy.Setup_PS2(clk, cmd, att, dat);
  Joy.Setup_GEAR_joy(pin_G1, pin_G2, pin_G3);
}

void loop() {
  error_PS = Joy.PS2_error();
  type_PS = Joy.PS2_type();
  if (error_PS == 1) {
    Joy.Setup_PS2(clk, cmd, att, dat);
    return;
  }
  if (type_PS != 2) {
    Joy.set_PS2_0();
    Joy.PS2_readValue();
    Joy.Print_GEAR(button_speedUP, button_speedDOWN);
    Joy.print_PS2();
    Joy.Joy_Sendvalue();
  }
}

#endif
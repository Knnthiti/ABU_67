#ifndef ABU_67_2
#define ABU_67_2

#include <Arduino.h>

//PS2
#include <PS2X_lib.h>

//nrf24
#include <RF24.h>

// #define DEBUG_PS2
// #define DEBUG_Wheel
// #define DEBUG_Clamp
#define DEBUG_Shoot
// #define DEBUG_Mega

class Joy : public PS2X {
public:
  boolean move[4];
  boolean attack[8];
  boolean seting[2];
  float stickValues[4];
  uint8_t error = 1;
  uint8_t type = 0;
  uint8_t vibrate = 0;

  struct ControllerData {
    boolean move[4];
    boolean attack[8];
    boolean seting[2];
    float stickValues[4];
  };
  ControllerData Str_PS2;

  Joy() {  // Default constructor
  }

  void structToArray();
  void Setup_PS2(uint8_t _clk, uint8_t _cmd, uint8_t _att, uint8_t _dat);
  uint8_t PS2_error();
  uint8_t PS2_type();
  void set_PS2_0();
  void PS2_readValue();
  void print_PS2();
};

class ABU_Joy : public Joy, public RF24 {
public:
  uint64_t pipes[2];
  int8_t Gear_joy = 0;
  uint8_t pin_G1;
  uint8_t pin_G2;
  uint8_t pin_G3;

  void Setup_Joy_nRF24(rf24_gpio_pin_t CE, rf24_gpio_pin_t CNS, const uint64_t pipes_0, const uint64_t pipes_1);
  void Setup_GEAR_joy(uint8_t _pin_G1, uint8_t _pin_G2, uint8_t _pin_G3){
    pin_G1 = _pin_G1;
    pin_G2 = _pin_G2;
    pin_G3 = _pin_G3;

   pinMode(pin_G1 ,OUTPUT);
   pinMode(pin_G2 ,OUTPUT);
   pinMode(pin_G3 ,OUTPUT);
  }
  void Print_GEAR(uint8_t button_speedUP, uint8_t button_speedDOWN);
  void Joy_Sendvalue();
};


class motor {
public:
  boolean digital_Setup[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  void ledcSetup_mega2560(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits);
  void motorDrive_BTS7960(uint8_t pin_PWM, int32_t duty, uint8_t pin_LPWM, uint8_t pin_RPWM);
  void motorDrive_BTS7960_DC(uint8_t pin_LPWM, uint8_t pin_RPWM, int32_t duty);
};

class Wheel : public motor {
public:
  float ROBOT_TOTAL_LEN;
  float Wheels_redius;

  float Wheels[4] = { 0, 0, 0, 0 };

  uint8_t pin_PWM[4];   //{LF_pin ,LB_pin ,RF_pin ,RB_pin}
  uint8_t pin_LPWM[4];  //{LF_pin ,LB_pin ,RF_pin ,RB_pin}
  uint8_t pin_RPWM[4];  //{LF_pin ,LB_pin ,RF_pin ,RB_pin}

  float SCALE_FACTOR[3];  //{SCALE_FACTOR_X ,SCALE_FACTOR_Y ,SCALE_FACTOR_Z}

  int8_t Gear = 0;
  float SCALE_FACTOR_Gear_1;
  float SCALE_FACTOR_Gear_2;
  float SCALE_FACTOR_Gear_3;

  float max_wheel_speed[4] = {0.0, 150.0, 170.0, 190.0};

  void Setup_LF_pin(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM);
  void Setup_LB_pin(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM);
  void Setup_RF_pin(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM);
  void Setup_RB_pin(uint8_t pin_PWM, uint32_t freq, uint8_t resolution_bits, uint8_t _pin_LPWM, uint8_t _pin_RPWM);

  void Setup_Scale_factor(float SCALE_FACTOR_X, float SCALE_FACTOR_Y, float SCALE_FACTOR_Z) {
    SCALE_FACTOR[0] = SCALE_FACTOR_X;
    SCALE_FACTOR[1] = SCALE_FACTOR_Y;
    SCALE_FACTOR[2] = SCALE_FACTOR_Z;
  }

  void Setup_Wheel(float _ROBOT_LEN, float _ROBOT_WID, float _Wheels_redius) {
    ROBOT_TOTAL_LEN = _ROBOT_LEN + _ROBOT_WID;
    Wheels_redius = _Wheels_redius;
  }
  void Wheel_equation(float _scaled[]);

  void Setup_max_speed(float Gear_0, float Gear_1, float Gear_2, float Gear_3){
    max_wheel_speed[0] = Gear_0;
    max_wheel_speed[1] = Gear_1;
    max_wheel_speed[2] = Gear_2;
    max_wheel_speed[3] = Gear_3;
  }

  void GEAR_Setup(float _SCALE_FACTOR_Gear_1, float _SCALE_FACTOR_Gear_2, float _SCALE_FACTOR_Gear_3) {
    SCALE_FACTOR_Gear_1 = _SCALE_FACTOR_Gear_1;
    SCALE_FACTOR_Gear_2 = _SCALE_FACTOR_Gear_2;
    SCALE_FACTOR_Gear_3 = _SCALE_FACTOR_Gear_3;
  }
  void GEAR();
};

class ABU_Car : public Joy, public RF24, public Wheel {
public:
  uint64_t pipes[2];

  long Timeactivate = 0;
  boolean Ready = 0;

  boolean neumatic = 0;  //{Clamp_L,Clamp_R}
  boolean Clamp_UP = 0;  //{ Clamp_up==1 , Clamp_down==0 }
  boolean Clamp_DOWN = 0;
  boolean Clamp_GRAD = 0;  //{ Clamp_in==0 , Clamp_out==1 }
  boolean Clamp_POLL = 0;

  //pneumatic นีบต้นกล้า_ซ้าย หุบ-คลาย
  uint8_t Clamp_L;

  //pneumatic นีบต้นกล้า_ขวา หุบ-คลาย
  uint8_t Clamp_R;

  //driver motor นีบต้นกล้า ขึ้น-ลง
  uint8_t Clamp_up;
  uint8_t Clamp_down;
  uint8_t Set_Clamp_up;    //limit switch
  uint8_t Set_Clamp_down;  //limit switch

  //driver motor นีบต้นกล้า เข้า-ออก
  uint8_t Clamp_grab;
  uint8_t Clamp_poll;
  uint8_t Set_Clamp_grab;  //limit switch
  uint8_t Set_Clamp_poll;  //limit switch

  //driver motor keep ball
  uint8_t pin_keep;
  uint8_t Keep_Ball;
  uint8_t UnKeep_Ball;
  uint8_t Set_UnKeep_Ball;  //limit switch
  uint8_t KEEP_BALL = 0;

  //driver motor shoot ball
  uint8_t pin_Shoot;
  uint8_t Attack_Ball;
  uint8_t Reload_Ball;
  uint8_t Set_Attack_Ball;  //limit switch
  uint8_t Set_Reload_Ball;  //limit switch

  ABU_Car() {  // Default constructor
  }

  void Setup_Car_nRF24(rf24_gpio_pin_t CE, rf24_gpio_pin_t CNS, const uint64_t pipes_0, const uint64_t pipes_1);
  void Readvalue();
  void Move();

  void Keep_Clamp_Setup(uint8_t _Clamp_L, uint8_t _Clamp_R) {
    Clamp_L = _Clamp_L;
    Clamp_R = _Clamp_R;

    pinMode(Clamp_L, OUTPUT);
    pinMode(Clamp_R, OUTPUT);
  }
  void UP_DOWN_Clamp_Setup(uint8_t _Clamp_up, uint8_t _Clamp_down, uint8_t _Set_Clamp_up, uint8_t _Set_Clamp_down) {
    Clamp_up = _Clamp_up;
    Clamp_down = _Clamp_down;
    Set_Clamp_up = _Set_Clamp_up;
    Set_Clamp_down = _Set_Clamp_down;

    pinMode(Clamp_up, OUTPUT);
    pinMode(Clamp_down, OUTPUT);
    pinMode(Set_Clamp_up, INPUT_PULLUP);
    pinMode(Set_Clamp_down, INPUT_PULLUP);
  }
  void Grab_Poll_Clamp_Setup(uint8_t _Clamp_grab, uint8_t _Clamp_poll, uint8_t _Set_Clamp_grab, uint8_t _Set_Clamp_poll) {
    Clamp_grab = _Clamp_grab;
    Clamp_poll = _Clamp_poll;
    Set_Clamp_grab = _Set_Clamp_grab;
    Set_Clamp_poll = _Set_Clamp_poll;

    pinMode(Clamp_grab, OUTPUT);
    pinMode(Clamp_poll, OUTPUT);
    pinMode(Set_Clamp_grab, INPUT_PULLUP);
    pinMode(Set_Clamp_poll, INPUT_PULLUP);
  }
  void CLAMP(uint8_t Clamp, uint8_t UP_Clamp, uint8_t DOWN_Clamp, uint8_t Grab_Clamp, uint8_t Poll_Clamp);

  void KEEP_Ball_Setup(uint8_t _pin_keep, uint8_t _Keep_Ball, uint8_t _UnKeep_Ball, uint8_t _Set_UnKeep_Ball) {
    pin_keep = _pin_keep;
    Keep_Ball = _Keep_Ball;
    UnKeep_Ball = _UnKeep_Ball;
    Set_UnKeep_Ball = _Set_UnKeep_Ball;

    ledcSetup_mega2560(pin_keep, 8000, 8);
    pinMode(Set_UnKeep_Ball, INPUT_PULLUP);
  }
  void KEEP_Ball(uint8_t button_keep, uint8_t button_UnKeep, int32_t force);


  void Shoot_Ball_Setup(uint8_t _pin_Shoot, uint8_t _Attack_Ball, uint8_t _Reload_Ball, uint8_t _Set_Attack_Ball, uint8_t _Set_Reload_Ball) {
    pin_Shoot = _pin_Shoot;
    Attack_Ball = _Attack_Ball;
    Reload_Ball = _Reload_Ball;
    Set_Attack_Ball = _Set_Attack_Ball;
    Set_Reload_Ball = _Set_Reload_Ball;

    ledcSetup_mega2560(pin_Shoot, 8000, 8);
    pinMode(Set_Attack_Ball, INPUT_PULLUP);
    pinMode(Set_Reload_Ball, INPUT_PULLUP);
  }
  void Shoot_Ball(uint8_t button_Shoot, uint8_t button_UP, uint8_t button_Reload, int32_t force_Shoot, int32_t force_UP, int32_t force_Reload);

  void UP_speed(uint8_t button_speedUP, uint8_t button_speedDOWN);
};
#endif
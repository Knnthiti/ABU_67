#include "ABU_67_2.h"
ABU_Car robot;

  //Timer_2
#define PWM_LF 10   //A      
#define IN1_LF 40
#define IN2_LF 38
  //Timer_1
#define PWM_LB  11   //A
#define IN1_LB  36
#define IN2_LB  34

#define PWM_RB  13   //B
#define IN1_RB  26
#define IN2_RB  28

#define PWM_RF 12   //C
#define IN1_RF 30
#define IN2_RF 32

#define ROBOT_LEN 0.41f       // Example value, adjust according to your robot's specifications
#define ROBOT_WID 0.41f       // Example value, adjust according to your robot's specifications
#define SCALE_FACTOR_X 0.09f  // Example value, adjust according to your requirements
#define SCALE_FACTOR_Y 0.09f  // Example value, adjust according to your requirements
#define SCALE_FACTOR_Z 0.09f  // Example value, adjust according to your requirements
#define WHEEL_RADIUS 0.06f    // Example value, adjust according to your robot's specifications

float SCALE_FACTOR_Gear_1 = 0.09f;
float SCALE_FACTOR_Gear_2 = 0.10f;
float SCALE_FACTOR_Gear_3 = 0.11f;

#define button_speedUP 0
#define button_speedDOWN 2

#define Clamp_L 23
#define Clamp_R 25

#define Clamp_up 44
#define Clamp_down 42
#define Set_Clamp_up 37
#define Set_Clamp_down 35

#define Clamp_grab 48
#define Clamp_poll 46
#define Set_Clamp_grab A9
#define Set_Clamp_poll A8

#define pin_keep 7
#define Keep_Ball A4
#define UnKeep_Ball A5 
#define Set_UnKeep_Ball 31
  
#define pin_Shoot 6
#define Attack_Ball A2
#define Reload_Ball A3
#define Set_Attack_Ball A6
#define Set_Reload_Ball A7

#define Clamp 2
#define UP_Clamp 4
#define DOWN_Clamp 6
#define Grab_Clamp 5
#define Poll_Clamp 7

#define button_keep 3
#define button_UnKeep 1
#define force 200

#define button_Shoot 0
#define button_UP 1
#define button_Reload 3
#define force_Shoot 230
#define force_UP 150
#define force_Reload 150

#ifdef __AVR_ATmega2560__

void setup() {
  Serial.begin(115200);
  robot.Setup_Car_nRF24(A1 ,A0 ,0xF0F0F0F0E1LL ,0xF0F0F0F0D2LL);
  
  // car.Setup_Scale_factor(SCALE_FACTOR_X ,SCALE_FACTOR_Y ,SCALE_FACTOR_Z);
  robot.GEAR_Setup(SCALE_FACTOR_Gear_1 ,SCALE_FACTOR_Gear_2 ,SCALE_FACTOR_Gear_3);
  robot.Setup_max_speed(0.0, 185.0, 205.0, 225.0);

  robot.Setup_LF_pin(PWM_LF ,8000 ,8 ,IN1_LF ,IN2_LF );
  robot.Setup_LB_pin(PWM_LB ,8000 ,8 ,IN1_LB ,IN2_LB );
  robot.Setup_RF_pin(PWM_RF ,8000 ,8 ,IN1_RF ,IN2_RF );
  robot.Setup_RB_pin(PWM_RB ,8000 ,8 ,IN1_RB ,IN2_RB );
  
  robot.Setup_Wheel(ROBOT_LEN ,ROBOT_WID ,WHEEL_RADIUS);

  robot.Keep_Clamp_Setup(Clamp_L ,Clamp_R);
  robot.UP_DOWN_Clamp_Setup(Clamp_up ,Clamp_down ,Set_Clamp_up ,Set_Clamp_down);
  robot.Grab_Poll_Clamp_Setup(Clamp_grab ,Clamp_poll ,Set_Clamp_grab ,Set_Clamp_poll);

  robot.KEEP_Ball_Setup(pin_keep ,Keep_Ball ,UnKeep_Ball ,Set_UnKeep_Ball);
  
  robot.Shoot_Ball_Setup(pin_Shoot ,Attack_Ball ,Reload_Ball ,Set_Attack_Ball ,Set_Reload_Ball);
}

void loop() {
  robot.Readvalue();

  robot.UP_speed(button_speedUP ,button_speedDOWN);
  robot.Move();

  robot.CLAMP(Clamp ,UP_Clamp ,DOWN_Clamp ,Grab_Clamp ,Poll_Clamp);

  robot.Shoot_Ball(button_Shoot ,button_UP ,button_Reload ,force_Shoot ,force_UP ,force_Reload);

  robot.KEEP_Ball(button_keep ,button_UnKeep ,force);

  delay(20);
}
#endif
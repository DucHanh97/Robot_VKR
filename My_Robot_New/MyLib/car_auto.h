#ifndef CAR_AUTO_H
#define CAR_AUTO_H
#include "main.h"
#include "servo.h"

/* ----- Define gains of PID controler ----- */
#define Kp			80
#define Ki			0
#define Kd			0
#define BASE_SPEED		30
/* ----------------------------------------- */

void car_auto_state_switch(Servo *ServoHcsr04, float distance);
void car_auto_state_handle(Servo *ServoHcsr04, float distance);
void arm_set_cmd(int8_t argv[6]);
void car_check_side(Servo *ServoHcsr04, float distance);
void car_obstacle(void);
#endif

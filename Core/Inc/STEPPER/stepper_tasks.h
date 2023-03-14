/*
 * stepper_tasks.h
 *
 *  Created on: Mar 7, 2023
 *      Author: saveasmtz
 */

#ifndef INC_STEPPER_STEPPER_TASKS_H_
#define INC_STEPPER_STEPPER_TASKS_H_

#include "main.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include <math.h>

extern const float MAX_STEERING; // Degrees
extern const float STEP_ANGLE; // Degrees
extern const float STEPS_REV;  // Steps per revolution
extern int dir_1;				 //
extern int goal_steps_1;		 // Required steps to reach desired angle
extern int steps_1;			  // Steps of motor 1
extern int current_step_1;
extern float desired_angle_1; // Degrees
extern float current_angle_1; // Degrees
extern TIM_HandleTypeDef htim2;

void steering_task_pwm(void *argument);

#endif /* INC_STEPPER_STEPPER_TASKS_H_ */

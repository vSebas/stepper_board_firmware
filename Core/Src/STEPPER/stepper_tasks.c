
#include <STEPPER/stepper_tasks.h>

const float MAX_STEERING = 57.3; // Degrees
const float STEP_ANGLE = 1.8; // Degrees
const float STEPS_REV = 200;  // Steps per revolution
int dir_1 = 1;				 //
int goal_steps_1 = 0;		 // Required steps to reach desired angle
int steps_1 = 0;			  // Steps of motor 1
int current_step_1 = 0;
float desired_angle_1 = 45; // Degrees
float current_angle_1 = 0; // Degrees

/* USER CODE BEGIN Header_steering_task_pwm */
/**
* @brief Function implementing the steeringTaskPWM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_steering_task_pwm */
void steering_task_pwm(void *argument)
{
  /* USER CODE BEGIN steering_task_pwm */
  /* Infinite loop */
  for(;;)
  {
		int angle_d = (int) desired_angle_1;
		dir_1 = abs(angle_d)/angle_d;

		if(fabs(desired_angle_1) > MAX_STEERING)
			desired_angle_1 = MAX_STEERING;

		desired_angle_1 *= dir_1; // to work only with positive numbers
		goal_steps_1 = desired_angle_1/STEP_ANGLE;

		HAL_GPIO_WritePin(GPIOC, STPR_DIR_1_Pin | STPR_EN_1_Pin, dir_1);

		if(steps_1 < goal_steps_1)
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		else
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  }
  /* USER CODE END steering_task_pwm */
}

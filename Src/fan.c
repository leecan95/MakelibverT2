/* Author: TTTBYT
 *
 * Brief: ERV Motor Control.
 * Two fan: FAN1 & FAN2 (2 signal wire: PWM-speed cotrol and TACH-speed feedback (2 or 3 ppr) )
 * Pin connect:
 * 		FAN1_TACH: PA6 (TIM3_CH1_IC)
 * 		FAN2_TACH: PA7 (TIM3_CH2_IC)
 
 * 		FAN1_PWM:  PA8 (TIM1_CH1_PWM)
 * 		FAN2_PWM:  PA11(TIM1_CH4_PWM)
 *
 * f(STM32F4) = 84MHz ; f(PWM) = 10KHz ; Counter Period = 100 -> Prescaler = 84MHz/(10KHz*100) = 84
 * 
 */
#include "fan.h"

//Define tuong tu main.h
#define lowSp		20
#define medSp		40
#define highSp		50
int current_fan1_speed = lowSp;		//Luu toc do hien tai cua dong co 1, ban dau: off
int current_fan2_speed = lowSp;		//Luu toc do hien tai cua dong co 2, ban dau: off
int softStartTime = 30;

void fan_init(){
  //Copy from CubeMX
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  //if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  //{
  //  Error_Handler();
 // }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
  //End copy from CubeMX
  
  /* PWM1 and PWM2 set */
 // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  fan_1_stop();
  fan_2_stop();
  current_fan1_speed = 0;
  current_fan2_speed = 0;
}


void fan_1_stop(){
//  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); dau ra bi floating point
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);	//Do thiet ke mach dang de keo len
	current_fan1_speed = lowSp;
}

void fan_1_run(uint8_t level){
  if(level != current_fan1_speed){		//Neu truyen vao toc do khac voi hien tai
	  if(level > current_fan1_speed){	//truyen vao > hien tai
		  for(int i = current_fan1_speed; i < level; i++){
			//  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100 - i);
			  HAL_Delay(softStartTime);
		  }
	  }
	  else {							//truyen vao < hien tai
		  for(int i = current_fan1_speed; i > level; i--){
			//  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100 - i);
			  HAL_Delay(softStartTime);
		  }
	  }
  }

  current_fan1_speed = level;
}

void fan_2_stop(){
//	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);
	current_fan2_speed = lowSp;
}

void fan_2_run(uint8_t level){
	if(level != current_fan2_speed){		//Neu truyen vao toc do khac voi hien tai
		  if(level > current_fan2_speed){	//truyen vao > hien tai
			  for(int i = current_fan2_speed; i < level; i++){
				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100 - i);
				  HAL_Delay(softStartTime);
			  }
		  }
		  else {							//truyen vao < hien tai
			  for(int i = current_fan2_speed; i > level; i--){
				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100 - i);
				  HAL_Delay(softStartTime);
			  }
		  }
	  }

	  current_fan2_speed = level;
}

void fan_12_run(uint8_t level){
  if(level != current_fan1_speed){		//Neu truyen vao toc do khac voi hien tai
	  if(level > current_fan1_speed){	//truyen vao > hien tai
		  for(int i = current_fan1_speed; i < level; i++){
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100 - i);
			 // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100 - i);
			  HAL_Delay(softStartTime);
		  }
	  }
	  else {							//truyen vao < hien tai
		  for(int i = current_fan1_speed; i > level; i--){
			 // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100 - i);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100 - i);
			  HAL_Delay(softStartTime);
		  }
	  }
  }

  current_fan1_speed = level;
  current_fan2_speed = level;
}

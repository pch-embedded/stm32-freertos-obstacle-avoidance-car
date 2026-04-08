/* motor.c */
#include "motor.h"

static TIM_HandleTypeDef *s_htim = 0;
static const uint16_t speed_table[3] = {0, 600, 950};

void Motor_Init(TIM_HandleTypeDef *htim)
{
    if (!htim) return;
    s_htim = htim;
    HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_2);
}

void Motor_Run(uint8_t step)
{
    if (!s_htim) return;

    /* 방향핀 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

    uint16_t duty = (step < 3) ? speed_table[step] : 0;

    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, duty);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_2, duty);
}

void Motor_Stop(void)
{
    if (!s_htim) return;

    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_2, 0);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
}
void Motor_Back(uint8_t step)
{
	if(!s_htim) return;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

	uint16_t duty =(step<3) ? speed_table[step] : 0;

	 __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, duty);
	 __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_2, duty);
}

void Motor_TurnLeft(uint8_t step)
{
	if (!s_htim) return;
	/* 왼쪽 바퀴 정지, 오른쪽 바퀴 전진*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	 uint16_t duty = (step <3) ? speed_table[step] : 0;

	 __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, 0);
	 __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_2, duty);
}

void Motor_TurnRight(uint8_t step) {
	if (!s_htim) return;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	uint16_t duty = (step < 3) ? speed_table[step] : 0;

	__HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, duty);  // 한쪽 바퀴 전진
	__HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_2, 0);     // 다른쪽 바퀴 정지
}

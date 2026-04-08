/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "tim.h"
#include "app_state.h"
#include "motor.h"
#include "ultrasonic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 외부 풀다운 기준: pressed=1(SET), released=0(RESET) */
#define BTN_PORT          GPIOC
#define BTN_PIN           GPIO_PIN_13

#define BTN_DEBOUNCE_MS   (30u)
#define US_NEAR_ON_CM    (18u)
#define US_NEAR_OFF_CM   (22u)
#define SCAN_SAFE_CM (25u)
#define SERVO_CENTER 1500
#define SERVO_LEFT   2000
#define SERVO_RIGHT   1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* ---- Ultrasonic debug ---- */
volatile uint32_t us_rise_cnt;
volatile uint32_t us_fall_cnt;
volatile uint32_t us_rise_us;
volatile uint32_t us_pulse_us;
volatile UsEchoEdgeMsg us_last;
volatile uint32_t us_last_seq;
volatile uint32_t us_dist_cm=0;
volatile uint8_t  us_is_near;
volatile uint32_t scan_center_cm =0;
volatile uint32_t scan_left_cm=0;
volatile uint32_t scan_right_cm=0;
volatile uint8_t scan_dir=0; // 0==none, 1=left,2=right
volatile uint8_t scan_done = 0u;



/* ---- Button debug (optional) ---- */
volatile uint32_t dbg_btn_msg    = 0;
volatile uint32_t dbg_edge_tick  = 0;
volatile uint32_t dbg_edge_level = 0;
volatile uint32_t dbg_press_tick = 0;
volatile uint32_t dbg_dur_ms     = 0;

/* control debug */
volatile uint8_t ctrl_want_run = 0u;
volatile uint8_t auto_speed_step = 0u;



/* ---- Queues ---- */
osMessageQueueId_t btnEdgeQHandle;
const osMessageQueueAttr_t btnEdgeQ_attributes = { .name = "btnEdgeQ" };

osMessageQueueId_t motorCmdQHandle;
const osMessageQueueAttr_t motorCmdQ_attributes = { .name = "motorCmdQ" };

osMessageQueueId_t usEchoEdgeQHandle;
const osMessageQueueAttr_t usEchoEdgeQ_attributes = { .name = "usEchoEdgeQ" };
/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN RTOS_THREADS_DECL */
osThreadId_t ButtonTaskHandle;
const osThreadAttr_t ButtonTask_attributes = {
  .name = "ButtonTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};

osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t UltrasonicEdgeTaskHandle;
const osThreadAttr_t UltrasonicEdgeTask_attributes = {
  .name = "UltrasonicEdgeTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t UltrasonicTrigTaskHandle;
const osThreadAttr_t UltrasonicTrigTask_attributes = {
  .name = "UltrasonicTrigTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
		.name = "ControlTask",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityNormal,
		};
/* USER CODE END RTOS_THREADS_DECL */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void delay_us(uint32_t us);

void ButtonTask(void *argument);
void MotorTask(void *argument);
void ControlTask(void *argument);

void UltrasonicEdgeTask(void *argument);
void UltrasonicTrigTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* Create queues (must be after osKernelInitialize(), before osKernelStart()) */
  btnEdgeQHandle    = osMessageQueueNew(16, sizeof(uint32_t),   &btnEdgeQ_attributes);
  motorCmdQHandle   = osMessageQueueNew(8,  sizeof(MotorCmd_t), &motorCmdQ_attributes);
  usEchoEdgeQHandle = osMessageQueueNew(16, sizeof(UsEchoEdgeMsg), &usEchoEdgeQ_attributes);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  ButtonTaskHandle         = osThreadNew(ButtonTask, NULL, &ButtonTask_attributes);
  MotorTaskHandle          = osThreadNew(MotorTask,  NULL, &MotorTask_attributes);
  UltrasonicEdgeTaskHandle = osThreadNew(UltrasonicEdgeTask, NULL, &UltrasonicEdgeTask_attributes);
  UltrasonicTrigTaskHandle = osThreadNew(UltrasonicTrigTask, NULL, &UltrasonicTrigTask_attributes);
  ControlTaskHandle = osThreadNew(ControlTask, NULL, &ControlTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void delay_us(uint32_t us)
{
  uint32_t start = __HAL_TIM_GET_COUNTER(&htim5);
  while ((uint32_t)(__HAL_TIM_GET_COUNTER(&htim5) - start) < us) {
    /* busy wait */
  }
}

/* ButtonTask: btnEdgeQ( packed tick/level ) -> motorCmdQ */
void ButtonTask(void *argument)
{
  (void)argument;

  uint32_t packed = 0;
  uint32_t press_tick = 0;
  uint8_t  pressed = 0;

  for (;;)
  {
    osMessageQueueGet(btnEdgeQHandle, &packed, NULL, osWaitForever);

    uint32_t isr_tick = (packed >> 1);
    uint32_t isr_level = (packed & 1u);

    dbg_btn_msg    = packed;
    dbg_edge_tick  = isr_tick;
    dbg_edge_level = isr_level;

    /* debounce: wait then re-read */
    osDelay(BTN_DEBOUNCE_MS);
    uint32_t stable_level =
        (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_SET) ? 1u : 0u;

    if (stable_level != isr_level) {
      continue; /* bounce/glitch */
    }

    if (stable_level == 1u) {
      /* pressed */
      pressed = 1;
      press_tick = isr_tick;
      dbg_press_tick = press_tick;
    } else {
      /* released */
      if (!pressed) continue;
      pressed = 0;

      uint32_t rel_tick = osKernelGetTickCount();
      uint32_t dt_ticks = (rel_tick - press_tick);
      uint32_t tick_hz = osKernelGetTickFreq();
      uint32_t dur_ms = (tick_hz != 0u) ? (dt_ticks * 1000u) / tick_hz : dt_ticks;
      dbg_dur_ms = dur_ms;
      stop_flag ^= 1u;


    }
  }
}

/* MotorTask: exclusive motor control */
void MotorTask(void *argument)
{
  (void)argument;

  /* safe boot */
  Motor_Stop();

  MotorCmd_t cmd;
  for (;;)
  {
    osMessageQueueGet(motorCmdQHandle, &cmd, NULL, osWaitForever);

    if (cmd.type == MOTOR_CMD_STOP) {
      Motor_Stop();
    }
    else if (cmd.type == MOTOR_CMD_RUN){
    	Motor_Run(cmd.speed_step);
    }
    else if (cmd.type == MOTOR_CMD_BACK) {
    	Motor_Back(cmd.speed_step);
    }
  }
}

/* UltrasonicEdgeTask: consume EXTI edge messages from main.c callback */
void UltrasonicEdgeTask(void *argument)
{
  (void)argument;
  UsEchoEdgeMsg msg;

  for (;;)
  {
    osMessageQueueGet(usEchoEdgeQHandle, &msg, NULL, osWaitForever);

    if (msg.level) {
      us_rise_cnt++;
      us_rise_us = msg.t_us;
    } else {
      us_fall_cnt++;
      us_pulse_us = (uint32_t)(msg.t_us - us_rise_us);
      if (us_pulse_us >= 150u && us_pulse_us <= 25000u) {
    	  us_dist_cm = us_pulse_us / 58u;
    	  if(us_dist_cm<=US_NEAR_ON_CM) {
    		  us_is_near=1u;
    	  }
    	  else if(us_dist_cm>=US_NEAR_OFF_CM) {
    		  us_is_near=0u;
    	  }
        }
    }

    us_last = msg;
    us_last_seq++;
  }
}

/* UltrasonicTrigTask: periodic trigger pulse */
void UltrasonicTrigTask(void *argument)
{
  (void)argument;

  /* TRIG default LOW */
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  for (;;)
  {
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

    /* avoid echo overlap */
    osDelay(60);
  }
}

static void ScanPracticeOnce(void)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CENTER);
	osDelay(500);
	scan_center_cm=us_dist_cm; // 중앙 확인

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_LEFT);
	osDelay(500);
	scan_left_cm=us_dist_cm; // 왼쪽 확인

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_RIGHT);
	osDelay(500);
	scan_right_cm=us_dist_cm; //오른쪽 확인

	if((scan_center_cm < SCAN_SAFE_CM) &&
	(scan_left_cm < SCAN_SAFE_CM) &&
	(scan_right_cm < SCAN_SAFE_CM)) {
		scan_dir=0u; //후진
	}

	else if(scan_left_cm>scan_right_cm) {
		scan_dir=1u;
	}
		else {
			scan_dir=2u;
		}

}

void ControlTask(void *argument) {
	(void)argument;

	uint8_t prev_want_run = 0u;
	uint8_t prev_speed_step=auto_speed_step;
	MotorCmd_t cmd;

	for (;;) {
		if(stop_flag) {
			ctrl_want_run=0u;
		}
		else {
			if(us_is_near) {
				ctrl_want_run=0u;

				if(scan_done==0u) {
					ScanPracticeOnce();
					scan_done=1u;

					if(scan_dir==0u) {
						cmd.type=MOTOR_CMD_BACK;
						cmd.speed_step=1u;
						(void)osMessageQueuePut(motorCmdQHandle, &cmd, 0U, 0U);

						osDelay(1000);

						cmd.type=MOTOR_CMD_STOP;
						cmd.speed_step=0u;
						(void)osMessageQueuePut(motorCmdQHandle, &cmd, 0U, 0U);

						scan_done=0u;
					}
					else if(scan_dir==1u) {
						cmd.type=MOTOR_CMD_STOP;
						cmd.speed_step=0u;
						(void)osMessageQueuePut(motorCmdQHandle, &cmd, 0U, 0U);
						Motor_TurnLeft(1);
						osDelay(500);
						Motor_Stop();
					}
					else if(scan_dir==2u) {
						cmd.type=MOTOR_CMD_STOP;
						cmd.speed_step=0u;
						(void)osMessageQueuePut(motorCmdQHandle, &cmd, 0U, 0U);

						Motor_TurnRight(1);
						osDelay(500);
						Motor_Stop();
					}

				}

			}
			else {
				if(us_dist_cm>=SCAN_SAFE_CM) {
					ctrl_want_run=1u;
					scan_done=0u;
				}
				else {
					ctrl_want_run=0u;
				}
			}

			if(us_dist_cm > 40u) {
				auto_speed_step =2u;
			}
			else {
				auto_speed_step=1u;
			}



		}
		if ((ctrl_want_run != prev_want_run) ||
			((ctrl_want_run == 1u) && (auto_speed_step != prev_speed_step)))  {
			prev_want_run=ctrl_want_run;
			prev_speed_step = auto_speed_step;

			if(ctrl_want_run) {
				cmd.type=MOTOR_CMD_RUN;
				cmd.speed_step=auto_speed_step;
			}
			else {
				cmd.type=MOTOR_CMD_STOP;
				cmd.speed_step=auto_speed_step;
			}
			(void)osMessageQueuePut(motorCmdQHandle, &cmd,0U,0U);
		}
		osDelay(20);
	}
}

/* USER CODE END Application */


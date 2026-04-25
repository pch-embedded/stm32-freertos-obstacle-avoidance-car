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

typedef enum {
    APP_EVT_BUTTON_TOGGLE=0
} AppEvent_t;

typedef struct {
	uint32_t dist_cm;
	uint8_t is_near;
	uint32_t pulse_us;
	uint32_t seq;
} UsSnapshot_t;

typedef enum {
    CTRL_STATE_STOPPED = 0,
    CTRL_STATE_RUN,

    CTRL_STATE_SCAN_CENTER_WAIT,
    CTRL_STATE_SCAN_LEFT_WAIT,
    CTRL_STATE_SCAN_RIGHT_WAIT,

    CTRL_STATE_BACK_WAIT,
    CTRL_STATE_TURN_LEFT_WAIT,
    CTRL_STATE_TURN_RIGHT_WAIT
} CtrlState_t;
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
#define SCAN_DIR_NONE 255u
#define US_SPIKE_CM 100u
#define APP_FLAG_US_UPDATE (1U << 0)
#define APP_FLAG_BUTTON_TOGGLE (1U << 1)

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


static UsSnapshot_t g_us_snapshot = {0};

volatile uint32_t scan_center_cm =0;
volatile uint32_t scan_left_cm=0;
volatile uint32_t scan_right_cm=0;
volatile uint8_t scan_dir=SCAN_DIR_NONE; //  1=left,2=right
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

osMessageQueueId_t appEventQHandle;
const osMessageQueueAttr_t appEventQ_attributes = { .name = "appEventQ" };



/*------Mutex---*/
osMutexId_t usMutexHandle;
const osMutexAttr_t usMutex_attributes = {.name = "usMutex"};

/* ---- Event Flags ---- */
osEventFlagsId_t appFlagsHandle;
const osEventFlagsAttr_t appFlags_attributes = {.name = "appFlags"};
		/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* USER CODE BEGIN RTOS_THREADS_DECL */
osThreadId_t ButtonTaskHandle;
const osThreadAttr_t ButtonTask_attributes = {
  .name = "ButtonTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
  .priority = (osPriority_t) osPriorityHigh,
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
		.priority = (osPriority_t) osPriorityAboveNormal,
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
  appEventQHandle   = osMessageQueueNew(8, sizeof(AppEvent_t), &appEventQ_attributes);

  appFlagsHandle    = osEventFlagsNew(&appFlags_attributes);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  usMutexHandle = osMutexNew(&usMutex_attributes);
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

static void UsSnapshot_Update(uint32_t dist_cm, uint8_t is_near, uint32_t pulse_us)
{
	osMutexAcquire(usMutexHandle, osWaitForever);

	g_us_snapshot.dist_cm  = dist_cm;
	g_us_snapshot.is_near  = is_near;
	g_us_snapshot.pulse_us = pulse_us;
	g_us_snapshot.seq++;

	osMutexRelease(usMutexHandle);

	if (appFlagsHandle != NULL) {
	    (void)osEventFlagsSet(appFlagsHandle, APP_FLAG_US_UPDATE);
	}
}

static UsSnapshot_t UsSnapshot_Get(void)
{
    UsSnapshot_t copy;

    osMutexAcquire(usMutexHandle, osWaitForever);

    copy = g_us_snapshot;

    osMutexRelease(usMutexHandle);

    return copy;
}

static void Motor_Post(MotorCmdType_t type, uint8_t speed_step){
    MotorCmd_t cmd;

    cmd.type = type;
    cmd.speed_step = speed_step;

    (void)osMessageQueuePut(motorCmdQHandle, &cmd, 0U, 0U);
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

      (void)osEventFlagsSet(appFlagsHandle, APP_FLAG_BUTTON_TOGGLE);

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
    else if (cmd.type == MOTOR_CMD_TURN_LEFT) {
    	Motor_TurnLeft(cmd.speed_step);
    }
    else if (cmd.type == MOTOR_CMD_TURN_RIGHT) {
    	Motor_TurnRight(cmd.speed_step);
    }
  }
}

/* UltrasonicEdgeTask: consume EXTI edge messages from main.c callback */
void UltrasonicEdgeTask(void *argument)
{
  (void)argument;
  UsEchoEdgeMsg msg;

  uint32_t dist_cm =0u;
  uint8_t is_near = 0u;


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
    	  uint32_t temp_dist_cm = us_pulse_us / 58u;

    	  if ((dist_cm == 0u) || ((temp_dist_cm > dist_cm) && ((temp_dist_cm - dist_cm) < US_SPIKE_CM)) || ((temp_dist_cm <= dist_cm) && ((dist_cm - temp_dist_cm) < US_SPIKE_CM)))
    	  {
    		  dist_cm = temp_dist_cm;
    	  }
    	  if(dist_cm<=US_NEAR_ON_CM) {
    		  is_near=1u;
    	  }
    	  else if(dist_cm>=US_NEAR_OFF_CM) {
    		  is_near=0u;
    	  }
    	  UsSnapshot_Update(dist_cm, is_near, us_pulse_us);
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


void ControlTask(void *argument)
{
    (void)argument;

    uint8_t run_enabled = 0u; // 0=STOP, 1=RUN
    uint8_t prev_want_run = 0u;
    uint8_t prev_speed_step = auto_speed_step;

    CtrlState_t state = CTRL_STATE_STOPPED;
    uint32_t deadline_tick = 0u;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        uint32_t wait_ticks = osWaitForever;

        if (state == CTRL_STATE_SCAN_CENTER_WAIT ||
            state == CTRL_STATE_SCAN_LEFT_WAIT ||
            state == CTRL_STATE_SCAN_RIGHT_WAIT ||
            state == CTRL_STATE_BACK_WAIT ||
            state == CTRL_STATE_TURN_LEFT_WAIT ||
            state == CTRL_STATE_TURN_RIGHT_WAIT)
        {
            if ((int32_t)(now - deadline_tick) >= 0) {
                wait_ticks = 0u;
            } else {
                wait_ticks = deadline_tick - now;
            }
        }

        uint32_t flags = osEventFlagsWait(
            appFlagsHandle,
            APP_FLAG_BUTTON_TOGGLE | APP_FLAG_US_UPDATE,
            osFlagsWaitAny,
            wait_ticks
        );

        if ((flags & osFlagsError) == 0u) {
            if ((flags & APP_FLAG_BUTTON_TOGGLE) != 0u) {
                run_enabled ^= 1u;

                if (!run_enabled) {
                    Motor_Post(MOTOR_CMD_STOP, 0u);
                    ctrl_want_run = 0u;
                    prev_want_run = 0u;
                    prev_speed_step = 0u;
                    state = CTRL_STATE_STOPPED;
                }
            }
        }

        UsSnapshot_t us = UsSnapshot_Get();
        now = osKernelGetTickCount();

        switch (state)
        {
        case CTRL_STATE_STOPPED:
            ctrl_want_run = 0u;

            if (run_enabled) {
                state = CTRL_STATE_RUN;
            }
            break;

        case CTRL_STATE_RUN:
        {
            if (!run_enabled) {
                Motor_Post(MOTOR_CMD_STOP, 0u);
                ctrl_want_run = 0u;
                state = CTRL_STATE_STOPPED;
                break;
            }

            if (us.is_near) {
                Motor_Post(MOTOR_CMD_STOP, 0u);
                ctrl_want_run = 0u;
                prev_want_run = 0u;
                prev_speed_step = 0u;

                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CENTER);
                deadline_tick = now + 500u;
                state = CTRL_STATE_SCAN_CENTER_WAIT;
                break;
            }

            if (us.dist_cm >= SCAN_SAFE_CM) {
                ctrl_want_run = 1u;

                if (us.dist_cm > 40u) {
                    auto_speed_step = 2u;
                } else {
                    auto_speed_step = 1u;
                }
            } else {
                ctrl_want_run = 0u;
                auto_speed_step = 0u;
            }

            if ((ctrl_want_run != prev_want_run) ||
                ((ctrl_want_run == 1u) && (auto_speed_step != prev_speed_step)))
            {
                prev_want_run = ctrl_want_run;
                prev_speed_step = auto_speed_step;

                if (ctrl_want_run) {
                    Motor_Post(MOTOR_CMD_RUN, auto_speed_step);
                } else {
                    Motor_Post(MOTOR_CMD_STOP, 0u);
                }
            }
            break;
        }

        case CTRL_STATE_SCAN_CENTER_WAIT:
            if ((int32_t)(now - deadline_tick) >= 0) {
                scan_center_cm = us.dist_cm;

                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_LEFT);
                deadline_tick = now + 500u;
                state = CTRL_STATE_SCAN_LEFT_WAIT;
            }
            break;

        case CTRL_STATE_SCAN_LEFT_WAIT:
            if ((int32_t)(now - deadline_tick) >= 0) {
                scan_left_cm = us.dist_cm;

                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_RIGHT);
                deadline_tick = now + 500u;
                state = CTRL_STATE_SCAN_RIGHT_WAIT;
            }
            break;

        case CTRL_STATE_SCAN_RIGHT_WAIT:
            if ((int32_t)(now - deadline_tick) >= 0) {
                scan_right_cm = us.dist_cm;

                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CENTER);

                if ((scan_center_cm < SCAN_SAFE_CM) &&
                    (scan_left_cm < SCAN_SAFE_CM) &&
                    (scan_right_cm < SCAN_SAFE_CM))
                {
                    scan_dir = 0u;
                    Motor_Post(MOTOR_CMD_BACK, 1u);
                    deadline_tick = now + 1000u;
                    state = CTRL_STATE_BACK_WAIT;
                }
                else if (scan_left_cm > scan_right_cm) {
                    scan_dir = 1u;
                    Motor_Post(MOTOR_CMD_TURN_LEFT, 1u);
                    deadline_tick = now + 500u;
                    state = CTRL_STATE_TURN_LEFT_WAIT;
                }
                else {
                    scan_dir = 2u;
                    Motor_Post(MOTOR_CMD_TURN_RIGHT, 1u);
                    deadline_tick = now + 500u;
                    state = CTRL_STATE_TURN_RIGHT_WAIT;
                }
            }
            break;

        case CTRL_STATE_BACK_WAIT:
        case CTRL_STATE_TURN_LEFT_WAIT:
        case CTRL_STATE_TURN_RIGHT_WAIT:
            if ((int32_t)(now - deadline_tick) >= 0) {
                Motor_Post(MOTOR_CMD_STOP, 0u);

                scan_dir = SCAN_DIR_NONE;
                scan_done = 0u;

                ctrl_want_run = 0u;
                prev_want_run = 0u;
                prev_speed_step = 0u;

                if (run_enabled) {
                    state = CTRL_STATE_RUN;
                } else {
                    state = CTRL_STATE_STOPPED;
                }
            }
            break;

        default:
            Motor_Post(MOTOR_CMD_STOP, 0u);
            ctrl_want_run = 0u;
            state = CTRL_STATE_STOPPED;
            break;
        }
    }
}

/* USER CODE END Application */


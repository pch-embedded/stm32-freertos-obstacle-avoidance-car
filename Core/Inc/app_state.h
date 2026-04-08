/* app_state.h */
#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>

typedef enum {
    MOTOR_CMD_STOP = 0,
    MOTOR_CMD_RUN  = 1,
	MOTOR_CMD_BACK = 2,
} MotorCmdType_t;

typedef struct {
    MotorCmdType_t type;
    uint8_t speed_step;   // 0..2
} MotorCmd_t;

extern volatile uint8_t stop_flag;   // 1=STOP, 0=RUN

#endif /* APP_STATE_H */

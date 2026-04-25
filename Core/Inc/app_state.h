/* app_state.h */
#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>

typedef enum {
    MOTOR_CMD_STOP = 0,
    MOTOR_CMD_RUN  = 1,
	MOTOR_CMD_BACK = 2,
	MOTOR_CMD_TURN_LEFT=3,
	MOTOR_CMD_TURN_RIGHT=4,
} MotorCmdType_t;

typedef struct {
    MotorCmdType_t type;
    uint8_t speed_step;   // 0..2
} MotorCmd_t;

#endif /* APP_STATE_H */

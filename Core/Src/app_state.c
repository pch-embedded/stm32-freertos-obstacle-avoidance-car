/* app_state.c */
#include "app_state.h"

/* Safe boot: STOP */
volatile uint8_t stop_flag  = 1;
volatile uint8_t speed_step = 0;

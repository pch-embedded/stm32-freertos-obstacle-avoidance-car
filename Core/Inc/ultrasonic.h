/*
 * ultrasonic.h
 *
 *  Created on: 2026. 2. 23.
 *      Author: parkc
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_
#include <stdint.h>

typedef struct{
	uint32_t t_us;
	uint8_t level;
} UsEchoEdgeMsg;




#endif /* INC_ULTRASONIC_H_ */

/*
 * getBall.h
 *
 *  Created on: 20 avr. 2019
 *      Author: Group 24
 */
#ifndef GET_BALL_H
#define GET_BALL_H

#define LOW_MOTOR_SPEED				200
#define HIGH_MOTOR_SPEED			900

#define LARGE_LIMIT_ANGLE    		180
#define SHORT_LIMIT_ANGLE    		120

#define ADJUSTING_ANGLE				75
#define ADJUSTING_CST				0.8

//start the getBall thread
void getBall_start(void);

#endif /* GET_BALL_H */

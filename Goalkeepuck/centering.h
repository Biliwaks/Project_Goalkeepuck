/*
 * centering.h
 *
 *  Created on: 20 avr. 2019
 *      Author: Group 24
 */

#ifndef GOAL_H_
#define GOAL_H_

#define DISTANCE_BETWEEN_PILLARS			600
#define DISTANCE_TO_GOALS					150
#define WAIT_BETWEEN_PILLARS				500
#define HIGH_SPEED							1000
#define LOW_SPEED							300
#define DISTANCE_PILLAR_TOO_CLOSE			200
#define DISTANCE_BOTH_PILLARS_TOO_CLOSE		400
#define ERROR_TOF							50
#define ANGLE_60							60
#define ANGLE_90							90
#define ANGLE_170							170
#define U_TURN								180
#define FULL_TURN							360


void centeringSequence(uint16_t dist_pillar_1, uint16_t dist_pillar_2);
void centerGuardian(void);


#endif /* GOAL_H_ */

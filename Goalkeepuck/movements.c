/*
 * movements.c
 *
 *  Created on: 20 avr. 2019
 *      Author: Group 24
 */
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <main.h>

#include <motors.h>

#include <movements.h>

// mouvement lineaire en fonction de la distance saisie, a la vitesse souhaitee, distance en mm
void translationDistance(uint16_t speed, int16_t distance){

	systime_t time_ref = 0;
	systime_t delta_t = 0;

	float speed_mm = speed*STEPS2MM;

	systime_t time = UNIT2MILLI*fabs(distance)/speed_mm;

	time_ref = chVTGetSystemTime();

	while(delta_t < time){

		delta_t = ST2MS(chVTGetSystemTime()-time_ref);

		if(distance>0){
			left_motor_set_speed(speed);
			right_motor_set_speed(speed);
		}
		else if(distance<0){
			left_motor_set_speed(-speed);
			right_motor_set_speed(-speed);
		}
		else{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

// mouvement circulaire en fonction de l'angle saisi, a la vitesse souhaitee
void rotationAngle(uint16_t speed, float angle){

	systime_t time_ref = 0;
	systime_t delta_t = 0;

	float speed_deg = speed*MM2DEG*STEPS2MM;

	systime_t time = UNIT2MILLI*fabs(angle)/speed_deg;

	time_ref = chVTGetSystemTime();

	while(delta_t < time){

		delta_t = ST2MS(chVTGetSystemTime()-time_ref);

		if(angle>0){
			left_motor_set_speed(speed);
			right_motor_set_speed(-speed);
		}
		else if(angle<0){
			left_motor_set_speed(-speed);
			right_motor_set_speed(speed);
		}
		else{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}




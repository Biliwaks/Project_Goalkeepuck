/*
 * centering.c
 *
 *  Code corrigé: 6 Mai 2019
 *      Author: Group 24
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <arm_math.h>

#include <leds.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <centering.h>
#include <movements.h>

// calcule les angles et distances a parcourir en fonction des distances aux poteaux, et lance la sequence de deplacement adequate
void centeringSequence(uint16_t dist_pillar_1, uint16_t dist_pillar_2){

	uint16_t L = 0;
	uint16_t h = 0;
	float alpha = 0;
	float alpha_deg = 0;

	uint16_t L_corrected = 0;
	uint16_t h_corrected = 0;
	float alpha_corrected = 0;
	float alpha_deg_corrected = 0;

// alpha calcule a partir de la formule d'Al Kashi
	alpha = abs(acos(((DISTANCE_BETWEEN_PILLARS*DISTANCE_BETWEEN_PILLARS)+(dist_pillar_2*dist_pillar_2)-(dist_pillar_1*dist_pillar_1))
			/(2*dist_pillar_2*DISTANCE_BETWEEN_PILLARS)));
	L = abs(((dist_pillar_2)*cos(alpha))-(DISTANCE_BETWEEN_PILLARS/2));
	h = abs((dist_pillar_2)*sin(alpha));
	alpha_deg=(FULL_TURN*alpha/(2*M_PI));

// valeurs corrigees empiriquement
	alpha_deg_corrected = 2*alpha_deg - ANGLE_90;
	alpha_corrected = 2*M_PI*alpha_deg_corrected/FULL_TURN;
	L_corrected = abs(((dist_pillar_2)*cos(alpha_corrected))-(DISTANCE_BETWEEN_PILLARS/2));
	h_corrected = abs((dist_pillar_2)*sin(alpha_corrected));

// lance la sequence de replacement correspondante
	if(dist_pillar_2>dist_pillar_1){

		rotationAngle(HIGH_SPEED,abs(alpha_deg_corrected));
		translationDistance(HIGH_SPEED,abs(L_corrected));
		rotationAngle(HIGH_SPEED,-ANGLE_90);
		translationDistance(HIGH_SPEED,(h_corrected-DISTANCE_TO_GOALS));
		rotationAngle(HIGH_SPEED,U_TURN);
		translationDistance(HIGH_SPEED,DISTANCE_TO_GOALS/2);
	}

	else if(dist_pillar_2<dist_pillar_1){

		rotationAngle(HIGH_SPEED,alpha_deg-U_TURN);
		translationDistance(HIGH_SPEED,abs(L));
		rotationAngle(HIGH_SPEED,ANGLE_90);
		translationDistance(HIGH_SPEED,(h-DISTANCE_TO_GOALS));
		rotationAngle(HIGH_SPEED,U_TURN);
		translationDistance(HIGH_SPEED,DISTANCE_TO_GOALS/2);
	}

	else{

		rotationAngle(HIGH_SPEED,(ANGLE_90+alpha_deg));
		translationDistance(HIGH_SPEED,DISTANCE_TO_GOALS/2);
	}

}

// reperage des poteaux
void centerGuardian(void){

	uint16_t dist_pillar_1 = 0;
	uint16_t dist_pillar_2 = 0;
	uint16_t dist = 0;

	bool pillar1_seen = FALSE;
	bool pillar2_seen = FALSE;
	bool position_found = FALSE;

	systime_t time = 0;

	while(position_found == FALSE){

		dist = VL53L0X_get_dist_mm();

		chThdSleepMilliseconds(TOF_SAMPLE_PERIOD);

		if ((VL53L0X_get_dist_mm() > (dist + ERROR_TOF)) || (VL53L0X_get_dist_mm() < (dist - ERROR_TOF))){

			dist = DISTANCE_MAX_SEEN;
			set_led(LED1,2);
		}

		left_motor_set_speed(LOW_SPEED);
		right_motor_set_speed(-LOW_SPEED);

		set_front_led(0);
		set_body_led(0);

		if((dist<DISTANCE_MAX_SEEN) && (dist > 0) && (pillar1_seen == FALSE)){

			dist_pillar_1 = dist + RADIUS_EPUCK_MM;
			pillar1_seen = TRUE;
			time = chVTGetSystemTime();
		}

		if((dist < DISTANCE_MAX_SEEN) && (dist > 0) && (pillar1_seen == TRUE) && (pillar2_seen == FALSE)
				&& ((chVTGetSystemTime()-time) > MS2ST(WAIT_BETWEEN_PILLARS))){

			dist_pillar_2 = dist + RADIUS_EPUCK_MM;
			pillar2_seen = TRUE;
		}

		if(pillar1_seen)
			set_body_led(1);
		if(pillar2_seen)
			set_front_led(1);

		if((pillar1_seen == TRUE) && (pillar2_seen == TRUE)){

// sequences de replacement pour eviter les cas limites ou la precision des mesures ne permet pas d'obtenir des resultats satisfaisants
			if(((dist_pillar_1 < DISTANCE_PILLAR_TOO_CLOSE) && (dist_pillar_1 > 0)) || ((dist_pillar_1 < DISTANCE_BOTH_PILLARS_TOO_CLOSE)
					&& (dist_pillar_2 < DISTANCE_BOTH_PILLARS_TOO_CLOSE) && (dist_pillar_1 > 0) && (dist_pillar_2 > 0))){

				rotationAngle(2*LOW_SPEED,ANGLE_60);
				translationDistance(2*LOW_SPEED,DISTANCE_PILLAR_TOO_CLOSE);
				pillar1_seen = FALSE;
				pillar2_seen = FALSE;
				position_found = FALSE;
			}

			else if((dist_pillar_2 < DISTANCE_PILLAR_TOO_CLOSE) && (dist_pillar_2 > 0)){

				rotationAngle(2*LOW_SPEED,-ANGLE_170);
				translationDistance(2*LOW_SPEED,DISTANCE_PILLAR_TOO_CLOSE);
				pillar1_seen = FALSE;
				pillar2_seen = FALSE;
				position_found = FALSE;
			}

			else if((dist_pillar_1 > (DISTANCE_BETWEEN_PILLARS+DISTANCE_PILLAR_TOO_CLOSE)) || (dist_pillar_2 > (DISTANCE_BETWEEN_PILLARS+DISTANCE_PILLAR_TOO_CLOSE))){

				if(dist_pillar_1  > dist_pillar_2){
					rotationAngle(2*LOW_SPEED,-ANGLE_170);
					translationDistance(2*LOW_SPEED,DISTANCE_PILLAR_TOO_CLOSE);
					pillar1_seen = FALSE;
					pillar2_seen = FALSE;
					position_found = FALSE;
				}

				else if(dist_pillar_2  > dist_pillar_1){

					rotationAngle(2*LOW_SPEED,-ANGLE_60);
					translationDistance(2*LOW_SPEED,DISTANCE_PILLAR_TOO_CLOSE);
					pillar1_seen = FALSE;
					pillar2_seen = FALSE;
					position_found = FALSE;
				}
			}

			else{

				centeringSequence(dist_pillar_1,dist_pillar_2);
				position_found = TRUE;
			}
		}
	}

//	arret du robot
	left_motor_set_speed(0);
	right_motor_set_speed(0);

	set_front_led(0);
	set_body_led(0);
	set_led(LED1,0);

}

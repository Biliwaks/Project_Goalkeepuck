/*
 * getBall.c
 *
 *  Created on: 20 avr. 2019
 *      Author: Group 24
 */
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>

#include <motors.h>
#include <leds.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <centering.h>
#include <celebration.h>
#include <getBall.h>
#include <movements.h>

// calcul de la distance a parcourir pour intercepter le trajet de la balle
uint16_t calculate_target(uint16_t d0, uint16_t d1, float beta, float gamma){

	gamma = gamma/DEG2RAD;
	beta = beta/DEG2RAD;

    uint16_t dist = 0;

    float delta_d = sqrt(d0*d0 + d1*d1 - 2*d0*d1*cos(beta));						//distance entre d0 et d1
    float theta = acos((d0*d0+delta_d*delta_d-d1*d1)/(2*delta_d*d0));				//angle entre d0 et la trajectoire de la balle

    dist =  fabs(sin(theta + beta)*d1/sin(theta+beta+gamma));

    return dist;
}

// effectue la recherche de la balle, en scannant le terrain. Retourne un tableau avec la direction, la distance et l'angle ou se trouve la balle.
float * ball_search(void){

	systime_t time_ref = 0;
	systime_t delta_t = 0;
	systime_t limit_time = 0;

	uint16_t d0 = 0;
	float alpha = 0;
	static float search_data[4] = {0};
	bool side_known = FALSE;
	bool first = TRUE;

	bool alternate = TRUE;								//variable pour que les rotations à droites et à gauches soient symmétriques
														//au niveau de temps d'execution entre chacune (compensation d'erreur)

	int time_to_get_ball_center = 0;
	float speed = LOW_MOTOR_SPEED*STEPS2MM;				//conversion step/s en mm/s (141 mm/s equivaut à 1100 step/s)

	speed = speed*MM2DEG;								//conversion mm/s en degre/seconde (360 degré équivaut à 167 mm de parcours des roues)

	limit_time = UNIT2MILLI*LARGE_LIMIT_ANGLE/speed;	//temps qu'il faut pour la rotation a la vitesse donnee (converison en ms)

	while(side_known == FALSE){							//Dans un premier temps on parcours un demi-angle

		if (first == TRUE){

			delta_t = 0;
			time_ref = chVTGetSystemTime();

			while (delta_t < limit_time/2){

				delta_t = ST2MS(chVTGetSystemTime()-time_ref);

				right_motor_set_speed(LOW_MOTOR_SPEED);
				left_motor_set_speed(-LOW_MOTOR_SPEED);

				chThdSleepMilliseconds(10);
			}
			right_motor_set_speed(0);
			left_motor_set_speed(0);
		}

		delta_t = 0;
		time_ref = chVTGetSystemTime();

		if (alternate == TRUE){

			while (delta_t < limit_time){

				delta_t = ST2MS(chVTGetSystemTime()-time_ref);

				right_motor_set_speed(-LOW_MOTOR_SPEED);
				left_motor_set_speed(LOW_MOTOR_SPEED);

				d0 = VL53L0X_get_dist_mm();

				if ((d0 <= DISTANCE_MAX_SEEN) && (d0 > 0) && (first == FALSE)){

					set_front_led(1);												//on l'appelle l'anti phantome
					chThdSleepMilliseconds(TOF_SAMPLE_PERIOD);						//il permet d'attendre le prochain échantillon
					set_front_led(0);												//afin d'etre sur de capter une vraie valeur
																					//inférieur à la distance max et non pas du bruit
					if (VL53L0X_get_dist_mm() <= DISTANCE_MAX_SEEN){

						time_to_get_ball_center = UNIT2MILLI*RADIUS_BALL_MM*ADJUSTING_CST*DEG2RAD/(d0*speed);
						alpha = speed*(delta_t + time_to_get_ball_center)/UNIT2MILLI-LARGE_LIMIT_ANGLE/2;	//compensation d'angle

						if(time_to_get_ball_center > TOF_SAMPLE_PERIOD)
							chThdSleepMilliseconds(time_to_get_ball_center-TOF_SAMPLE_PERIOD); //On attend pour que le robot se centre
																								//grace a la formule
						right_motor_set_speed(0);
						left_motor_set_speed(0);

						if ((delta_t + time_to_get_ball_center) <= limit_time/2)
							search_data[1] = LEFT;										//ball_side
						else
							search_data[1] = RIGHT;

						search_data[2] = alpha;

						search_data[3] = d0 + RADIUS_EPUCK_MM;

						return search_data;
					}
				}
				chThdSleepMilliseconds(10);
			}
			first = FALSE;
		}

		if (alternate == FALSE){

			while (delta_t < limit_time){

				delta_t = ST2MS(chVTGetSystemTime()-time_ref);

				right_motor_set_speed(LOW_MOTOR_SPEED);
				left_motor_set_speed(-LOW_MOTOR_SPEED);

				d0 = VL53L0X_get_dist_mm();

				if ((d0 <= DISTANCE_MAX_SEEN) && (d0 > 0)){

						set_front_led(1);
						chThdSleepMilliseconds(TOF_SAMPLE_PERIOD);
						set_front_led(0);

						if (VL53L0X_get_dist_mm() <= DISTANCE_MAX_SEEN){

							time_to_get_ball_center = UNIT2MILLI*RADIUS_BALL_MM*ADJUSTING_CST*DEG2RAD/(d0*speed);
							alpha = -speed*(delta_t + time_to_get_ball_center)/UNIT2MILLI + LARGE_LIMIT_ANGLE/2;

							if(time_to_get_ball_center > TOF_SAMPLE_PERIOD)
								chThdSleepMilliseconds(time_to_get_ball_center - TOF_SAMPLE_PERIOD);

							right_motor_set_speed(0);
							left_motor_set_speed(0);

							if ((delta_t + time_to_get_ball_center) <= limit_time/2)
								search_data[1] = RIGHT;									//ball_side
							else
								search_data[1] = LEFT;

							search_data[2] = alpha;


							search_data[3] = d0 + RADIUS_EPUCK_MM;						// le capteur n'est pas au centre, on corrige donc la distance

							return search_data;
						}
				}
				chThdSleepMilliseconds(10);
			}
		}

		right_motor_set_speed(0);
		left_motor_set_speed(0);

		alternate = !alternate;
		side_known = FALSE;
	}
	return NULL;
}

// scanne le terrain autour d'un angle de 120° pour determiner la direction du lancer
float * direction_search(uint8_t ball_side, float alpha){

	systime_t time_ref = 0;
	systime_t delta_t = 0;
	systime_t limit_time = 0;
	systime_t time_to_get_ball_center = 0;

	uint16_t d1 = 0;

	float beta = 0;

	static float direction_data[4] = {0};
	static bool first = TRUE;
	static bool alternate = TRUE;

	float speed = HIGH_MOTOR_SPEED*STEPS2MM;										//conversion step/s en mm/s (141 mm/s equivaut à 1100 step/s)

	speed = speed*MM2DEG;															//conversion mm/s en degres/seconde (360 degres equivaut à 167 mm de parcours des roues)

	limit_time = UNIT2MILLI*SHORT_LIMIT_ANGLE/speed;								//temps qu'il faut pour la rotation a la vitesse donnee (converison en ms)

	if (first == TRUE){

		if (ball_side == LEFT)														// Commence par tourner du cote le moins bien protege par le goal
			alternate = TRUE;
		else
			alternate = FALSE;

		delta_t = 0;
		time_ref = chVTGetSystemTime();

		while (delta_t < limit_time/2){

			delta_t = ST2MS(chVTGetSystemTime()-time_ref);

			if (ball_side == LEFT){
				right_motor_set_speed(HIGH_MOTOR_SPEED);
				left_motor_set_speed(-HIGH_MOTOR_SPEED);
			}
			else{
				right_motor_set_speed(-HIGH_MOTOR_SPEED);
				left_motor_set_speed(HIGH_MOTOR_SPEED);
			}

			beta = speed*delta_t/UNIT2MILLI;						//calcule une première version de beta pour eviter de voir les poteaux

			if ((fabs(alpha) + beta) < ADJUSTING_ANGLE){

				d1 = VL53L0X_get_dist_mm();

				if ((d1 <= DISTANCE_MAX_SEEN) && (d1 > 0)){

					set_front_led(1);
					chThdSleepMilliseconds(TOF_SAMPLE_PERIOD);
					set_front_led(0);

					if (VL53L0X_get_dist_mm() <= DISTANCE_MAX_SEEN){

						right_motor_set_speed(0);
						left_motor_set_speed(0);

						time_to_get_ball_center = UNIT2MILLI*RADIUS_BALL_MM*ADJUSTING_CST*DEG2RAD/(d1*speed);

						beta = speed*(delta_t + time_to_get_ball_center) /UNIT2MILLI; 	//calcule une meilleure version de beta
																						//grace au terme correctif

						if (ball_side == LEFT)
							direction_data[1] = LEFT;									//direction
						else
							direction_data[1] = RIGHT;

						direction_data[2] = beta;

						direction_data[3] = d1 + RADIUS_EPUCK_MM;

						direction_data[0] = TRUE;										//direction connue

						first = TRUE;													//réinitialisation des variables statiques
						alternate = TRUE;												//pour le prochain passage

						return direction_data;
					}
				}
			}
			chThdSleepMilliseconds(10);
		}
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}

	delta_t = 0;
	time_ref = chVTGetSystemTime();

	if (alternate == TRUE){

		while (delta_t < limit_time){

			delta_t = ST2MS(chVTGetSystemTime()-time_ref);

			right_motor_set_speed(-HIGH_MOTOR_SPEED);
			left_motor_set_speed(HIGH_MOTOR_SPEED);


			beta = speed*delta_t/UNIT2MILLI-SHORT_LIMIT_ANGLE/2;

			if (((alpha + beta) < ADJUSTING_ANGLE) && ((alpha + beta) > -ADJUSTING_ANGLE)){

				d1 = VL53L0X_get_dist_mm();

				if ((d1 <= DISTANCE_MAX_SEEN) && (d1 > 0)){

					set_front_led(1);
					chThdSleepMilliseconds(TOF_SAMPLE_PERIOD);
					set_front_led(0);

					if (VL53L0X_get_dist_mm() <= DISTANCE_MAX_SEEN){

						right_motor_set_speed(0);
						left_motor_set_speed(0);

						time_to_get_ball_center = UNIT2MILLI*RADIUS_BALL_MM*ADJUSTING_CST*DEG2RAD/(d1*speed);
						beta = speed*(delta_t + time_to_get_ball_center)/UNIT2MILLI - SHORT_LIMIT_ANGLE/2;

						if (delta_t <= limit_time/2)
							direction_data[1] = LEFT;
						else
							direction_data[1] = RIGHT;


						direction_data[2] = beta;


						direction_data[3] = d1 + RADIUS_EPUCK_MM;


						direction_data[0] = TRUE;									// direction connue

						first = TRUE;
						alternate = TRUE;

						return direction_data;
					}

				}
			}
			chThdSleepMilliseconds(10);
		}
		first = FALSE;
	}

	if (alternate == FALSE){

		while (delta_t < limit_time){

			delta_t = ST2MS(chVTGetSystemTime()-time_ref);

			right_motor_set_speed(HIGH_MOTOR_SPEED);
			left_motor_set_speed(-HIGH_MOTOR_SPEED);

			beta = -speed*delta_t/UNIT2MILLI + SHORT_LIMIT_ANGLE/2;

			if (((alpha + beta) < ADJUSTING_ANGLE) && ((alpha + beta) > -ADJUSTING_ANGLE)){

				d1 = VL53L0X_get_dist_mm();

				if ((d1 <= DISTANCE_MAX_SEEN) && (d1 > 0)){

					set_front_led(1);
					chThdSleepMilliseconds(TOF_SAMPLE_PERIOD);
					set_front_led(0);

					if (VL53L0X_get_dist_mm() <= DISTANCE_MAX_SEEN){

						right_motor_set_speed(0);
						left_motor_set_speed(0);

						time_to_get_ball_center = UNIT2MILLI*RADIUS_BALL_MM*ADJUSTING_CST*DEG2RAD/(d1*speed);
						beta = -speed*(delta_t + time_to_get_ball_center)/UNIT2MILLI + SHORT_LIMIT_ANGLE/2;

						if (delta_t <= limit_time/2)
							direction_data[1] = RIGHT;
						else
							direction_data[1] = LEFT;


						direction_data[2] = beta;

						direction_data[3] = d1 + RADIUS_EPUCK_MM;


						direction_data[0] = TRUE;

						first = TRUE;
						alternate = TRUE;

						return direction_data;
					}
				}
			}
			chThdSleepMilliseconds(10);												//on echantillonne à 100 Hz
		}
		first = FALSE;
	}

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	alternate = !alternate;

	direction_data[3] = 0;
	direction_data[2] = 0;
	direction_data[1] = 0;
	direction_data[0] = FALSE;

	return direction_data;
}


float find_gamma(float alpha, float beta, uint8_t ball_side, uint8_t direction){

	float gamma = 0;

	if (direction == ball_side)
		gamma = 90 - fabs(beta) - fabs(alpha);

	else if (direction != ball_side)
		gamma = 90 - fabs(beta) + fabs(alpha);

	else
		chprintf((BaseSequentialStream *) &SDU1, "error\n");

	if (direction == RIGHT)
		gamma = fabs(gamma);

	else if (direction == LEFT)
		gamma = -fabs(gamma);

	else
		chprintf((BaseSequentialStream *) &SDU1, "error\n");

	return gamma;
}

static THD_WORKING_AREA(waGetBall, 1024);											// Pas sur de la taille en mémoire
static THD_FUNCTION(GetBall, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float alpha = 0;																// radians ou degrés
    float beta = 0;																	// radians ou degrés
    float gamma = 0;																// radians ou degrés

   	uint16_t d0 = 0, d1 = 0;														// en mm
   	uint16_t dist = 0;																// en mm

   	float *search_data = NULL;

   	uint8_t ball_side = 0;

   	float * direction_data = NULL;
	bool direction_known = FALSE;
	uint8_t direction = 0;

	bool rotation_done = FALSE;
	bool target_reached = FALSE;

    while(1){

        centerGuardian();

        search_data = ball_search();												//chercher la balle

        ball_side = search_data[1];
        alpha = search_data[2];
        d0 = search_data[3];

        while (1){

        	if ((VL53L0X_get_dist_mm() < d0/3) && (VL53L0X_get_dist_mm() > 0)){
        		translationDistance(MOTOR_SPEED_LIMIT, d0/3);							// si la balle s'approche trop du robot
				rotation_done = TRUE;													//en restant toujours devant,
				target_reached = TRUE;													// alors le robot va essayer de couper l'angle
				direction_known = TRUE;
				break;
        	}

        	if (VL53L0X_get_dist_mm() > DISTANCE_MAX_SEEN){								//A de longues distances, le robot capte
        																				//de moins en moins la balle. Pour maximiser les
        		chThdSleepMilliseconds(TOF_SAMPLE_PERIOD);								//distances ou la capture est possible,
        		if (VL53L0X_get_dist_mm() > DISTANCE_MAX_SEEN){							//on regarde si sur trois échantillons différents,
        			chThdSleepMilliseconds(TOF_SAMPLE_PERIOD);							//la balle n'est pas vue.
        			if (VL53L0X_get_dist_mm() > DISTANCE_MAX_SEEN)
        				break;
        		}
        	}
        }

		while (direction_known == FALSE){

			direction_data = direction_search(ball_side, alpha);				//cherche la direction

			direction_known = direction_data[0];
			direction = direction_data[1];
			beta = direction_data[2];
			d1 = fabs(direction_data[3]);
		}


		if (rotation_done == FALSE){
			gamma = find_gamma(alpha, beta, ball_side, direction);					//caclule gamma et le retourne en degré
			rotationAngle(MOTOR_SPEED_LIMIT, gamma);
			rotation_done = TRUE;
		}

		dist = calculate_target(d0,d1,fabs(beta),fabs(gamma));

		if(target_reached == FALSE){
			translationDistance(MOTOR_SPEED_LIMIT, dist);
			target_reached = TRUE;
		}


        chThdSleepMilliseconds(WAIT_TO_CELBRATE);									//attend de savoir s'il y a un but

        if (get_goal_info() == FALSE)
        	celebrate();

        set_goal_false();

        direction_known = FALSE;
        rotation_done = FALSE;														//réinitialisation des variables pour
        target_reached = FALSE;														//le prochain essai

        chThdSleepMilliseconds(10);
    }
}

void getBall_start(void){
	chThdCreateStatic(waGetBall, sizeof(waGetBall), NORMALPRIO, GetBall, NULL);
}


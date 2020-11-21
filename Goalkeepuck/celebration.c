/*
 * celebration.c
 *
 *  Created on: 20 avr. 2019
 *      Modified by Group 24, originally from TP5_Noisy (audio_processing.c)
 */
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <fft.h>
#include <arm_math.h>
#include <leds.h>
#include <audio/play_melody.h>
#include <audio/audio_thread.h>

#include <getBall.h>
#include <celebration.h>

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)

static float micFront_cmplx_input[2 * FFT_SIZE];
static float micFront_output[FFT_SIZE];

static bool goal = FALSE;

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		340	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	353	// 5400 Hz
#define MAX_FREQ		360	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-2)
#define FREQ_FORWARD_H		(FREQ_FORWARD+2)

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

void set_goal_false(void){
	goal = FALSE;
}

bool get_goal_info(void){
	return goal;
}

void celebrate(void){

	playMelody(WE_ARE_THE_CHAMPIONS,ML_SIMPLE_PLAY,NULL);
	for(uint8_t i = 0; i < 100; i++){
		set_body_led(1);
		chThdSleepMilliseconds(50);
		set_body_led(0);
		chThdSleepMilliseconds(50);
	}
	stopCurrentMelody();
}

void get_angry(float* data){								//Sequence d'operations pour allumer et eteindre les leds lorsque
															//le robot entend une certaine frequence representant un coup de sifflet
															//et signifiant qu'il y a un but.
	static bool first = TRUE;

	if (first == FALSE){
		float max_norm = MIN_VALUE_THRESHOLD;
		int16_t max_norm_index = -1;

		//search for the highest peak
		for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
			if(data[i] > max_norm){
				max_norm = data[i];
				max_norm_index = i;
			}
		}

		//go forward
		if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){

			goal = TRUE;

			for (uint8_t i = 0; i <= 8; i += 4){			//vitesse de rotation des leds croissante
				set_led(LED7,0);
				set_led(LED1,1);
				chThdSleepMilliseconds(480-40*i);
				set_led(LED1,0);
				set_led(LED3,1);
				chThdSleepMilliseconds(460-40*i);
				set_led(LED3,0);
				set_led(LED5,1);
				chThdSleepMilliseconds(440-40*i);
				set_led(LED5,0);
				set_led(LED7,1);
				chThdSleepMilliseconds(420-40*i);
			}

			for (uint8_t i = 0; i <= 4; i++){				//Rotation des leds haute mais constante
				set_led(LED7,0);
				set_led(LED1,1);
				chThdSleepMilliseconds(75);
				set_led(LED1,0);
				set_led(LED3,1);
				chThdSleepMilliseconds(75);
				set_led(LED3,0);
				set_led(LED5,1);
				chThdSleepMilliseconds(75);
				set_led(LED5,0);
				set_led(LED7,1);
				chThdSleepMilliseconds(75);
			}
			for (uint8_t i = 0; i <= 10; i ++){				// Toggle des leds a  haute frequence

				set_led(LED1,1);
				set_led(LED3,1);
				set_led(LED5,1);
				set_led(LED7,1);

				chThdSleepMilliseconds(75);

				set_led(LED1,0);
				set_led(LED3,0);
				set_led(LED5,0);
				set_led(LED7,0);

				chThdSleepMilliseconds(75);
			}
		}
	}
	first = FALSE;
}

/*
*	Callback called when the demodulation of the front microphones is done.
*	We get 160 samples for the front mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*							So we only take the front mic every 4th increment
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){

		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/

		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		nb_samples = 0;
		mustSend++;

		get_angry(micFront_output);
	}
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){

	if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}

	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}

	else{
		return NULL;
	}
}

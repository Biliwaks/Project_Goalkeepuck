/*
 * celebration.h
 *
 *  Created on: 20 avr. 2019
 *      Modified by Group 24, originally from TP5_Noisy (audio_processing.h)
 */
#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 			1024
#define WAIT_TO_CELBRATE 	5000

typedef enum {

	FRONT_CMPLX_INPUT = 0,
	FRONT_OUTPUT,

} BUFFER_NAME_t;


void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

void celebrate(void);

bool get_goal_info(void);

void set_goal_false(void);

#endif /* AUDIO_PROCESSING_H */

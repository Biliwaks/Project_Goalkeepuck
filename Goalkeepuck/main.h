/*
 * main.h
 *
 *  Created on: 20 avr. 2019
 *      Modified by Group 24, originally from TP5_Noisy
 */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include <math.h>

#define RIGHT					2
#define LEFT					1
#define CENTER					0
#define DIST_LIMIT				1000
#define SEARCHING				1
#define NOT_SEARCHING			0
#define RADIUS_BALL_MM			60
#define STEPS2MM				154/1200
#define MM2DEG					360/167
#define UNIT2MILLI				1000
#define DEG2RAD					180/M_PI
#define DISTANCE_MAX_SEEN		900
#define RADIUS_EPUCK_MM			35

#define TOF_SAMPLE_PERIOD		50

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif

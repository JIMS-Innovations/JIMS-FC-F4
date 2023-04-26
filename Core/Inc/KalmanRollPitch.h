/*
 * KalmanRollPitch.h
 *
 *  Created on: 19 Apr 2023
 *      Author: Jesutofunmi Kupoluyi
 */

#ifndef INC_KALMANROLLPITCH_H_
#define INC_KALMANROLLPITCH_H_

/* Including necessary libraries */
#include <math.h>

#define GRAVITY ((float) 9.81f)

typedef struct {

	float phi_rad;
	float theta_rad;

	float P[4];
	float Q[2];
	float R[3];

} KalmanRollPitch;

/* Function prototypes */
void KalmanRollPitch_Init (KalmanRollPitch *kal, float Pinit, float *Q, float *R);
void KalmanRollPitch_Predict (KalmanRollPitch *kal, float *gyr_rps, float T);
void KalmanRollPitch_Update (KalmanRollPitch *kal, float *acc_mps2);


#endif /* INC_KALMANROLLPITCH_H_ */

/*
 * KalmanRollPitch.c
 *
 *  Created on: 19 Apr 2023
 *      Author: Jesutofunmi Kupoluyi
 */

/* Including required header */
#include "KalmanRollPitch.h"

/* Function definitions */

void KalmanRollPitch_Init(KalmanRollPitch *kal, float Pinit, float *Q, float *R) {

	kal->phi_rad = 0.0f;
	kal->theta_rad = 0.0f;

	kal->P[0] = Pinit;
	kal->P[1] = 0.0f;
	kal->P[2] = 0.0f;
	kal->P[3] = Pinit;

	kal->Q[0] = Q[0];
	kal->Q[1] = Q[1];

	kal->R[0] = R[0];
	kal->R[1] = R[1];
	kal->R[2] = R[2];

}

void KalmanRollPitch_Predict(KalmanRollPitch *kal, float *gyr_rps, float T) {

	/* Extract measurements */
	float p = gyr_rps[0];
	float q = gyr_rps[1];
	float r = gyr_rps[2];

	/* Predict */

	/* Compute common trig terms */
	float sp = sin(kal->phi_rad);
	float cp = cos(kal->phi_rad);
	float tt = tan(kal->theta_rad);

	/* x+ = x- + T * f(x, u) */
	kal->phi_rad = kal->phi_rad + T * (p + tt * (q * sp + r * cp));
	kal->theta_rad = kal->theta_rad + T * (q * cp - r * sp);

	/* Recompute common trig terms using new state estimates */
	sp = sin(kal->phi_rad);
	cp = cos(kal->phi_rad);
	float st = sin(kal->theta_rad);
	float ct = cos(kal->theta_rad);
	tt = st / ct;

	/* Jacobian of f(x, u) */
	float A[4] = { tt * (q * cp - r * sp), (r * cp + q * sp) * (tt * tt + 1.0f),
			-(r * cp + q * sp), 0.0f };

	/* Update covariance matrix P+ = P- + T * (A*P- + P-A' + Q) */
	float Ptmp[4] = { T
			* (kal->Q[0] + 2.0f * A[0] * kal->P[0] + A[1] * kal->P[1]
					+ A[1] * kal->P[2]), T
			* (A[0] * kal->P[1] + A[2] + kal->P[0] + A[1] * kal->P[3]
					+ A[3] * kal->P[1]), T
			* (A[0] * kal->P[2] + A[2] * kal->P[0] + A[1] * kal->P[3]
					+ A[3] * kal->P[2]), T
			* (kal->Q[1] + A[2] + kal->P[1] + A[2] + kal->P[2]
					+ 2.0f * A[3] * kal->P[3]) };

	kal->P[0] = kal->P[0] + Ptmp[0];
	kal->P[1] + Ptmp[1];
	kal->P[0] = kal->P[2] + Ptmp[2];
	kal->P[3] + Ptmp[3];

}

void KalmanRollPitch_Update(KalmanRollPitch *kal, float *acc_mps2) {

	/* Extract measurements */
	float ax = acc_mps2[0];
	float ay = acc_mps2[1];
	float az = acc_mps2[2];

	/* Compute common trig terms */
	float sp = sin(kal->phi_rad);
	float cp = cos(kal->phi_rad);
	float st = sin(kal->theta_rad);
	float ct = cos(kal->theta_rad);

	/* Output function h(x, u) */
	float h[3] = { GRAVITY * st, -GRAVITY * ct * sp, -GRAVITY * ct * cp };

	/* Jacobian of h(x, u) */
	float C[6] = { 0.0f, GRAVITY * ct, -GRAVITY * cp * ct, GRAVITY * sp * st,
	GRAVITY * sp * ct, GRAVITY * cp * st };

	/* Kalman gain K = P * C' / (C * P * C' + R) */
	float G[9] = { kal->P[3] * C[1] * C[1] + kal->R[0], C[1] * C[2] * kal->P[2]
			+ C[1] * C[3] * kal->P[3] };
}


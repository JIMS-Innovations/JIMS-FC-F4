/*
 * Filters.c
 *
 *  Created on: Apr 2, 2023
 *      Author: Jesutofunmi Kupoluyi
 */

#include "RCFilter.h"


void RC_filter_init(RC_filter *filter, float cut_off_freq_Hz, float sample_time_S){

	/* Compute equivalent 'RC' constant from cut off frequency */
	float RC = 1.0f / (6.28318530718f * cut_off_freq_Hz);

	/* Pre-compute filter coefficients for firat-order low pass filter */
	filter->coeff[0] = sample_time_S / (sample_time_S + RC);
	filter->coeff[1] = RC / (sample_time_S + RC);

	/* Clear the output buffer */
	filter->out[0] = 0.0f;
	filter->out[1] = 0.0f;

}

float RC_filter_Update(RC_filter *filter, float input){

	/* Shift output samples */
	filter->out[1] = filter->out[0];

	/* Compute new output sample */
	filter->out[0] = filter->coeff[0] * input + filter->coeff[1] * filter->out[1];

	/* Return filter sample */
	return (filter->out[0]);

}

/*
 * FIRFilter.c
 *
 *  Created on: 18 Apr 2023
 *      Author: Jesutofunmi Kupoluyi
 */

/* Including required header */
#include "FIRFilter.h"

/* Variables */
//static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {-0.0032906f, -0.0052635f, -0.0068811f, 0.0000000f, 0.0254209f, 0.1311260f, 0.1805961f, 0.2000000, 0.1805961f, 0.1311260f, 0.0724719f, 0.0254209f, 0.0000000f, -0.0068811f, -0.0052635f};
static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}; /* For 10 point Moving Average Filter*/

void FIRFilter_Init(FIRFilter *fir) {

	/* Clear filter buffer */
	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++ ){

		fir->buf[n] = 0.0f;

	}

	/* Reset buffer index */
	fir->bufIndex = 0;

	/* Clear filter output */
	fir->out = 0.0f;

}

float FIRFilter_Update(FIRFilter *fir, float inp) {

	/*Store the latest input sample in buffer */
	fir->buf[fir->bufIndex] = inp;

	/* Increment buffer index and wrap around if necessary */
	fir->bufIndex++;

	if (fir->bufIndex == FIR_FILTER_LENGTH)
			fir->bufIndex = 0;

	/* Compute new output sample (via convolution) */
	fir->out = 0.0f;

	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

		/* Decrement index and wrap around if necessary */
		if (sumIndex > 0)
			sumIndex--;

		else
			sumIndex = FIR_FILTER_LENGTH - 1;

		/* Multiply impulse response with shifted input sample and add to output */
		fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

	}

	/* Return filtered output */
	return fir->out;

}

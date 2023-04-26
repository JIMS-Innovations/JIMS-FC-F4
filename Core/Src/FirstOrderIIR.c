/*
 * FirstOrderIIR.c
 *
 *  Created on: 18 Apr 2023
 *      Author: Jesutofunmi Kupoluyi
 */

/* Including require header */
#include "FirstOrderIIR.h"

void FirstOrderIIR_Init (FirstOrderIIR *filt, float alpha) {

	/* Check filter coefficient bounds and store */
	if (alpha < 0.0f )
		filt->alpha = 0.0f;

	else if (alpha > 1.0f)
		filt->alpha = 1.0f;

	else
		filt->alpha = alpha;

	/* Reset filter output */
	filt->out = 0.0f;

}

float FirstOrderIIR_Update (FirstOrderIIR *filt, float in) {

	/* Compute output using input and previous output */
	filt->out = (1.0f - filt->alpha) * in + filt->alpha * filt->out;

	/* Return filtered output */
	return filt->out;

}


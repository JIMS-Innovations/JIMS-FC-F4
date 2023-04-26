/*
 * FIRFilter.h
 *
 *  Created on: 18 Apr 2023
 *      Author: ayokupoluyi
 */

#ifndef INC_FIRFILTER_H_
#define INC_FIRFILTER_H_

/* Including required libraries */
#include <stdint.h>

/* Defines */
#define FIR_FILTER_LENGTH 10 /* Formerly 16 for LPF / 10 for MA filter */

typedef struct {

	float buf[FIR_FILTER_LENGTH];
	uint8_t bufIndex;

	float out;

} FIRFilter;

/* Function prototypes */
void FIRFilter_Init(FIRFilter *fir);
float FIRFilter_Update(FIRFilter *fir, float inp);

#endif /* INC_FIRFILTER_H_ */

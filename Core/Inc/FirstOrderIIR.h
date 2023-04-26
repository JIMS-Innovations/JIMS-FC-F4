/*
 * FirstOrderIIR.h
 *
 *  Created on: 18 Apr 2023
 *      Author: ayokupoluyi
 */

#ifndef INC_FIRSTORDERIIR_H_
#define INC_FIRSTORDERIIR_H_

/* IIR Fiter structure */
typedef struct {

	float alpha;

	float out;

} FirstOrderIIR;

/* Filter Initialization (store filter coefficient, clear output) */
void FirstOrderIIR_Init(FirstOrderIIR *filt, float alpha);

/* Filter update (compute new output sample) */
float FirstOrderIIR_Update (FirstOrderIIR *filt, float in);

#endif /* INC_FIRSTORDERIIR_H_ */

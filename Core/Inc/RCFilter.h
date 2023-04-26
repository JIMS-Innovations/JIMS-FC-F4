/*
 * Filters.h
 *
 *  Created on: Apr 2, 2023
 *      Author: Jesutofunmi Kupoluyif
 */

#ifndef INC_RCFILTER_H_
#define INC_RCFILTER_H_

typedef struct {

	float  coeff[2];
	float out[2];

} RC_filter;

void RC_filter_init(RC_filter *filter, float cut_off_freq_Hz, float sample_time_S);

float RC_filter_Update(RC_filter *filter, float input);


#endif /* INC_RCFILTER_H_ */

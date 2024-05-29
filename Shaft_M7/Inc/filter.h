/*
 * filter.h
 *
 *  Created on: Oct 19, 2023
 *      Author: ADMIN
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include "common.h"


int16_t iir(int16_t NewSample);
int16_t fir(int16_t NewSample);
int16_t firLIDAR(int16_t NewSample);

#endif /* INC_FILTER_H_ */

/*
 * filter.c
 *
 *  Created on: Oct 19, 2023
 *      Author: ADMIN
 */


#include "filter.h"

typedef int16_t __int16;
typedef int32_t __int32;

#define NCoef 2
#define DCgain 1

int16_t iir(int16_t NewSample) {
    __int16 ACoef[NCoef+1] = {
        12823,
        25646,
        12823
    };

    __int16 BCoef[NCoef+1] = {
        32767,
        12108,
         6416
    };

    static __int32 y[NCoef+1]; //output samples
    //Warning!!!!!! This variable should be signed (input sample width + Coefs width + 2 )-bit width to avoid saturation.

    static __int16 x[NCoef+1]; //input samples
    int n;

    //shift the old samples
    for(n=NCoef; n>0; n--) {
       x[n] = x[n-1];
       y[n] = y[n-1];
    }

    //Calculate the new output
    x[0] = NewSample;
    y[0] = ACoef[0] * x[0];
    for(n=1; n<=NCoef; n++)
        y[0] += ACoef[n] * x[n] - BCoef[n] * y[n];

    y[0] /= BCoef[0];

    return y[0] / DCgain;
}



#define Ntap 10
#define FIRDCgain 65536


const __int16 FIRCoef[Ntap] = {
    471,
    2554,
    6730,
    13958,
    18458,
    13958,
    6730,
    2554,
    471,
    -354
};

int16_t fir(int16_t NewSample) {

    static __int16 x[Ntap]; //input samples
    __int32 y=0;            //output sample
    int n;

    //shift the old samples
    for(n=Ntap-1; n>0; n--)
       x[n] = x[n-1];

    //Calculate the new output
    x[0] = NewSample;
    for(n=0; n<Ntap; n++)
        y += FIRCoef[n] * x[n];

    return y / FIRDCgain;
}

int16_t firLIDAR(int16_t NewSample) {

    static __int16 x[Ntap]; //input samples
    __int32 y=0;            //output sample
    int n;

    //shift the old samples
    for(n=Ntap-1; n>0; n--)
       x[n] = x[n-1];

    //Calculate the new output
    x[0] = NewSample;
    for(n=0; n<Ntap; n++)
        y += FIRCoef[n] * x[n];

    return y / FIRDCgain;
}

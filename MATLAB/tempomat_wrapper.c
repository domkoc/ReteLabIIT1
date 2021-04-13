
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void tempomat_Outputs_wrapper(const real_T *speed,
			const real_T *pedal,
			const real_T *onoff,
			real_T *output,
			const real_T *xD,
			const real_T *Kp, const int_T p_width0,
			const real_T *Ki, const int_T p_width1)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void tempomat_Update_wrapper(const real_T *speed,
			const real_T *pedal,
			const real_T *onoff,
			real_T *output,
			real_T *xD,
			const real_T *Kp, const int_T p_width0,
			const real_T *Ki, const int_T p_width1)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Code example
 *   xD[0] = u0[0];
 */
static double prevOnoff = 0;
static double baseSpeed = 0;

if(onoff[0] == 1){
    if (prevOnoff == 0){
        baseSpeed = speed[0];
        xD[0] = pedal[0] / Ki[0];
    }
    double ek = (baseSpeed - speed[0]);
    output[0] = Kp[0] * ek + Ki[0]* xD[0] + Ki[0] * 0.1 * ek;
    if(output[0] <=1 && output[0] >=0){
        xD[0] += 0.1 * ek;
    } else {
        xD[0] += 0;
        if (output[0] <=0){
            output[0] = 0;
        } else {
            output[0] =1;
        }
    }
} else {
    output[0] = pedal[0];
}
prevOnoff = onoff[0];
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}


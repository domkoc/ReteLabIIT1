/*
 * external_jarmu_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "external_jarmu".
 *
 * Model version              : 1.3
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C source code generated on : Mon Apr 12 18:46:32 2021
 *
 * Target selection: sldrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_external_jarmu_private_h_
#define RTW_HEADER_external_jarmu_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "zero_crossing_types.h"

real_T look_SplNBinCZcd(uint32_T numDims, const real_T* u, const
  rt_LUTSplineWork * const SWork);
void rt_Spline2Derivd(const real_T *x, const real_T *y, uint32_T n, real_T *u,
                      real_T *y2);
real_T intrp_NSplcd(uint32_T numDims, const rt_LUTSplineWork * const splWork,
                    uint32_T extrapMethod);
extern uint32_T plook_binc(real_T u, const real_T bp[], uint32_T maxIndex,
  real_T *fraction);
extern uint32_T binsearch_u32d(real_T u, const real_T bp[], uint32_T startIndex,
  uint32_T maxIndex);

/* private model entry point functions */
extern void external_jarmu_derivatives(void);

#endif                                 /* RTW_HEADER_external_jarmu_private_h_ */

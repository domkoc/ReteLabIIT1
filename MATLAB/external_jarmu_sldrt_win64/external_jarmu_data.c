/*
 * external_jarmu_data.c
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

#include "external_jarmu.h"
#include "external_jarmu_private.h"

/* Block parameters (default storage) */
P_external_jarmu_T external_jarmu_P = {
  /* Variable: CdA
   * Referenced by: '<Root>/Fd=1//2*CdA*p*v^2'
   */
  0.576,

  /* Variable: g
   * Referenced by: '<Root>/Gain'
   */
  9.81,

  /* Variable: m
   * Referenced by:
   *   '<Root>/Gain'
   *   '<Root>/Gain1'
   */
  2200.0,

  /* Variable: p
   * Referenced by: '<Root>/Fd=1//2*CdA*p*v^2'
   */
  1.2,

  /* Mask Parameter: StreamInput_MaxMissedTicks
   * Referenced by: '<S2>/Stream Input'
   */
  0.0,

  /* Mask Parameter: StreamOutput_MaxMissedTicks
   * Referenced by: '<S3>/Stream Output'
   */
  0.0,

  /* Mask Parameter: StreamInput_YieldWhenWaiting
   * Referenced by: '<S2>/Stream Input'
   */
  0.0,

  /* Mask Parameter: StreamOutput_YieldWhenWaiting
   * Referenced by: '<S3>/Stream Output'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S4>/Constant'
   */
  0.0,

  /* Expression: 37
   * Referenced by: '<Root>/Integrator'
   */
  37.0,

  /* Expression: 100
   * Referenced by: '<Root>/Integrator'
   */
  100.0,

  /* Expression: 0
   * Referenced by: '<Root>/Integrator'
   */
  0.0,

  /* Expression: 3.6
   * Referenced by: '<Root>/Gain2'
   */
  3.6,

  /* Expression: 0
   * Referenced by: '<Root>/Degree'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/TempomatOnOff'
   */
  0.0,

  /* Expression: [14500 7200 3800 1900]
   * Referenced by: '<Root>/F(v)'
   */
  { 14500.0, 7200.0, 3800.0, 1900.0 },

  /* Expression: [20 27 36 53]
   * Referenced by: '<Root>/F(v)'
   */
  { 20.0, 27.0, 36.0, 53.0 },

  /* Expression: pi/180
   * Referenced by: '<S1>/Gain1'
   */
  0.017453292519943295,

  /* Expression: 0
   * Referenced by: '<Root>/Pedal'
   */
  0.0,

  /* Computed Parameter: Fv_maxIndex
   * Referenced by: '<Root>/F(v)'
   */
  3U,

  /* Computed Parameter: Fv_dimSizes
   * Referenced by: '<Root>/F(v)'
   */
  1U,

  /* Computed Parameter: Fv_numYWorkElts
   * Referenced by: '<Root>/F(v)'
   */
  { 1U, 0U }
};

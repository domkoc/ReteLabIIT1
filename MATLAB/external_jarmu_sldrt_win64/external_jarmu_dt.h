/*
 * external_jarmu_dt.h
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

#include "ext_types.h"

/* data type size table */
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T)
};

/* data type name table */
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T"
};

/* data type transitions for block I/O structure */
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&external_jarmu_B.vkmh), 0, 0, 5 },

  { (char_T *)(&external_jarmu_B.Compare), 3, 0, 1 }
  ,

  { (char_T *)(&external_jarmu_DW.m_bpLambda), 0, 0, 20 },

  { (char_T *)
    (&external_jarmu_DW.HiddenToAsyncQueue_InsertedFor_Degree_at_outport_0_PWORK.AQHandles),
    11, 0, 23 },

  { (char_T *)(&external_jarmu_DW.m_bpIndex), 7, 0, 1 },

  { (char_T *)(&external_jarmu_DW.reCalcSecDerivFirstDimCoeffs), 3, 0, 1 }
};

/* data type transition table for block I/O structure */
static DataTypeTransitionTable rtBTransTable = {
  6U,
  rtBTransitions
};

/* data type transitions for Parameters structure */
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&external_jarmu_P.CdA), 0, 0, 25 },

  { (char_T *)(&external_jarmu_P.Fv_maxIndex), 7, 0, 4 }
};

/* data type transition table for Parameters structure */
static DataTypeTransitionTable rtPTransTable = {
  2U,
  rtPTransitions
};

/* [EOF] external_jarmu_dt.h */

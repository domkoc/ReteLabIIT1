/*
 * external_jarmu.c
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
#include "external_jarmu_dt.h"

/* options for Simulink Desktop Real-Time board 0 */
static double SLDRTBoardOptions0[] = {
  115200.0,
  8.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
};

/* list of Simulink Desktop Real-Time timers */
const int SLDRTTimerCount = 1;
const double SLDRTTimers[2] = {
  0.01, 0.0,
};

/* list of Simulink Desktop Real-Time boards */
const int SLDRTBoardCount = 1;
SLDRTBOARD SLDRTBoards[1] = {
  { "Standard_Devices/Serial_Port", 6U, 8, SLDRTBoardOptions0 },
};

/* Block signals (default storage) */
B_external_jarmu_T external_jarmu_B;

/* Continuous states */
X_external_jarmu_T external_jarmu_X;

/* Block states (default storage) */
DW_external_jarmu_T external_jarmu_DW;

/* Real-time model */
RT_MODEL_external_jarmu_T external_jarmu_M_;
RT_MODEL_external_jarmu_T *const external_jarmu_M = &external_jarmu_M_;
static void rate_scheduler(void);

/* n-D Spline interpolation function */
real_T look_SplNBinCZcd(uint32_T numDims, const real_T* u, const
  rt_LUTSplineWork * const SWork)
{
  /*
   *   n-D column-major table lookup operating on real_T with:
   *       - Spline interpolation
   *       - Clipping
   *       - Binary breakpoint search
   *       - Index search starts at the same place each time
   */
  rt_LUTnWork * const TWork_look = SWork->m_TWork;
  real_T* const fraction = (real_T*) TWork_look->m_bpLambda;
  uint32_T* const bpIdx = TWork_look->m_bpIndex;
  const uint32_T* const maxIndex = TWork_look->m_maxIndex;
  uint32_T k;
  for (k = 0U; k < numDims; k++) {
    const real_T* const bpData = ((const real_T * const *)
      TWork_look->m_bpDataSet)[k];
    bpIdx[k] = plook_binc(u[k], &bpData[0], maxIndex[k], &fraction[k]);
  }

  return(intrp_NSplcd(numDims, SWork, 1U));
}

/*
 * Second derivative initialization function for spline
 * for last dimension.
 */
void rt_Spline2Derivd(const real_T *x, const real_T *y, uint32_T n, real_T *u,
                      real_T *y2)
{
  real_T p, qn, sig, un;
  uint32_T n1, i, k;
  n1 = n - 1U;
  y2[0U] = 0.0;
  u[0U] = 0.0;
  for (i = 1U; i < n1; i++) {
    real_T dxm1 = x[i] - x[i - 1U];
    real_T dxp1 = x[i + 1U] - x[i];
    real_T dxpm = dxp1 + dxm1;
    sig = dxm1 / dxpm;
    p = (sig * y2[i - 1U]) + 2.0;
    y2[i] = (sig - 1.0) / p;
    u[i] = ((y[i + 1U] - y[i]) / dxp1) - ((y[i] - y[i - 1U]) / dxm1);
    u[i] = (((6.0 * u[i]) / dxpm) - (sig * u[i - 1U])) / p;
  }

  qn = 0.0;
  un = 0.0;
  y2[n1] = (un - (qn * u[n1 - 1U])) / ((qn * y2[n1 - 1U]) + 1.0);
  for (k = n1; k > 0U; k--) {
    y2[k-1U] = (y2[k-1U] * y2[k]) + u[k-1U];
  }

  return;
}

/* n-D natural spline calculation function */
real_T intrp_NSplcd(uint32_T numDims, const rt_LUTSplineWork * const splWork,
                    uint32_T extrapMethod)
{
  uint32_T il;
  uint32_T iu, k, i;
  real_T h, s, p, smsq, pmsq;

  /* intermediate results work areas "this" and "next" */
  const rt_LUTnWork *TWork_interp = (const rt_LUTnWork *)splWork->m_TWork;
  const real_T *fraction = (real_T *) TWork_interp->m_bpLambda;
  const real_T *yp = (real_T *) TWork_interp->m_tableData;
  real_T *yyA = (real_T *) splWork->m_yyA;
  real_T *yyB = (real_T *) splWork->m_yyB;
  real_T *yy2 = (real_T *) splWork->m_yy2;
  real_T *up = (real_T *) splWork->m_up;
  real_T *y2 = (real_T *) splWork->m_y2;
  uint8_T* reCalc = splWork->m_reCalc;
  real_T *dp = (real_T *) splWork->m_preBp0AndTable;
  const real_T **bpDataSet = (const real_T **) TWork_interp->m_bpDataSet;
  const real_T *xp = bpDataSet[0U];
  real_T *yy = yyA;
  uint32_T bufBank = 0U;
  uint32_T len = TWork_interp->m_maxIndex[0U] + 1U;

  /* compare bp0 and table to see whether they get changed */
  {
    /* compare the bp0 data */
    if (memcmp(dp, xp,
               len * sizeof(real_T)) != 0) {
      *reCalc = 1;
      (void) memcpy(dp, xp,
                    len * sizeof(real_T));
    }

    /* compare the table data */
    dp = &(dp[len]);
    if (memcmp(dp, yp,
               len * splWork->m_numYWorkElts[0U] * sizeof(real_T)) != 0) {
      *reCalc = 1;
      (void) memcpy(dp, yp,
                    len * splWork->m_numYWorkElts[0U] * sizeof(real_T));
    }
  }

  if (*reCalc == 1) {
    /* If table and bps are tunable calculate 1st dim 2nd deriv */
    /* Generate first dimension's second derivatives */
    for (i = 0U; i < splWork->m_numYWorkElts[0U]; i++) {
      rt_Spline2Derivd(xp, yp, len, up, y2);
      yp = &yp[len];
      y2 = &y2[len];
    }

    /* Set pointers back to beginning */
    yp = (const real_T *) TWork_interp->m_tableData;
    y2 = (real_T *) splWork->m_y2;
  }

  *reCalc = 0;

  /* Generate at-point splines in each dimension */
  for (k = 0U; k < numDims; k++ ) {
    /* this dimension's input setup */
    xp = bpDataSet[k];
    len = TWork_interp->m_maxIndex[k] + 1U;
    il = TWork_interp->m_bpIndex[k];
    iu = il + 1U;
    h = xp[iu] - xp[il];
    p = fraction[k];
    s = 1.0 - p;
    pmsq = p * ((p*p) - 1.0);
    smsq = s * ((s*s) - 1.0);

    /*
     * Calculate spline curves for input in this
     * dimension at each value of the higher
     * other dimensions\' points in the table.
     */
    if ((p > 1.0) && (extrapMethod == 2U) ) {
      real_T slope;
      for (i = 0U; i < splWork->m_numYWorkElts[k]; i++) {
        slope = (yp[iu] - yp[il]) + ((y2[il]*h*h)*(1.0/6.0));
        yy[i] = yp[iu] + (slope * (p-1.0));
        yp = &yp[len];
        y2 = &y2[len];
      }
    } else if ((p < 0.0) && (extrapMethod == 2U) ) {
      real_T slope;
      for (i = 0U; i < splWork->m_numYWorkElts[k]; i++) {
        slope = (yp[iu] - yp[il]) - ((y2[iu]*h*h)*(1.0/6.0));
        yy[i] = yp[il] + (slope * p);
        yp = &yp[len];
        y2 = &y2[len];
      }
    } else {
      for (i = 0U; i < splWork->m_numYWorkElts[k]; i++) {
        yy[i] = yp[il] + p * (yp[iu] - yp[il]) +
          ((smsq * y2[il] + pmsq * y2[iu])*h*h)*(1.0/6.0);
        yp = &yp[len];
        y2 = &y2[len];
      }
    }

    /* set pointers to new result and calculate second derivatives */
    yp = yy;
    y2 = yy2;
    if (splWork->m_numYWorkElts[k+1U] > 0U ) {
      uint32_T nextLen = TWork_interp->m_maxIndex[k+1U] + 1U;
      const real_T *nextXp = bpDataSet[k+1U];
      for (i = 0U; i < splWork->m_numYWorkElts[k+1U]; i++) {
        rt_Spline2Derivd(nextXp, yp, nextLen, up, y2);
        yp = &yp[nextLen];
        y2 = &y2[nextLen];
      }
    }

    /*
     * Set work vectors yp, y2 and yy for next iteration;
     * the yy just calculated becomes the yp in the
     * next iteration, y2 was just calculated for these
     * new points and the yy buffer is swapped to the space
     * for storing the next iteration\'s results.
     */
    yp = yy;
    y2 = yy2;

    /*
     * Swap buffers for next dimension and
     * toggle bufBank for next iteration.
     */
    if (bufBank == 0U) {
      yy = yyA;
      bufBank = 1U;
    } else {
      yy = yyB;
      bufBank = 0U;
    }
  }

  return( yp[0U] );
}

uint32_T plook_binc(real_T u, const real_T bp[], uint32_T maxIndex, real_T
                    *fraction)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32d(u, bp, maxIndex >> 1U, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = 1.0;
  }

  return bpIndex;
}

uint32_T binsearch_u32d(real_T u, const real_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T iRght;
  uint32_T bpIdx;

  /* Binary Search */
  bpIdx = startIndex;
  bpIndex = 0U;
  iRght = maxIndex;
  while (iRght - bpIndex > 1U) {
    if (u < bp[bpIdx]) {
      iRght = bpIdx;
    } else {
      bpIndex = bpIdx;
    }

    bpIdx = (iRght + bpIndex) >> 1U;
  }

  return bpIndex;
}

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (external_jarmu_M->Timing.TaskCounters.TID[2])++;
  if ((external_jarmu_M->Timing.TaskCounters.TID[2]) > 9) {/* Sample time: [0.1s, 0.0s] */
    external_jarmu_M->Timing.TaskCounters.TID[2] = 0;
  }

  external_jarmu_M->Timing.sampleHits[2] =
    (external_jarmu_M->Timing.TaskCounters.TID[2] == 0);
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 1;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  external_jarmu_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  external_jarmu_output();
  external_jarmu_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  external_jarmu_output();
  external_jarmu_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model output function */
void external_jarmu_output(void)
{
  /* local block i/o variables */
  real_T rtb_v;
  real_T rtb_Fmax;
  if (rtmIsMajorTimeStep(external_jarmu_M)) {
    /* set solver stop time */
    if (!(external_jarmu_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&external_jarmu_M->solverInfo,
                            ((external_jarmu_M->Timing.clockTickH0 + 1) *
        external_jarmu_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&external_jarmu_M->solverInfo,
                            ((external_jarmu_M->Timing.clockTick0 + 1) *
        external_jarmu_M->Timing.stepSize0 +
        external_jarmu_M->Timing.clockTickH0 *
        external_jarmu_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(external_jarmu_M)) {
    external_jarmu_M->Timing.t[0] = rtsiGetT(&external_jarmu_M->solverInfo);
  }

  /* Integrator: '<Root>/Integrator' */
  /* Limited  Integrator  */
  if (external_jarmu_X.Integrator_CSTATE >= external_jarmu_P.Integrator_UpperSat)
  {
    external_jarmu_X.Integrator_CSTATE = external_jarmu_P.Integrator_UpperSat;
  } else {
    if (external_jarmu_X.Integrator_CSTATE <=
        external_jarmu_P.Integrator_LowerSat) {
      external_jarmu_X.Integrator_CSTATE = external_jarmu_P.Integrator_LowerSat;
    }
  }

  rtb_v = external_jarmu_X.Integrator_CSTATE;

  /* End of Integrator: '<Root>/Integrator' */

  /* Gain: '<Root>/Gain2' */
  external_jarmu_B.vkmh = external_jarmu_P.Gain2_Gain * rtb_v;
  if (rtmIsMajorTimeStep(external_jarmu_M) &&
      external_jarmu_M->Timing.TaskCounters.TID[1] == 0) {
    /* Constant: '<Root>/Degree' */
    external_jarmu_B.degAlpha = external_jarmu_P.Degree_Value;

    /* ToAsyncQueueBlock: '<Root>/HiddenToAsyncQueue_InsertedFor_Degree_at_outport_0' */
    if (rtmIsMajorTimeStep(external_jarmu_M)) {
      {
        double time = external_jarmu_M->Timing.t[1];
        void *pData = (void *)&external_jarmu_B.degAlpha;
        int32_T size = 1*sizeof(real_T);
        sendToAsyncQueueTgtAppSvc(2083318501U, time, pData, size);
      }
    }

    /* ToAsyncQueueBlock: '<Root>/HiddenToAsyncQueue_InsertedFor_Gain2_at_outport_0' */
    if (rtmIsMajorTimeStep(external_jarmu_M)) {
      {
        double time = external_jarmu_M->Timing.t[1];
        void *pData = (void *)&external_jarmu_B.vkmh;
        int32_T size = 1*sizeof(real_T);
        sendToAsyncQueueTgtAppSvc(697992781U, time, pData, size);
      }
    }
  }

  if (rtmIsMajorTimeStep(external_jarmu_M) &&
      external_jarmu_M->Timing.TaskCounters.TID[2] == 0) {
    /* Outputs for Atomic SubSystem: '<Root>/Vehicle Receive' */
    /* S-Function (sldrtsi): '<S2>/Stream Input' */
    /* S-Function Block: <S2>/Stream Input */
    {
      char indata[329U];
      int status;
      const char* terminators = "\r\0\n\0";
      memset(indata, 0, sizeof(indata));
      status = RTBIO_DriverIO(0, STREAMINPUT, IOREADWITHRESET, 328U, NULL,
        (double*) indata, terminators);
      if (status & 0x1) {
        real32_T var0 = 0;
        sscanf(indata, "%f", &var0);
        external_jarmu_B.StreamInput = (real_T) var0;
      }
    }

    /* End of Outputs for SubSystem: '<Root>/Vehicle Receive' */

    /* ToAsyncQueueBlock: '<Root>/HiddenToAsyncQueue_InsertedFor_Vehicle Receive_at_outport_0' */
    if (rtmIsMajorTimeStep(external_jarmu_M)) {
      {
        double time = external_jarmu_M->Timing.t[2];
        void *pData = (void *)&external_jarmu_B.StreamInput;
        int32_T size = 1*sizeof(real_T);
        sendToAsyncQueueTgtAppSvc(3964772606U, time, pData, size);
      }
    }

    /* Outputs for Atomic SubSystem: '<Root>/Vehicle Send' */
    /* S-Function (sldrtso): '<S3>/Stream Output' incorporates:
     *  Constant: '<Root>/Pedal'
     */
    /* S-Function Block: <S3>/Stream Output */

    /* no code required */

    /* RelationalOperator: '<S4>/Compare' incorporates:
     *  Constant: '<Root>/TempomatOnOff'
     *  Constant: '<S4>/Constant'
     */
    external_jarmu_B.Compare = (uint8_T)(external_jarmu_P.TempomatOnOff_Value >
      external_jarmu_P.Constant_Value);

    /* End of Outputs for SubSystem: '<Root>/Vehicle Send' */
  }

  /* Lookup_n-D: '<Root>/F(v)' */
  /*
   * About '<Root>/F(v)':
   *       Table size:  4
   *    Interpolation:  Spline
   *    Extrapolation:  None - Clip
   *   Breakpt Search:  Binary
   *    Breakpt Cache:  OFF
   */
  rtb_Fmax = look_SplNBinCZcd(1U, &rtb_v, (rt_LUTSplineWork*)
    &external_jarmu_DW.SWork[0]);
  if (rtmIsMajorTimeStep(external_jarmu_M) &&
      external_jarmu_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<Root>/Gain' incorporates:
     *  Gain: '<S1>/Gain1'
     *  Trigonometry: '<Root>/Trigonometric Function'
     */
    external_jarmu_B.Gain = external_jarmu_P.m * external_jarmu_P.g * sin
      (external_jarmu_P.Gain1_Gain * external_jarmu_B.degAlpha);
  }

  /* Gain: '<Root>/Gain1' incorporates:
   *  Gain: '<Root>/Fd=1//2*CdA*p*v^2'
   *  Math: '<Root>/v^2'
   *  Product: '<Root>/Fm=Fmax(v)*T'
   *  Sum: '<Root>/Add'
   */
  external_jarmu_B.v = ((rtb_Fmax * external_jarmu_B.StreamInput - 0.5 *
    external_jarmu_P.CdA * external_jarmu_P.p * (rtb_v * rtb_v)) -
                        external_jarmu_B.Gain) * (1.0 / external_jarmu_P.m);
}

/* Model update function */
void external_jarmu_update(void)
{
  if (rtmIsMajorTimeStep(external_jarmu_M) &&
      external_jarmu_M->Timing.TaskCounters.TID[2] == 0) {
    /* Update for Atomic SubSystem: '<Root>/Vehicle Send' */
    /* Update for S-Function (sldrtso): '<S3>/Stream Output' incorporates:
     *  Constant: '<Root>/Pedal'
     */

    /* S-Function Block: <S3>/Stream Output */
    {
      char outstring[663U];
      int n = snprintf(outstring, 663U, "%011.7f:%011.7f:%d", (real_T)
                       external_jarmu_B.vkmh, (real_T)
                       external_jarmu_P.Pedal_Value, (int32_T)
                       external_jarmu_B.Compare);
      RTBIO_DriverIO(0, STREAMOUTPUT, IOWRITE, n, NULL, (double*) outstring,
                     NULL);
    }

    /* End of Update for SubSystem: '<Root>/Vehicle Send' */
  }

  if (rtmIsMajorTimeStep(external_jarmu_M)) {
    rt_ertODEUpdateContinuousStates(&external_jarmu_M->solverInfo);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++external_jarmu_M->Timing.clockTick0)) {
    ++external_jarmu_M->Timing.clockTickH0;
  }

  external_jarmu_M->Timing.t[0] = rtsiGetSolverStopTime
    (&external_jarmu_M->solverInfo);

  {
    /* Update absolute timer for sample time: [0.01s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++external_jarmu_M->Timing.clockTick1)) {
      ++external_jarmu_M->Timing.clockTickH1;
    }

    external_jarmu_M->Timing.t[1] = external_jarmu_M->Timing.clockTick1 *
      external_jarmu_M->Timing.stepSize1 + external_jarmu_M->Timing.clockTickH1 *
      external_jarmu_M->Timing.stepSize1 * 4294967296.0;
  }

  if (rtmIsMajorTimeStep(external_jarmu_M) &&
      external_jarmu_M->Timing.TaskCounters.TID[2] == 0) {
    /* Update absolute timer for sample time: [0.1s, 0.0s] */
    /* The "clockTick2" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick2"
     * and "Timing.stepSize2". Size of "clockTick2" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick2 and the high bits
     * Timing.clockTickH2. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++external_jarmu_M->Timing.clockTick2)) {
      ++external_jarmu_M->Timing.clockTickH2;
    }

    external_jarmu_M->Timing.t[2] = external_jarmu_M->Timing.clockTick2 *
      external_jarmu_M->Timing.stepSize2 + external_jarmu_M->Timing.clockTickH2 *
      external_jarmu_M->Timing.stepSize2 * 4294967296.0;
  }

  rate_scheduler();
}

/* Derivatives for root system: '<Root>' */
void external_jarmu_derivatives(void)
{
  boolean_T lsat;
  boolean_T usat;
  XDot_external_jarmu_T *_rtXdot;
  _rtXdot = ((XDot_external_jarmu_T *) external_jarmu_M->derivs);

  /* Derivatives for Integrator: '<Root>/Integrator' */
  lsat = (external_jarmu_X.Integrator_CSTATE <=
          external_jarmu_P.Integrator_LowerSat);
  usat = (external_jarmu_X.Integrator_CSTATE >=
          external_jarmu_P.Integrator_UpperSat);
  if (((!lsat) && (!usat)) || (lsat && (external_jarmu_B.v > 0.0)) || (usat &&
       (external_jarmu_B.v < 0.0))) {
    _rtXdot->Integrator_CSTATE = external_jarmu_B.v;
  } else {
    /* in saturation */
    _rtXdot->Integrator_CSTATE = 0.0;
  }

  /* End of Derivatives for Integrator: '<Root>/Integrator' */
}

/* Model initialize function */
void external_jarmu_initialize(void)
{
  /* Start for Atomic SubSystem: '<Root>/Vehicle Send' */

  /* Start for S-Function (sldrtso): '<S3>/Stream Output' incorporates:
   *  Constant: '<Root>/Pedal'
   */

  /* S-Function Block: <S3>/Stream Output */
  /* no initial value should be set */

  /* End of Start for SubSystem: '<Root>/Vehicle Send' */

  /* Start for Lookup_n-D: '<Root>/F(v)' */
  {
    rt_LUTnWork *TWork_start = (rt_LUTnWork *) &external_jarmu_DW.TWork[0];
    void **bpDataSet = (void **) &external_jarmu_DW.m_bpDataSet;
    TWork_start->m_dimSizes = (const uint32_T *) &external_jarmu_P.Fv_dimSizes;
    TWork_start->m_tableData = (void *) external_jarmu_P.Fv_tableData;
    TWork_start->m_bpDataSet = bpDataSet;
    TWork_start->m_bpIndex = &external_jarmu_DW.m_bpIndex;
    TWork_start->m_bpLambda = &external_jarmu_DW.m_bpLambda;
    TWork_start->m_maxIndex = (const uint32_T *) &external_jarmu_P.Fv_maxIndex;
    bpDataSet[0] = (void *) external_jarmu_P.Fv_bp01Data;
  }

  {
    const real_T **bpDataSet;
    const real_T *xp, *yp;
    real_T *dp;
    uint32_T len;
    const rt_LUTnWork *TWork_interp;
    rt_LUTSplineWork *rt_SplWk = (rt_LUTSplineWork*)&external_jarmu_DW.SWork[0];
    rt_SplWk->m_TWork = (rt_LUTnWork*)&external_jarmu_DW.TWork[0];
    rt_SplWk->m_yyA = &external_jarmu_DW.m_yyA;
    rt_SplWk->m_yyB = &external_jarmu_DW.m_yyB;
    rt_SplWk->m_yy2 = &external_jarmu_DW.m_yy2;
    rt_SplWk->m_up = &external_jarmu_DW.m_up[0];
    rt_SplWk->m_y2 = &external_jarmu_DW.m_y2[0];
    rt_SplWk->m_numYWorkElts = external_jarmu_P.Fv_numYWorkElts;
    rt_SplWk->m_reCalc = &external_jarmu_DW.reCalcSecDerivFirstDimCoeffs;
    rt_SplWk->m_preBp0AndTable = &external_jarmu_DW.prevBp0AndTableData[0];
    *rt_SplWk->m_reCalc = 1;

    /* cache table data and first breakpoint data */
    TWork_interp = (const rt_LUTnWork *)rt_SplWk->m_TWork;
    bpDataSet = (const real_T **) TWork_interp->m_bpDataSet;
    xp = bpDataSet[0U];
    len = TWork_interp->m_maxIndex[0U] + 1U;
    dp = (real_T *) rt_SplWk->m_preBp0AndTable;
    yp = (real_T *) TWork_interp->m_tableData;
    (void) memcpy(dp, xp,
                  len * sizeof(real_T));
    dp = &(dp[len]);

    /* save the table data */
    (void) memcpy(dp, yp,
                  len * rt_SplWk->m_numYWorkElts[0U] * sizeof(real_T));
  }

  /* InitializeConditions for Integrator: '<Root>/Integrator' */
  external_jarmu_X.Integrator_CSTATE = external_jarmu_P.Integrator_IC;
}

/* Model terminate function */
void external_jarmu_terminate(void)
{
  /* Terminate for Atomic SubSystem: '<Root>/Vehicle Send' */

  /* Terminate for S-Function (sldrtso): '<S3>/Stream Output' incorporates:
   *  Constant: '<Root>/Pedal'
   */

  /* no final value should be set */

  /* End of Terminate for SubSystem: '<Root>/Vehicle Send' */
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/

/* Solver interface called by GRT_Main */
#ifndef USE_GENERATED_SOLVER

void rt_ODECreateIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEDestroyIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEUpdateContinuousStates(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

#endif

void MdlOutputs(int_T tid)
{
  external_jarmu_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  external_jarmu_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  external_jarmu_initialize();
}

void MdlTerminate(void)
{
  external_jarmu_terminate();
}

/* Registration function */
RT_MODEL_external_jarmu_T *external_jarmu(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)external_jarmu_M, 0,
                sizeof(RT_MODEL_external_jarmu_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&external_jarmu_M->solverInfo,
                          &external_jarmu_M->Timing.simTimeStep);
    rtsiSetTPtr(&external_jarmu_M->solverInfo, &rtmGetTPtr(external_jarmu_M));
    rtsiSetStepSizePtr(&external_jarmu_M->solverInfo,
                       &external_jarmu_M->Timing.stepSize0);
    rtsiSetdXPtr(&external_jarmu_M->solverInfo, &external_jarmu_M->derivs);
    rtsiSetContStatesPtr(&external_jarmu_M->solverInfo, (real_T **)
                         &external_jarmu_M->contStates);
    rtsiSetNumContStatesPtr(&external_jarmu_M->solverInfo,
      &external_jarmu_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&external_jarmu_M->solverInfo,
      &external_jarmu_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&external_jarmu_M->solverInfo,
      &external_jarmu_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&external_jarmu_M->solverInfo,
      &external_jarmu_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&external_jarmu_M->solverInfo, (&rtmGetErrorStatus
      (external_jarmu_M)));
    rtsiSetRTModelPtr(&external_jarmu_M->solverInfo, external_jarmu_M);
  }

  rtsiSetSimTimeStep(&external_jarmu_M->solverInfo, MAJOR_TIME_STEP);
  external_jarmu_M->intgData.y = external_jarmu_M->odeY;
  external_jarmu_M->intgData.f[0] = external_jarmu_M->odeF[0];
  external_jarmu_M->intgData.f[1] = external_jarmu_M->odeF[1];
  external_jarmu_M->intgData.f[2] = external_jarmu_M->odeF[2];
  external_jarmu_M->contStates = ((real_T *) &external_jarmu_X);
  rtsiSetSolverData(&external_jarmu_M->solverInfo, (void *)
                    &external_jarmu_M->intgData);
  rtsiSetSolverName(&external_jarmu_M->solverInfo,"ode3");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = external_jarmu_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    mdlTsMap[2] = 2;
    external_jarmu_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    external_jarmu_M->Timing.sampleTimes =
      (&external_jarmu_M->Timing.sampleTimesArray[0]);
    external_jarmu_M->Timing.offsetTimes =
      (&external_jarmu_M->Timing.offsetTimesArray[0]);

    /* task periods */
    external_jarmu_M->Timing.sampleTimes[0] = (0.0);
    external_jarmu_M->Timing.sampleTimes[1] = (0.01);
    external_jarmu_M->Timing.sampleTimes[2] = (0.1);

    /* task offsets */
    external_jarmu_M->Timing.offsetTimes[0] = (0.0);
    external_jarmu_M->Timing.offsetTimes[1] = (0.0);
    external_jarmu_M->Timing.offsetTimes[2] = (0.0);
  }

  rtmSetTPtr(external_jarmu_M, &external_jarmu_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = external_jarmu_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    mdlSampleHits[2] = 1;
    external_jarmu_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(external_jarmu_M, -1);
  external_jarmu_M->Timing.stepSize0 = 0.01;
  external_jarmu_M->Timing.stepSize1 = 0.01;
  external_jarmu_M->Timing.stepSize2 = 0.1;

  /* External mode info */
  external_jarmu_M->Sizes.checksums[0] = (3855837708U);
  external_jarmu_M->Sizes.checksums[1] = (2029492344U);
  external_jarmu_M->Sizes.checksums[2] = (2831711816U);
  external_jarmu_M->Sizes.checksums[3] = (3293837445U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[3];
    external_jarmu_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(external_jarmu_M->extModeInfo,
      &external_jarmu_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(external_jarmu_M->extModeInfo,
                        external_jarmu_M->Sizes.checksums);
    rteiSetTPtr(external_jarmu_M->extModeInfo, rtmGetTPtr(external_jarmu_M));
  }

  external_jarmu_M->solverInfoPtr = (&external_jarmu_M->solverInfo);
  external_jarmu_M->Timing.stepSize = (0.01);
  rtsiSetFixedStepSize(&external_jarmu_M->solverInfo, 0.01);
  rtsiSetSolverMode(&external_jarmu_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  external_jarmu_M->blockIO = ((void *) &external_jarmu_B);
  (void) memset(((void *) &external_jarmu_B), 0,
                sizeof(B_external_jarmu_T));

  /* parameters */
  external_jarmu_M->defaultParam = ((real_T *)&external_jarmu_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &external_jarmu_X;
    external_jarmu_M->contStates = (x);
    (void) memset((void *)&external_jarmu_X, 0,
                  sizeof(X_external_jarmu_T));
  }

  /* states (dwork) */
  external_jarmu_M->dwork = ((void *) &external_jarmu_DW);
  (void) memset((void *)&external_jarmu_DW, 0,
                sizeof(DW_external_jarmu_T));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    external_jarmu_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 14;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Initialize Sizes */
  external_jarmu_M->Sizes.numContStates = (1);/* Number of continuous states */
  external_jarmu_M->Sizes.numPeriodicContStates = (0);/* Number of periodic continuous states */
  external_jarmu_M->Sizes.numY = (0);  /* Number of model outputs */
  external_jarmu_M->Sizes.numU = (0);  /* Number of model inputs */
  external_jarmu_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  external_jarmu_M->Sizes.numSampTimes = (3);/* Number of sample times */
  external_jarmu_M->Sizes.numBlocks = (24);/* Number of blocks */
  external_jarmu_M->Sizes.numBlockIO = (6);/* Number of block outputs */
  external_jarmu_M->Sizes.numBlockPrms = (29);/* Sum of parameter "widths" */
  return external_jarmu_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/

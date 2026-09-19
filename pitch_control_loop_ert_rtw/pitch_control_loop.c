/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: pitch_control_loop.c
 *
 * Code generated for Simulink model 'pitch_control_loop'.
 *
 * Model version                  : 1.2
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Sat Sep 19 21:23:16 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "pitch_control_loop.h"
#include <math.h>
#include "rtwtypes.h"
#include "pitch_control_loop_private.h"

/* Block signals (default storage) */
B_pitch_control_loop_T pitch_control_loop_B;

/* Continuous states */
X_pitch_control_loop_T pitch_control_loop_X;

/* Disabled State Vector */
XDis_pitch_control_loop_T pitch_control_loop_XDis;

/* Block states (default storage) */
DW_pitch_control_loop_T pitch_control_loop_DW;

/* External outputs (root outports fed by signals with default storage) */
ExtY_pitch_control_loop_T pitch_control_loop_Y;

/* Real-time model */
static RT_MODEL_pitch_control_loop_T pitch_control_loop_M_;
RT_MODEL_pitch_control_loop_T *const pitch_control_loop_M =
  &pitch_control_loop_M_;

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 2;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  pitch_control_loop_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  pitch_control_loop_step();
  pitch_control_loop_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  pitch_control_loop_step();
  pitch_control_loop_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  pitch_control_loop_step();
  pitch_control_loop_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void pitch_control_loop_step(void)
{
  real_T err_sum;
  real_T rtb_Clock;
  if (rtmIsMajorTimeStep(pitch_control_loop_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&pitch_control_loop_M->solverInfo,
                          ((pitch_control_loop_M->Timing.clockTick0+1)*
      pitch_control_loop_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(pitch_control_loop_M)) {
    pitch_control_loop_M->Timing.t[0] = rtsiGetT
      (&pitch_control_loop_M->solverInfo);
  }

  /* Outport: '<Root>/theta_out' incorporates:
   *  Integrator: '<Root>/theta_integrator'
   */
  pitch_control_loop_Y.theta_out = pitch_control_loop_X.theta_integrator_CSTATE;

  /* Clock: '<Root>/Clock' */
  rtb_Clock = pitch_control_loop_M->Timing.t[0];

  /* MATLAB Function: '<Root>/pitch_program_block' */
  if (rtb_Clock < 10.0) {
    rtb_Clock = 90.0;
  } else if (rtb_Clock < 15.0) {
    rtb_Clock = 90.0 - (rtb_Clock - 10.0) / 5.0 * 3.0;
  } else if (rtb_Clock < 260.0) {
    rtb_Clock = (rtb_Clock - 15.0) / 245.0 * -81.0 + 87.0;
  } else {
    rtb_Clock = 6.0;
  }

  /* Sum: '<Root>/err_sum' incorporates:
   *  Integrator: '<Root>/theta_integrator'
   *  MATLAB Function: '<Root>/pitch_program_block'
   */
  err_sum = rtb_Clock * 3.1415926535897931 / 180.0 -
    pitch_control_loop_X.theta_integrator_CSTATE;
  if (rtmIsMajorTimeStep(pitch_control_loop_M)) {
    /* MATLAB Function: '<Root>/pid_controller_block' incorporates:
     *  ZeroOrderHold: '<Root>/pid_input_zoh'
     */
    pitch_control_loop_DW.integral += err_sum * 0.05;
    if (pitch_control_loop_DW.first_call) {
      rtb_Clock = 0.0;
    } else {
      rtb_Clock = (err_sum - pitch_control_loop_DW.prev_err) / 0.05;
    }

    pitch_control_loop_B.Mc = fmax(fmin((800000.0 * err_sum + 5000.0 *
      pitch_control_loop_DW.integral) + 250000.0 * rtb_Clock, 2.0E+6), -2.0E+6);
    pitch_control_loop_DW.prev_err = err_sum;
    pitch_control_loop_DW.first_call = false;

    /* End of MATLAB Function: '<Root>/pid_controller_block' */
  }

  /* Integrator: '<Root>/theta_dot_integrator' */
  pitch_control_loop_B.theta_dot_integrator =
    pitch_control_loop_X.theta_dot_integrator_CSTATE;

  /* Gain: '<Root>/inv_inertia_gain' incorporates:
   *  Gain: '<Root>/damping_gain'
   *  Sum: '<Root>/moment_sum'
   */
  pitch_control_loop_B.inv_inertia_gain = (pitch_control_loop_B.Mc - 50000.0 *
    pitch_control_loop_B.theta_dot_integrator) * 8.3333333333333337E-6;
  if (rtmIsMajorTimeStep(pitch_control_loop_M)) {
    rt_ertODEUpdateContinuousStates(&pitch_control_loop_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++pitch_control_loop_M->Timing.clockTick0;
    pitch_control_loop_M->Timing.t[0] = rtsiGetSolverStopTime
      (&pitch_control_loop_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.05s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.05, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      pitch_control_loop_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void pitch_control_loop_derivatives(void)
{
  XDot_pitch_control_loop_T *_rtXdot;
  _rtXdot = ((XDot_pitch_control_loop_T *) pitch_control_loop_M->derivs);

  /* Derivatives for Integrator: '<Root>/theta_integrator' */
  _rtXdot->theta_integrator_CSTATE = pitch_control_loop_B.theta_dot_integrator;

  /* Derivatives for Integrator: '<Root>/theta_dot_integrator' */
  _rtXdot->theta_dot_integrator_CSTATE = pitch_control_loop_B.inv_inertia_gain;
}

/* Model initialize function */
void pitch_control_loop_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&pitch_control_loop_M->solverInfo,
                          &pitch_control_loop_M->Timing.simTimeStep);
    rtsiSetTPtr(&pitch_control_loop_M->solverInfo, &rtmGetTPtr
                (pitch_control_loop_M));
    rtsiSetStepSizePtr(&pitch_control_loop_M->solverInfo,
                       &pitch_control_loop_M->Timing.stepSize0);
    rtsiSetdXPtr(&pitch_control_loop_M->solverInfo,
                 &pitch_control_loop_M->derivs);
    rtsiSetContStatesPtr(&pitch_control_loop_M->solverInfo, (real_T **)
                         &pitch_control_loop_M->contStates);
    rtsiSetNumContStatesPtr(&pitch_control_loop_M->solverInfo,
      &pitch_control_loop_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&pitch_control_loop_M->solverInfo,
      &pitch_control_loop_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&pitch_control_loop_M->solverInfo,
      &pitch_control_loop_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&pitch_control_loop_M->solverInfo,
      &pitch_control_loop_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&pitch_control_loop_M->solverInfo, (boolean_T**)
      &pitch_control_loop_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&pitch_control_loop_M->solverInfo, (&rtmGetErrorStatus
      (pitch_control_loop_M)));
    rtsiSetRTModelPtr(&pitch_control_loop_M->solverInfo, pitch_control_loop_M);
  }

  rtsiSetSimTimeStep(&pitch_control_loop_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&pitch_control_loop_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&pitch_control_loop_M->solverInfo, false);
  pitch_control_loop_M->intgData.y = pitch_control_loop_M->odeY;
  pitch_control_loop_M->intgData.f[0] = pitch_control_loop_M->odeF[0];
  pitch_control_loop_M->intgData.f[1] = pitch_control_loop_M->odeF[1];
  pitch_control_loop_M->intgData.f[2] = pitch_control_loop_M->odeF[2];
  pitch_control_loop_M->intgData.f[3] = pitch_control_loop_M->odeF[3];
  pitch_control_loop_M->contStates = ((X_pitch_control_loop_T *)
    &pitch_control_loop_X);
  pitch_control_loop_M->contStateDisabled = ((XDis_pitch_control_loop_T *)
    &pitch_control_loop_XDis);
  pitch_control_loop_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&pitch_control_loop_M->solverInfo, (void *)
                    &pitch_control_loop_M->intgData);
  rtsiSetSolverName(&pitch_control_loop_M->solverInfo,"ode4");
  rtmSetTPtr(pitch_control_loop_M, &pitch_control_loop_M->Timing.tArray[0]);
  pitch_control_loop_M->Timing.stepSize0 = 0.05;

  /* InitializeConditions for Integrator: '<Root>/theta_integrator' */
  pitch_control_loop_X.theta_integrator_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<Root>/theta_dot_integrator' */
  pitch_control_loop_X.theta_dot_integrator_CSTATE = 0.0;

  /* SystemInitialize for MATLAB Function: '<Root>/pid_controller_block' */
  pitch_control_loop_DW.first_call = true;
}

/* Model terminate function */
void pitch_control_loop_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

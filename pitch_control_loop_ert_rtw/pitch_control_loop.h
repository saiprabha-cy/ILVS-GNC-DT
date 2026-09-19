/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: pitch_control_loop.h
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

#ifndef pitch_control_loop_h_
#define pitch_control_loop_h_
#ifndef pitch_control_loop_COMMON_INCLUDES_
#define pitch_control_loop_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                                 /* pitch_control_loop_COMMON_INCLUDES_ */

#include "pitch_control_loop_types.h"
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T theta_dot_integrator;         /* '<Root>/theta_dot_integrator' */
  real_T inv_inertia_gain;             /* '<Root>/inv_inertia_gain' */
  real_T Mc;                           /* '<Root>/pid_controller_block' */
} B_pitch_control_loop_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T integral;                     /* '<Root>/pid_controller_block' */
  real_T prev_err;                     /* '<Root>/pid_controller_block' */
  boolean_T first_call;                /* '<Root>/pid_controller_block' */
} DW_pitch_control_loop_T;

/* Continuous states (default storage) */
typedef struct {
  real_T theta_integrator_CSTATE;      /* '<Root>/theta_integrator' */
  real_T theta_dot_integrator_CSTATE;  /* '<Root>/theta_dot_integrator' */
} X_pitch_control_loop_T;

/* State derivatives (default storage) */
typedef struct {
  real_T theta_integrator_CSTATE;      /* '<Root>/theta_integrator' */
  real_T theta_dot_integrator_CSTATE;  /* '<Root>/theta_dot_integrator' */
} XDot_pitch_control_loop_T;

/* State disabled  */
typedef struct {
  boolean_T theta_integrator_CSTATE;   /* '<Root>/theta_integrator' */
  boolean_T theta_dot_integrator_CSTATE;/* '<Root>/theta_dot_integrator' */
} XDis_pitch_control_loop_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T theta_out;                    /* '<Root>/theta_out' */
} ExtY_pitch_control_loop_T;

/* Real-time Model Data Structure */
struct tag_RTM_pitch_control_loop_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_pitch_control_loop_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_pitch_control_loop_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[2];
  real_T odeF[4][2];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
extern B_pitch_control_loop_T pitch_control_loop_B;

/* Continuous states (default storage) */
extern X_pitch_control_loop_T pitch_control_loop_X;

/* Disabled states (default storage) */
extern XDis_pitch_control_loop_T pitch_control_loop_XDis;

/* Block states (default storage) */
extern DW_pitch_control_loop_T pitch_control_loop_DW;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_pitch_control_loop_T pitch_control_loop_Y;

/* Model entry point functions */
extern void pitch_control_loop_initialize(void);
extern void pitch_control_loop_step(void);
extern void pitch_control_loop_terminate(void);

/* Real-time Model object */
extern RT_MODEL_pitch_control_loop_T *const pitch_control_loop_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'pitch_control_loop'
 * '<S1>'   : 'pitch_control_loop/pid_controller_block'
 * '<S2>'   : 'pitch_control_loop/pitch_program_block'
 */
#endif                                 /* pitch_control_loop_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: libdigitalfilter.h
 *
 * Code generated for Simulink model 'libdigitalfilter'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Wed Aug 14 11:19:50 2024
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_libdigitalfilter_h_
#define RTW_HEADER_libdigitalfilter_h_
#ifndef libdigitalfilter_COMMON_INCLUDES_
#define libdigitalfilter_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* libdigitalfilter_COMMON_INCLUDES_ */

#include "libdigitalfilter_types.h"
#include "rtw_modelmap.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm)         ((rtm)->DataMapInfo)
#endif

#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val)    ((rtm)->DataMapInfo = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Input;                        /* '<Root>/Input' */
} ExtU_libdigitalfilter_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Out1;                         /* '<Root>/Out1' */
} ExtY_libdigitalfilter_T;

/* Real-time Model Data Structure */
struct tag_RTM_libdigitalfilter_T {
  const char_T * volatile errorStatus;

  /*
   * DataMapInfo:
   * The following substructure contains information regarding
   * structures generated in the model's C API.
   */
  struct {
    rtwCAPI_ModelMappingInfo mmi;
  } DataMapInfo;
};

/* External inputs (root inport signals with default storage) */
extern ExtU_libdigitalfilter_T libdigitalfilter_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_libdigitalfilter_T libdigitalfilter_Y;

/* Model entry point functions */
extern void libdigitalfilter_initialize(void);
extern void libdigitalfilter_step(void);
extern void libdigitalfilter_terminate(void);

/* Function to get C API Model Mapping Static Info */
extern const rtwCAPI_ModelMappingStaticInfo*
  libdigitalfilter_GetCAPIStaticMap(void);

/* Real-time Model object */
extern RT_MODEL_libdigitalfilter_T *const libdigitalfilter_M;

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
 * '<Root>' : 'libdigitalfilter'
 */
#endif                                 /* RTW_HEADER_libdigitalfilter_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

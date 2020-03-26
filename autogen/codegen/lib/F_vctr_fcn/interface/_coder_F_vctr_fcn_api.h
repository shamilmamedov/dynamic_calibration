/*
 * File: _coder_F_vctr_fcn_api.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 25-Mar-2020 16:46:36
 */

#ifndef _CODER_F_VCTR_FCN_API_H
#define _CODER_F_VCTR_FCN_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_F_vctr_fcn_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void F_vctr_fcn(real_T in1[6], real_T in2[18], real_T tau_frcn[6]);
extern void F_vctr_fcn_api(const mxArray * const prhs[2], int32_T nlhs, const
  mxArray *plhs[1]);
extern void F_vctr_fcn_atexit(void);
extern void F_vctr_fcn_initialize(void);
extern void F_vctr_fcn_terminate(void);
extern void F_vctr_fcn_xil_terminate(void);

#endif

/*
 * File trailer for _coder_F_vctr_fcn_api.h
 *
 * [EOF]
 */

/*
 * File: _coder_G_vctr_fcn_api.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 25-Mar-2020 16:43:47
 */

#ifndef _CODER_G_VCTR_FCN_API_H
#define _CODER_G_VCTR_FCN_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_G_vctr_fcn_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void G_vctr_fcn(real_T in1[6], real_T in2[60], real_T G_vctr_sym[6]);
extern void G_vctr_fcn_api(const mxArray * const prhs[2], int32_T nlhs, const
  mxArray *plhs[1]);
extern void G_vctr_fcn_atexit(void);
extern void G_vctr_fcn_initialize(void);
extern void G_vctr_fcn_terminate(void);
extern void G_vctr_fcn_xil_terminate(void);

#endif

/*
 * File trailer for _coder_G_vctr_fcn_api.h
 *
 * [EOF]
 */

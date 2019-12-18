/************************************************************************
*   U = mexchol(X)         X = U'*U 
*  Computes upper-triangular Cholesky factor U of full symmetric positive
*  definite matrix X. 
*
*  Uses LAPACK routine DPOTRF
************************************************************************/

#include "mex.h"
#include <math.h>

/********************************************************************
  PROCEDURE mexFunction - Entry for Matlab
*********************************************************************/
void mexFunction(const int nlhs, mxArray *plhs[],
                 const int nrhs, const mxArray *prhs[])
{
  int n; 
  double *x, *info;
  char   *chn;   
  int    infotmp=0, j,k,jn; 
  

  if(nrhs < 1)
    mexErrMsgTxt("mexchol: requires 1 input arguments.");
  if(nlhs > 1)
    mexErrMsgTxt("mexchol: requires 1 output argument.");

  x = mxGetPr(prhs[0]);
  if (mxIsSparse(prhs[0]))
     mexErrMsgTxt("Sparse X not supported by mexchol.");
  n = mxGetM(prhs[0]); 
  if ( n != mxGetN(prhs[0]) )
     mexErrMsgTxt("X should be square.");
  plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);  
  info = mxGetPr(plhs[0]); 

  chn = "U";
  dpotrf_ (chn, &n, x, &n, &infotmp); 
  info[0] = (double)infotmp; 
  /***** make lower triangular part of x zero *****/ 
  for (j=0; j<n; j++) { 
    jn = j*n; 
    for (k=j+1; k<n; k++) { x[k+jn] = 0; } 
  }
  return;
}
/************************************************************************/





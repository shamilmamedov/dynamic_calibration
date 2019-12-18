/************************************************************************
*  mexpertdiag(X,pertdiag)         
*  add pertdiag to the diagonal of X (a square matrix) 
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
  double *X, *pertdiag;
  int    j,jn; 
  

  if(nrhs != 2)
    mexErrMsgTxt("mexpertdiag: requires 2 input arguments.");
  if(nlhs > 0)
    mexErrMsgTxt("mexpertdiag: requires no output argument.");

  X = mxGetPr(prhs[0]);
  if (mxIsSparse(prhs[0]))
     mexErrMsgTxt("Sparse X not supported by mexpertdiag.");
  n = mxGetM(prhs[0]); 
  if ( n != mxGetN(prhs[0]) )
     mexErrMsgTxt("X should be square.");
  pertdiag = mxGetPr(prhs[1]); 
  if (mxIsSparse(prhs[1]))
     mexErrMsgTxt("Sparse pertdiag not supported by mexpertdiag.");
  

  for (j=0; j<n; j++) { 
    jn = j*n; 
    X[j+jn] += pertdiag[j]; 
  }
  return;
}
/************************************************************************/





/************************************************************************
*  mexschurfun(X,dd,options)         
*  options = 1, add dd to the diagonal of X (a square matrix) 
*  options = 2, add the scalar dd to X.
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
  double *X, *dd;
  int    *irX, *jcX; 
  int    isspX, j, jn, k, kstart, kend, r; 
  int    options; 
  double tmp, tmp2, ddtmp;

  if(nrhs < 2)
    mexErrMsgTxt("mexschurfun: requires at least 2 input arguments.");
  if(nlhs > 0)
    mexErrMsgTxt("mexschurfun: requires no output argument.");

  X = mxGetPr(prhs[0]);
  isspX = mxIsSparse(prhs[0]); 
  if (isspX) {
     irX = mxGetIr(prhs[0]);
     jcX = mxGetJc(prhs[0]);
  }
  n = mxGetM(prhs[0]); 
  if ( n != mxGetN(prhs[0]) )
     mexErrMsgTxt("X should be square.");
  dd = mxGetPr(prhs[1]); 
  if (mxIsSparse(prhs[1]))
     mexErrMsgTxt("Sparse dd not supported by mexschurfun.");
  if (nrhs == 2) { 
     options = 1; 
  } else {
     options = (int) (*mxGetPr(prhs[2])); 
     ddtmp = dd[0]; 
  }
  /********************************************************/
  if (options==1) { 
     if (isspX) {
        for (j=0; j<n; j++) {
           kstart = jcX[j]; kend = jcX[j+1]; 
           for (k=kstart; k<kend; k++) { 
	      r = irX[k];
              if (r==j) { X[k] += dd[j]; break; } 
	   }
	}
      } else { 
        for (j=0; j<n; j++) { jn = j*n; X[j+jn] += dd[j]; }
      }
  } else {
     if (isspX) {
        for (j=0; j<n; j++) {
           kstart = jcX[j]; kend = jcX[j+1]; 
           for (k=kstart; k<kend; k++) { 
	      r = irX[k];
              X[k] += ddtmp; 
	   }
	}
      } else { 
        for (j=0; j<n; j++) { 
           jn = j*n;
           for (k=0; k<n; k++) {  
               X[k+jn] += ddtmp;
	   }
	}
      }
  }

return;
}
/************************************************************************/

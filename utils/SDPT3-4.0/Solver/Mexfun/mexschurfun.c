/************************************************************************
*  mexschurfun(X,Y,options)         
*  options = 1, add Y to the diagonal of X (a square matrix) 
*  options = 2, add Y to X
*  options = 3, X(i,j) = X(i,j)*Y(i)*Y(j)
************************************************************************/

#include <mex.h>
#include <math.h>
#include <matrix.h>
#include "header.h"

/********************************************************************
  PROCEDURE mexFunction - Entry for Matlab
*********************************************************************/
void mexFunction(const int nlhs, mxArray *plhs[],
                 const int nrhs, const mxArray *prhs[])
{
  double   *X, *Y, *Ytmp;
  mwIndex  *irX, *jcX, *irY, *jcY; 
  mwSize    n, isspX, isspY, j, jn, k, kstart, kend, r, kstart2, kend2; 
  mwSize    options, scalarY, nzmax; 
  double    tmp, tmp2, alpha;

  if(nrhs < 2)
    mexErrMsgTxt("mexschurfun: requires at least 2 input arguments.");
  if(nlhs != 1)
    mexErrMsgTxt("mexschurfun: requires 1 output argument.");

  isspX = mxIsSparse(prhs[0]); 
  n = mxGetM(prhs[0]);   
  if (n != mxGetN(prhs[0])) {
     mexErrMsgTxt("X should be square."); }  
  if (isspX) {
     nzmax = mxGetNzmax(prhs[0]);
     plhs[0] = mxCreateSparse(n,n,nzmax,mxREAL);   
     X = mxGetPr(plhs[0]);
     irX = mxGetIr(plhs[0]);
     jcX = mxGetJc(plhs[0]);     
     memcpy(X,mxGetPr(prhs[0]),nzmax*sizeof(double)); 
     memcpy(irX,mxGetIr(prhs[0]),nzmax*sizeof(mwSize)); 
     memcpy(jcX,mxGetJc(prhs[0]),(n+1)*sizeof(mwSize));
  } else {
    plhs[0] = mxCreateDoubleMatrix(n,n,mxREAL);
    X = mxGetPr(plhs[0]);    
    memcpy(X,mxGetPr(prhs[0]),(n*n)*sizeof(double)); 
  }
  isspY = mxIsSparse(prhs[1]);
  Y = mxGetPr(prhs[1]);  
  if (isspY) {
     irY = mxGetIr(prhs[1]);
     jcY = mxGetJc(prhs[1]);
  } 
  if (nrhs == 2) { 
     if ((mxGetM(prhs[1]) == n) & (mxGetN(prhs[1]) == n)) {
       options = 2; 
     } else {
       options = 1; 
     }
  } else {
     options = (mwSize) (*mxGetPr(prhs[2])); 
  }
  if (options == 1 || options == 3) {
     if ((mxGetN(prhs[1]) != 1) & (mxGetM(prhs[1]) != 1)) {
        mexErrMsgTxt("mexschurfun: Y should be a vector."); }
  } else {
     if ((mxGetN(prhs[1]) == 1) & (mxGetM(prhs[1]) == 1)) {
	scalarY = 1; 
        alpha = Y[0]; 
     } else {
        scalarY = 0; 
     }
  }
  /********************************************************/
  Ytmp = mxCalloc(n,sizeof(double)); 
  if (options==1) { 
     if (isspY) { 
        for (k=0; k<jcY[1]; ++k) { r=irY[k]; Ytmp[r]=Y[k]; }  
     } else {
        for (k=0; k<n; ++k) { Ytmp[k]=Y[k]; }  
     }
     if (isspX) {
        for (j=0; j<n; j++) {
           kstart = jcX[j]; kend = jcX[j+1]; 
           for (k=kstart; k<kend; k++) { 
	          r = irX[k];
              if (r==j) { X[k] += Ytmp[j]; break; } 
           }
        }
      } else { 
        for (j=0; j<n; j++) { jn = j*n; X[j+jn] += Ytmp[j]; }
      }
  }
  if (options==2) {
     if (scalarY==1) {
        if (isspX) {
           for (j=0; j<n; j++) {
              kstart = jcX[j]; kend = jcX[j+1]; 
              for (k=kstart; k<kend; k++) { 
	             r = irX[k];
                 X[k] += alpha; }
           }
        } else { 
           for (j=0; j<n; j++) { 
              jn = j*n;
              for (k=0; k<n; k++) { X[k+jn] += alpha; }
           }
        }
     } else {
        if (isspX & !isspY) {
           for (j=0; j<n; j++) {
	          kstart = jcX[j]; kend = jcX[j+1]; jn = j*n; 
              for (k=kstart; k<kend; k++) { 
	             r = irX[k];
                 X[k] += Y[r+jn]; }
           }
        } else if (!isspX & !isspY) { 
           for (j=0; j<n; j++) { 
              jn = j*n;
              for (k=0; k<n; k++) { X[k+jn] += Y[k+jn]; }
           }
        } else if (!isspX & isspY) { 
           for (j=0; j<n; j++) {
	          kstart = jcY[j]; kend = jcY[j+1]; jn = j*n; 
              for (k=kstart; k<kend; k++) { 
	             r = irY[k];
                 X[r+jn] += Y[k]; }
           }
        } else {
          for (j=0; j<n; j++) {
	          kstart2 = jcY[j]; kend2 = jcY[j+1]; 
              for (k=kstart2; k<kend2; ++k) { r=irY[k]; Ytmp[r]=Y[k]; }  
	          kstart = jcX[j]; kend = jcX[j+1];  
              for (k=kstart; k<kend; k++) { 
	             r = irX[k];
                 X[k] += Ytmp[r]; }
              for (k=kstart2; k<kend2; ++k) { r=irY[k]; Ytmp[r]=0.0; }  
          }
        }
     }     
  }
  if (options==3) {
     if (isspX) { 
        for (j=0; j<n; j++) {
	       kstart = jcX[j]; kend = jcX[j+1]; jn = j*n; 
           for (k=kstart; k<kend; k++) { 
	          r = irX[k];
              X[k] *= Y[r]*Y[j]; }
        }
     } else {
        for (j=0; j<n; j++) { 
           jn = j*n;
           for (k=0; k<n; k++) { X[k+jn] *= Y[k]*Y[j]; }
        }
     }   
  }
  mxFree(Ytmp); 
return;
}
/************************************************************************/

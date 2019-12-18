/**********************************************************************
*  mextriangsp: given upper triangular U,
*  options = 1 (default), solves     U *y = b (backward substitutions)
*          = 2          , solves     U'*y = b (forward substitutions). 
*
*  y = mextriangsp(Uinput,b,options)
*
*  Important: U is assumed to be sparse. 
*  If options = 1, Uinput must be the transpose of U. 
*   
*********************************************************************/

#include <mex.h>
#include <math.h>
#include <matrix.h>

/*************************************************************
  ubsolve2: solve Ux = b for x by backward substitutions. 
  Ut: sparse, Ut = transpose(U)
**************************************************************/
void ubsolve2(int n, const double *Ut, const int *irUt, const int *jcUt, 
              double *b, double *x)

{ int j, k, kstart, kend, idx; 
  double tmp; 
  
  /************************************************
      x[j]*Ut[j,j] + x[j+1:n]*Ut[j+1:n,j] = b[j]
  ************************************************/

  x[n-1] = b[n-1]/Ut[jcUt[n]-1];
 
  for (j=n-2; j>=0; j--) {
      kstart = jcUt[j]+1; kend = jcUt[j+1]; 
      tmp = 0.0; 
      for (k=kstart; k<kend; k++) { 
	  idx = irUt[k]; 
          tmp += Ut[k]*x[idx];  	 
      }
      x[j] = (b[j]-tmp)/Ut[kstart-1]; 
  }
  return; 
}
/*************************************************************
  lbsolve2: solve U'*x = b for x by forward substitutions. 

**************************************************************/
void lbsolve2(int n, const double *U, const int *irU, const int *jcU, 
              double *b, double *x)

{ int j, k, kstart, kend, idx; 
  double tmp; 
  
  /************************************************
      x[j]*U[j,j] + x[0:j-1]*U[0:j-1,j] = b[j]
  ************************************************/
  x[0] = b[0]/U[0]; 
  for (j=1; j<n; j++) {
      kstart = jcU[j]; kend = jcU[j+1]-1; 
      tmp = 0.0; 
      for (k=kstart; k<kend; k++) { 
	  idx = irU[k]; 
          tmp += U[k]*x[idx];  	 
      }
      x[j] = (b[j]-tmp)/U[kend]; 
  }
  return; 
}
/*************************************************************
*   PROCEDURE mexFunction - Entry for Matlab
**************************************************************/
 void mexFunction(const int nlhs, mxArray *plhs[],
                  const int nrhs, const mxArray *prhs[])
{
   const double  *U;
   const int     *irb, *jcb, *irU, *jcU;    
   int           n, k, kend, isspb, isspU, options;   
   double        *x, *b, *btmp;

   if (nrhs < 2) {
      mexErrMsgTxt("mextriangsp requires 2 input arguments."); }
   if (nlhs > 1) {
      mexErrMsgTxt("mextriangsp generates 1 output argument."); }

   n = mxGetM(prhs[0]); 
   if (mxGetN(prhs[0]) != n) {
      mexErrMsgTxt("mextriangsp: U must be square and upper triangular."); }
   U = mxGetPr(prhs[0]);
   isspU = mxIsSparse(prhs[0]);   
   if (!isspU) {
      mexErrMsgTxt("mextriangsp: U must be sparse"); 
   } else {
      irU = mxGetIr(prhs[0]); 
      jcU = mxGetJc(prhs[0]); 
   }
   isspb = mxIsSparse(prhs[1]); 
   if ( mxGetM(prhs[1])*mxGetN(prhs[1]) != n ) {
       mexErrMsgTxt("mextriangsp: Size of U,b mismatch."); }
   if (nrhs > 2) { 
      options = (int)*mxGetPr(prhs[2]); }
   else {
      options = 1; 
   }
   if (isspb) {
      btmp = mxGetPr(prhs[1]);
      irb = mxGetIr(prhs[1]); jcb = mxGetJc(prhs[1]); 
      b = mxCalloc(n,sizeof(double));       
      kend = jcb[1]; 
      for (k=0; k<kend; k++) { b[irb[k]] = btmp[k]; } 
   } else {
      b = mxGetPr(prhs[1]); 
   }
   /************************************************/
   plhs[0] = mxCreateDoubleMatrix(n, 1, mxREAL);
   x = mxGetPr(plhs[0]);
  
   /************************************************/
   if (options==1) { 
      if (irU[jcU[n]-1] < n-1) { 
         mexErrMsgTxt("mextriangsp: matrix not lower triangular."); } 
      ubsolve2(n,U,irU,jcU,b,x);
   } else if (options==2) { 
      if (irU[jcU[1]-1] > 0) {
         mexErrMsgTxt("mextriangsp: matrix not upper triangular."); }
      lbsolve2(n,U,irU,jcU,b,x);
   }   
   if (isspb) { mxFree(b); }
   return;
}
/*************************************************************/

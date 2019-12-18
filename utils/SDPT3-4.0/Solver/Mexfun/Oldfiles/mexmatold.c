/***********************************************************************
* mexmat.c : C mex file 
*
*  M = mexmat(blk,x,isspM,rowidx,colidx,iscellM); 
*
* SDPT3: version 3.0
* Copyright (c) 1997 by
* K.C. Toh, M.J. Todd, R.H. Tutuncu
* Last Modified: 2 Feb 01
***********************************************************************/

#include <math.h>
#include <mex.h>

/**********************************************************
* single block 
**********************************************************/
void  mat1(int n, 
           const double *A, const int *irA, const int *jcA, int isspA,
           int mA, int colidx, 
           double *B, int *irB, int *jcB, int isspB)

{  int idx, i, j, r, jn, k, kstart, kend, idxj, j2, count;
   double tmp;  
   
   if (!isspA & !isspB) { 
      idx = colidx*mA; 
      for (j=0; j<n; j++) { 
          jn = j*n; 
          for (i=0; i<n; i++) { 
              B[i+jn] = A[idx]; 
              idx++; } 
      }
   } 
   else if (!isspA & isspB) { 
      idx = colidx*mA; 
      count = 0; 
      for (j=0; j<n; j++) { 
          for (i=0; i<n; i++) {
   	      tmp = A[idx];
              if (tmp != 0) { irB[count] = i; B[count] = tmp; count++; }
              idx++; 
          }     
          jcB[j+1] = count; 
      }   
   }
   else if (isspA & !isspB) {
      j2 = 0; idxj = 0; 
      kstart = jcA[colidx];  kend = jcA[colidx+1]; 
      for (k=kstart; k<kend; k++) { 
          r = irA[k];
          for (j=j2; j<n; j++) {i=r-idxj; if (i>=n) {idxj+=n;} else {break;}} j2=j; 
          B[i+j*n] = A[k]; 
      }
   }
   else if (isspA & isspB) {
      count = 0; 
      j2 = 0; idxj = 0; 
      kstart = jcA[colidx];  kend = jcA[colidx+1]; 
      for (k=kstart; k<kend; k++) { 
          r = irA[k];
          for (j=j2; j<n; j++) {i=r-idxj; if (i>=n) {idxj+=n;} else {break;}} j2=j; 
          irB[count] = i;
          B[count] = A[k];
          ++jcB[j+1]; 
          count++; 
      }   
      for (j=0; j<n; j++) { jcB[j+1] += jcB[j]; }
   }
return; 
}
/**********************************************************
* B is sparse
* multiple sub-blocks 
**********************************************************/
void  mat2(int n, const int numblk, const int *cumblksize, const int *blknnz, 
           const double *A,  int mA, int colidx,  
           double *B, int *irB, int *jcB, int isspB)

{  int idx, i, j, r, jn, k, kstart, kend, idxj, j2, count;
   int t, t2, istart, jstart, jend, rowidx, nsub; 
   double tmp;  

      idx = 0; 
      jstart = 0; jend = 0; jcB[0]=0;  
      for (t=0; t<numblk; t++) {   	
 	  jend = cumblksize[t+1]; 
          istart = jstart;
          idxj = colidx*mA; 
          nsub = jend-jstart; 
          for (j=jstart; j<jend; j++){
   	      idxj = (j-jstart)*nsub; 
              rowidx = blknnz[t]-istart+idxj; 
              for (i=jstart; i<jend; i++) {
  		  irB[idx] = i;  B[idx] = A[rowidx+i]; idx++; }                    
              jcB[j+1] = idx; 
          } 
          jstart = jend; 
      }
return; 
}
/**********************************************************
* 
***********************************************************/
void mexFunction(
      int nlhs,   mxArray  *plhs[], 
      int nrhs,   const mxArray  *prhs[] )

{    
     mxArray  *blk_cell_pr, *A_cell_pr;
     double   *A,  *B,  *blksize, *Atmp;
     int      *irA, *jcA, *irB, *jcB;
     int      *cumblksize, *blknnz;
     int      iscellA, mblk, mA, nA, m1, n1, rowidx, colidx, isspA, isspB;

     int   subs[2];
     int   nsubs=2; 
     int   n, n2, k, nsub, index, numblk, NZmax, r, kstart, kend;

/* CHECK FOR PROPER NUMBER OF ARGUMENTS */

     if (nrhs < 2){
         mexErrMsgTxt("mexsmat: requires at least 2 input arguments."); }
     else if (nlhs>1){ 
         mexErrMsgTxt("mexsmat: requires 1 output argument."); }

/* CHECK THE DIMENSIONS */

     iscellA = mxIsCell(prhs[1]); 
     if (iscellA) { mA = mxGetM(prhs[1]); nA = mxGetN(prhs[1]); }
     else         { mA = 1; nA = 1; }
     if (mxGetM(prhs[0]) != mA) {
         mexErrMsgTxt("mexsmat: blk and Avec not compatible"); }

/***** main body *****/ 
       
     if (nrhs > 3) {rowidx = (int)*mxGetPr(prhs[3]); } else {rowidx = 1;}  
     if (rowidx > mA) {
         mexErrMsgTxt("mexsmat: rowidx exceeds size(Avec,1)."); }
     subs[0] = rowidx-1;  /* subtract 1 to adjust for Matlab index */
     subs[1] = 1;
     index = mxCalcSingleSubscript(prhs[0],nsubs,subs); 
     blk_cell_pr = mxGetCell(prhs[0],index);
     numblk  = mxGetN(blk_cell_pr);            
     blksize = mxGetPr(blk_cell_pr);
     cumblksize = mxCalloc(numblk+1,sizeof(int)); 
     blknnz = mxCalloc(numblk+1,sizeof(int)); 
     cumblksize[0] = 0; blknnz[0] = 0; 
     n = 0;  n2 = 0; 
     for (k=0; k<numblk; ++k) {
          nsub = (int) blksize[k];
          n  += nsub; 
          n2 += nsub*nsub;  
          cumblksize[k+1] = n; 
          blknnz[k+1] = n2;
     }
     /***** assign pointers *****/
     if (iscellA) { 
         subs[0] = rowidx-1; 
         subs[1] = 0;
         index = mxCalcSingleSubscript(prhs[1],nsubs,subs); 
         A_cell_pr = mxGetCell(prhs[1],index); 
         A  = mxGetPr(A_cell_pr); 
         m1 = mxGetM(A_cell_pr); 
         n1 = mxGetN(A_cell_pr);
         isspA = mxIsSparse(A_cell_pr);
         if (isspA) { irA = mxGetIr(A_cell_pr);
                      jcA = mxGetJc(A_cell_pr); } 
     }
     else { 
         A  = mxGetPr(prhs[1]); 
         m1 = mxGetM(prhs[1]); 
         n1 = mxGetN(prhs[1]); 
         isspA = mxIsSparse(prhs[1]); 
         if (isspA) {  irA = mxGetIr(prhs[1]); 
                       jcA = mxGetJc(prhs[1]); }
     }
     if (numblk > 1) 
        { isspB = 1; }
     else { 
        if (nrhs > 2) {isspB = (int)*mxGetPr(prhs[2]);} else {isspB = isspA;} 
     }
     if (nrhs > 4) {colidx = (int)*mxGetPr(prhs[4]) -1;} else {colidx = 0;} 
     if (colidx > n1) { 
         mexErrMsgTxt("mexsmat: colidx exceeds size(Avec,2)."); 
     }    
     /***** create return argument *****/
     if (isspB) {
        if (numblk == 1 & isspA) {           
	  NZmax = jcA[colidx+1]-jcA[colidx]; }
        else {
	  NZmax = blknnz[numblk]; 
        }
        plhs[0] = mxCreateSparse(n,n,NZmax,mxREAL); 
        B = mxGetPr(plhs[0]);
        irB = mxGetIr(plhs[0]); 
        jcB = mxGetJc(plhs[0]); }
     else {
        plhs[0] = mxCreateDoubleMatrix(n,n,mxREAL); 
        B = mxGetPr(plhs[0]); 
     }
     /***** Do the computations in a subroutine *****/
     if (numblk == 1) { 
        mat1(n,A,irA,jcA,isspA,m1,colidx,B,irB,jcB,isspB);  }   
     else {
       if (isspA) { 
	  /** Atmp = mxGetPr(mxCreateDoubleMatrix(blknnz[numblk],1,mxREAL)); **/
          Atmp = mxCalloc(blknnz[numblk],sizeof(double)); 
          kstart = jcA[colidx]; kend = jcA[colidx+1]; 
          for (k=kstart; k<kend; k++) { r = irA[k]; Atmp[r] = A[k]; }       
          mat2(n,numblk,cumblksize,blknnz,Atmp,m1,0,B,irB,jcB,isspB); }
       else {
          mat2(n,numblk,cumblksize,blknnz,A,m1,colidx,B,irB,jcB,isspB); }
     }
 return;
 }
/**********************************************************/


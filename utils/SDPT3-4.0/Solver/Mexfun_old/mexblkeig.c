/***********************************************************************
* mexblkeig: find the eigenvalues of a symmetric block diagonal matrix
*  
* d = mexblkeig(blk,A); 
*
* SDPT3: version 3.0
* Copyright (c) 1997 by
* K.C. Toh, M.J. Todd, R.H. Tutuncu
* Last Modified: 2 Feb 01   
***********************************************************************/

#include <mex.h>
#include <math.h>
#include <matrix.h>
#include <string.h> /* needed for memcpy() */

#if !defined(MX_API_VER) || ( MX_API_VER < 0x07030000 )
typedef int mwIndex;
typedef int mwSize;
#endif

#if !defined(MAX)
#define  MAX(A, B)   ((A) > (B) ? (A) : (B))
#endif

#if !defined(r2)
#define  r2   1.41421356237309504880      /* sqrt(2) */
#endif

#if !defined(ir2)
#define  ir2  0.70710678118654752440      /* 1/sqrt(2) */
#endif

/**********************************************************
* 
***********************************************************/
void mexFunction(int nlhs, mxArray  *plhs[], 
                 int nrhs, const mxArray  *prhs[] )

{    mxArray  *blk_cell_pr;
     mxArray  *tmparr[1];
     double   *A, *blksizetmp, *P, *V, *D, *eigP; 
     double   *work, *work2; 
     mwIndex  *irA, *jcA, *irV, *jcV, *irD, *jcD;
     int      *blksize, *cumblksize, *cumblksize2;
     int       isspA;
     char     *jobz="V";
     char     *uplo="U"; 

     mwIndex   subs[2];
     mwSize    nsubs=2; 
     mwSize    lwork, lwork2, info;
     int    m, n, n2, nsub, k, index, numblk, len, maxblksize; 
     int    i, j, l, jn, idxstart, idxend, kstart, kend, idx, count; 
     int    blklen, blklen2, rowidx, colidx, kn; 

/* CHECK FOR PROPER NUMBER OF ARGUMENTS */

   if (nrhs < 2){
      mexErrMsgTxt("mexblkeig: requires 2 input arguments."); }
   if (nlhs > 2){
      mexErrMsgTxt("mexblkeig: requires at 2 output arguments."); }
   
/* CHECK THE DIMENSIONS */

    if (mxGetM(prhs[0]) > 1) { 
       mexErrMsgTxt("mexblkeig: blk can only have 1 row."); }
    if (mxGetN(prhs[0]) < 2) { 
       mexErrMsgTxt("mexblkeig: blk must have at least 2 columns."); }
    subs[0] = 0; subs[1] = 1;
    index = mxCalcSingleSubscript(prhs[0],nsubs,subs); 
    blk_cell_pr = mxGetCell(prhs[0],index);
    numblk     = mxGetN(blk_cell_pr);
    blksizetmp = mxGetPr(blk_cell_pr); 
    blksize    = mxCalloc(numblk,sizeof(int));
    for (k=0; k<numblk; k++) { 
        blksize[k] = (int) blksizetmp[k]; }
    cumblksize  = mxCalloc(numblk+1,sizeof(int));
    cumblksize2 = mxCalloc(numblk+1,sizeof(int)); 
    cumblksize[0] = 0; cumblksize2[0] = 0;  
    maxblksize=0; 
    for (k=0; k<numblk; ++k) {
        nsub = blksize[k];
        n2   = nsub*nsub;
        cumblksize[k+1]  = cumblksize[k] + nsub;                 
        cumblksize2[k+1] = cumblksize2[k] + n2; 
        maxblksize = MAX(maxblksize,nsub);  
    }
    /***** assign pointers *****/
    m = mxGetM(prhs[1]); 
    n = mxGetN(prhs[1]); 
    if (m != n) { 
       mexErrMsgTxt("mexblkeig: matrix A must be square."); }
    A = mxGetPr(prhs[1]); 
    isspA = mxIsSparse(prhs[1]); 
    if (isspA) { irA = mxGetIr(prhs[1]); 
                 jcA = mxGetJc(prhs[1]); }  

    /***** create temporary array *****/
    tmparr[0] = mxCreateDoubleMatrix(maxblksize,maxblksize,mxREAL);
    P = mxGetPr(tmparr[0]);
    eigP = mxCalloc(maxblksize,sizeof(double)); 
    lwork  = 1+6*maxblksize+2*maxblksize*maxblksize;  
    work   = mxCalloc(lwork,sizeof(double)); 
    lwork2 = 3 + 5*maxblksize; 
    work2  = mxCalloc(lwork2,sizeof(double)); 

    /***** create return argument *****/
    if (nlhs==2) {        
       plhs[0] = mxCreateSparse(n,n,1+cumblksize2[numblk],mxREAL); 
       irV = mxGetIr(plhs[0]); 
       jcV = mxGetJc(plhs[0]); 
       V   = mxGetPr(plhs[0]);
       plhs[1] = mxCreateSparse(n,n,n+1,mxREAL);  
       irD = mxGetIr(plhs[1]); 
       jcD = mxGetJc(plhs[1]); 
       D   = mxGetPr(plhs[1]); 
       jobz="V"; 
       jcD[0] = 0; jcV[0]=0; 
    } else { 
       plhs[0] = mxCreateDoubleMatrix(n,1,mxREAL);  
       D = mxGetPr(plhs[0]); 
       jobz="N"; 
    }
    /***** Do the computations in a subroutine *****/
    for (l=0; l<numblk; l++) {
        blklen = blksize[l];
        /*----- copy lth block to P -----*/ 
        for (j=0; j<blklen; j++) {
	    jn = j*maxblksize; 
	    for (k=0; k<blklen; k++) {
	        idx = k+jn; 
                P[idx] = 0; }	    
        }
        idxstart = cumblksize[l]; 
        idxend   = cumblksize[l+1]; 
        if (isspA) {
           for (j=idxstart; j<idxend; j++) {
	       kstart = jcA[j]; kend = jcA[j+1];
               jn = (j-idxstart)*maxblksize; 
               for (k=kstart; k<kend; k++) {
	           idx = irA[k]-idxstart; 
	           P[idx+jn] = A[k]; }	
	   }
	} else {
           for (j=idxstart; j<idxend; j++) {
	       colidx = j*n; 
               jn = (j-idxstart)*maxblksize; 
               for (k=idxstart; k<idxend; k++) {
		   idx = (k-idxstart) + jn;
		   P[idx] = A[k+colidx]; 
               }
	   }
	}
        /*----- eig(P) -----*/ 
        dsyevd_(jobz,uplo,&blklen, P,&maxblksize, eigP, work,&lwork, work2,&lwork2, &info); 

        /*----- put result in V,D -----*/
        if (nlhs == 2) { 
  	   idx   = cumblksize[l]; 
           count = cumblksize2[l];
           for (k=0; k<blklen; k++) { 
              kn = maxblksize*k; 
	      for (i=0; i<blklen; i++){
                 V[count]   = P[kn+i]; 
	  	 irV[count] = idx+i;
                 count++;  
	      }
              jcV[idx+k+1] = jcV[idx+k] + blklen; 
              D[idx+k]   = eigP[k]; 
              irD[idx+k] = idx+k; 
              jcD[idx+k+1] = jcD[idx+k]+1;
	   }
	} else { 
           idx = cumblksize[l]; 
           for (k=0; k<blklen; k++) { D[idx+k] = eigP[k]; }
	} 
    }
    /***** free memory *************************/  
    mxFree(blksize);    mxFree(cumblksize2); 
    mxFree(cumblksize); 
    mxDestroyArray(*tmparr);   
return;
}
/**********************************************************/


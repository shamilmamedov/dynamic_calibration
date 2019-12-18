/***********************************************************************
* mexqprod.c : C mex file 
*
*   jj = mexqprod(blk); 
* 
*   Input: blk   = [n1, n2, ..., nk]
*
*   Output: [1 1...1  2 2 ... 2 .... k k ...k]'    
*            n1       n2             nk
*
* SDPT3: version 3.0
* Copyright (c) 1997 by
* K.C. Toh, M.J. Todd, R.H. Tutuncu
* Last Modified: 2 Feb 01   
***********************************************************************/

#include <math.h>
#include <mex.h>

/**********************************************************
* 
***********************************************************/
void mexFunction(
      int nlhs,   mxArray  *plhs[], 
      int nrhs,   const mxArray  *prhs[] )

{    double   *blksize, *jj; 

     int k, l, blkdim, numblk, cols, idx;

/* CHECK FOR PROPER NUMBER OF ARGUMENTS */

   if (nrhs !=1){
      mexErrMsgTxt("mexqprod: requires 1 input arguments."); }
   if (nlhs > 1){ 
      mexErrMsgTxt("mexqprod: requires 1 output argument."); }

/* CHECK THE DIMENSIONS */

    if (mxIsCell(prhs[0])) {
        mexErrMsgTxt("mexqprod: blk must be a double array."); }
    numblk = mxGetN(prhs[0]); 
    blksize = mxGetPr(prhs[0]);
    cols = 0; 
    for (k=0; k<numblk; k++) { 
        cols = cols + (int)blksize[k]; 
    } 
    plhs[0] = mxCreateDoubleMatrix(cols,1,mxREAL); 
    jj = mxGetPr(plhs[0]);    
    idx = 0; 
    for (k=1; k<=numblk; k++) { 
       blkdim = (int)blksize[k-1]; 
       for (l=0; l<blkdim; l++) { jj[idx] = k; idx++; }
    }
    return;
 }
/**********************************************************/


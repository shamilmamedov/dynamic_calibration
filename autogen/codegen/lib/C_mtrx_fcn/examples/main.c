/*
 * File: main.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 25-Mar-2020 16:41:34
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include Files */
#include "C_mtrx_fcn.h"
#include "main.h"
#include "C_mtrx_fcn_terminate.h"
#include "C_mtrx_fcn_initialize.h"

/* Function Declarations */
static void argInit_60x1_real_T(double result[60]);
static void argInit_6x1_real_T(double result[6]);
static double argInit_real_T(void);
static void main_C_mtrx_fcn(void);

/* Function Definitions */

/*
 * Arguments    : double result[60]
 * Return Type  : void
 */
static void argInit_60x1_real_T(double result[60])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 60; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[6]
 * Return Type  : void
 */
static void argInit_6x1_real_T(double result[6])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 6; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_C_mtrx_fcn(void)
{
  double in1_tmp[6];
  double dv0[60];
  double C_mtrx_sym[36];

  /* Initialize function 'C_mtrx_fcn' input arguments. */
  /* Initialize function input argument 'in1'. */
  argInit_6x1_real_T(in1_tmp);

  /* Initialize function input argument 'in2'. */
  /* Initialize function input argument 'in3'. */
  /* Call the entry-point 'C_mtrx_fcn'. */
  argInit_60x1_real_T(dv0);
  C_mtrx_fcn(in1_tmp, in1_tmp, dv0, C_mtrx_sym);
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* Initialize the application.
     You do not need to do this more than one time. */
  C_mtrx_fcn_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_C_mtrx_fcn();

  /* Terminate the application.
     You do not need to do this more than one time. */
  C_mtrx_fcn_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */

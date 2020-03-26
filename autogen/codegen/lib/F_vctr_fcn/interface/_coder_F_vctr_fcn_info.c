/*
 * File: _coder_F_vctr_fcn_info.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 25-Mar-2020 16:46:36
 */

/* Include Files */
#include "_coder_F_vctr_fcn_info.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : const mxArray *
 */
const mxArray *emlrtMexFcnResolvedFunctionsInfo(void)
{
  const mxArray *nameCaptureInfo;
  const char * data[8] = {
    "789ced584d6f1241181e4c6d6a8cca49132f5e7a31354c0b4decc5a44059a1a19f346d6d559c5d86b2303bb39ddda5c58bfd09fe007f80476f7aacfe022f7ae8"
    "cd1fe1d5b80b2cb01b27a05bb7a1dd3721c39b6799e7e5d997678717c40a2b3100c06dfbf5e30d005fa7413b6e751610efaed78037fc78acbb4efa7237ae8309",
    "cfe7625dbe77dd5c61d4c4c76627212ac5ab9626636e271469b8b74d85692a45d4dc6ae918706c30d2c49536525509de52355c6403495eb5134d1a807a890339"
    "efb335ac344a960678cde8974b0613d0d5c7890f82ef3f31a23eab027de23e7c3ff702d69886a151439a4ae012361a26d3e111e30da87356c78a69c04acb5647",
    "55ca0a22aacc91a9320a9165b2034ca1546e2a262f57159ad0faf5bf12d4e7deb761f5fb57376e80a9816c717118dfa87a4df872d0bbae8318ea0175d6f3e213"
    "f56fbc8b54982513dce77b1f902f23e4f3e2fb85e2aedd1296c12161f6ed862be9ad623a033793b3730b32341923323b865823d06e05a8219320196252b52874",
    "34727ac08910fbe0ded9e7efe970fb2eec3ebf38be63c17ea3f6dd5d015fdc8797966c97adcfd553b9bc7684aad2e64ed67a96ebd7b13e8467581d409087b57f"
    "f4fbf586b7dfa67b3e7e22d86f549d1e08f8e23e5c6115cc13aa7d18e01491846a642c9598056a1f073057950bf3f94f01f936857c5efc2ffac479cdb4f58233",
    "ae60d02f58f87d73f2f1e7d9b7c8f7ff135f58bebfd63ccca6abf9e6c2d6c67c4bc9693429ed917ce4fb57c3f793e7e6fbf7057c711feef37dc3960af1847bba"
    "0f7ebef787a81e3742ffbf17d0f7673a8af57a26bc7e599cfcf525f2fb71f7fbc3ed8df95d8e09d532a914e7cbb30493c752e4f757cdefdf0af61b55a78702be",
    "b80ff7f93dd275d22ab52d4cb2a8e20cb20a749d20c51dfdb9f5e901ebbb33a43e17af76ab28d710add87f045cfed380fccf87f0bb78d0e78158d04e8785d75f"
    "a7f2cd7a34071af7e70343c5ba8cd0d36c25b55037769378654f6f2d5d9ee7c3a9e0f3d1efda1bde3e7c746e73fe295f0e7ad77510d53074c40d3cae73febc90",
    "cf8bffdbf9c1d1c63941b82a25b410cfff534f5e46fe3eeefe7eb4533024d2d89692555e22cdfada327dbd918dfcfd6afb7be2d2cd81fed5ffa339d09feb8fe6"
    "40e1f04573a060fbff0661ba3e2a", "" };

  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(data, 9448U, &nameCaptureInfo);
  return nameCaptureInfo;
}

/*
 * Arguments    : void
 * Return Type  : mxArray *
 */
mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xResult;
  mxArray *xEntryPoints;
  const char * fldNames[6] = { "Name", "NumberOfInputs", "NumberOfOutputs",
    "ConstantInputs", "FullPath", "TimeStamp" };

  mxArray *xInputs;
  const char * b_fldNames[4] = { "Version", "ResolvedFunctions", "EntryPoints",
    "CoverageInfo" };

  xEntryPoints = emlrtCreateStructMatrix(1, 1, 6, fldNames);
  xInputs = emlrtCreateLogicalMatrix(1, 2);
  emlrtSetField(xEntryPoints, 0, "Name", emlrtMxCreateString("F_vctr_fcn"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs", emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs", emlrtMxCreateDoubleScalar
                (1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "FullPath", emlrtMxCreateString(
    "/home/shamil/Desktop/work/projects/dynamic_calibration/autogen/F_vctr_fcn.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp", emlrtMxCreateDoubleScalar
                (737875.62275462958));
  xResult = emlrtCreateStructMatrix(1, 1, 4, b_fldNames);
  emlrtSetField(xResult, 0, "Version", emlrtMxCreateString(
    "9.5.0.944444 (R2018b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions", (mxArray *)
                emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/*
 * File trailer for _coder_F_vctr_fcn_info.c
 *
 * [EOF]
 */

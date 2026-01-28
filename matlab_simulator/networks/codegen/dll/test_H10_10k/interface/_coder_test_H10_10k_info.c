/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_test_H10_10k_info.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Jan-2026 10:52:33
 */

/* Include Files */
#include "_coder_test_H10_10k_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[4] = {
      "789ce5534b4ec330147450416c0a5d718b4242d8c0922008484485362b8c423e2eb112db"
      "55ecd0c2921d121217e1201c82c340be4d2345a990280b66331e8dfd"
      "e6f9c906d2f9a50400d80219e48d8cbbb9eee5bc061651f7a5062eb00e3a0be70aff2d67"
      "975181662213d426a83ce93182a94dc5e871824084380b1f90973a63",
      "1ca2112668581546a2c869c52a4562256bcd476e308c09887c3eef30ac8a721e770df7ed"
      "b4cca38efa3ceafbfe4bdec70ff38afae396bcc2bf316fb523687214"
      "71a81eaa07323c616e4c10151c9e61a1c70ec4b49fbfbabe137a2e24b6086dc7e298c4a1"
      "2d580429125316051c526f62518639b27c45b61439d01559dffddedf",
      "3ec7ed25efd5f46fba6033e53de933b55695f7fefafcb2cabc027f95376ba8b7ecbbdc69"
      "c8ebd5fc6385059e7c6d90a7fb8bfd81a97ae44a18dabc8f414b4e5b"
      "1fa041ff76fd2f1dc15f43",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 1608U, &nameCaptureInfo);
  return nameCaptureInfo;
}

/*
 * Arguments    : void
 * Return Type  : mxArray *
 */
mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9] = {"Version",
                                    "ResolvedFunctions",
                                    "Checksum",
                                    "EntryPoints",
                                    "CoverageInfo",
                                    "IsPolymorphic",
                                    "PropertyList",
                                    "UUID",
                                    "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8] = {
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 1);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("test_H10_10k"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "ResolvedFilePath",
      emlrtMxCreateString("C:\\Users\\39340\\Documents\\GitHub\\in-context-"
                          "bldc\\matlab_simulator\\networks\\test_H10_10k.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(740010.45223379624));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.2.0.2923080 (R2024b) Update 6"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("B1okd0RNmzgJ2PU3dmQtNC"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/*
 * File trailer for _coder_test_H10_10k_info.c
 *
 * [EOF]
 */

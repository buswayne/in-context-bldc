#ifndef __customcode_cgcsrtSpE57vLKeNlFdwME_h__
#define __customcode_cgcsrtSpE57vLKeNlFdwME_h__

/* Include files */
#include "mex.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "tmwtypes.h"


/* Helper definitions for DLL support */
#if defined _WIN32 
  #define DLL_EXPORT_CC __declspec(dllexport)
#else
  #if __GNUC__ >= 4
    #define DLL_EXPORT_CC __attribute__ ((visibility ("default")))
  #else
    #define DLL_EXPORT_CC
  #endif
#endif
/* Custom Code from Simulation Target dialog */
#include "net_predict_L_40k_initialize.h"
#include "net_predict_L_40k_data.h"
#include "rt_nonfinite.h"
#include "main.h"
#include "net_predict_L_40k.h"
#include "net_predict_L_40k_terminate.h"
#include "predict.h"


/* Function Declarations */
#ifdef __cplusplus
extern "C" {
#endif
#define customcode_cgcsrtSpE57vLKeNlFdwME_initializer()

#define customcode_cgcsrtSpE57vLKeNlFdwME_terminator()
#ifdef __cplusplus
}
#endif

#endif


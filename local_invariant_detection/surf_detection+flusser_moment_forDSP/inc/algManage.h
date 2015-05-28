// Copyright [2015] <Phil Hu>
// algorithm management functions that follows the DSP transplantation standard.

#ifndef ALG_FUNCTION_H
#define ALG_FUNCTION_H

#include "../inc/algBase.h"


int16 alg_create(int16 w, int16 h);

int32 alg_control();

int32 alg_process(uint8* imgPtr, int16 w, int16 h);

int32 alg_delete();



#endif  // ALG_FUNCTION_H

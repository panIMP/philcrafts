#ifndef _ALG_POINT_FEAT_EXTRACT_H_
#define _ALG_POINT_FEAT_EXTRACT_H_


#include "imgMath.h"
#include "error.h"
#include "pointLocate.h"

typedef struct _ALG_PFE_PARAM_
{
	int detHesThresh;
	short r;
	PRE_PROC_MODE preProcMode;

}algPfeParam;

int alg_create(short w, short h);

int alg_control();

int alg_process(unsigned char* imgPtr, int imgSeq, short w, short h);

int alg_delete();


#endif

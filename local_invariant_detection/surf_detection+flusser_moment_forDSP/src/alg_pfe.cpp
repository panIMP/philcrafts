#include "../inc/alg_pfe.h"

/*
global variables
*/

algPfeParam g_algPfePar;

unsigned char* g_t_imgPtr;
unsigned char* g_markImgPtr;
Contour* g_contourPtr;

int* g_integImgPtr = NULL;
hesMat* g_detHesImgPtr = NULL;

InterestPoint* g_pointPtrL = NULL;
InterestPoint* g_pointPtrR = NULL;

Coord* g_coordPtr = NULL;

double* g_featMaxPtr = NULL;
double* g_featMinPtr = NULL;
double* g_featGapPtr = NULL;

unsigned char* g_imgStitchPtr = NULL;

int alg_create(short w, short h)
{
	int size = w * h;
	g_t_imgPtr = (unsigned char*)malloc_check(sizeof(unsigned char) * size);
	g_markImgPtr = (unsigned char*)malloc_check(sizeof(unsigned char) * size);
	g_contourPtr = (Contour*)malloc_check(sizeof(Contour));
	g_contourPtr->coordPtr = (Coord*)malloc_check(sizeof(Coord) * size);

	g_integImgPtr = (int*)malloc_check(sizeof(int) * size);
	g_detHesImgPtr = (hesMat*)malloc_check(sizeof(hesMat) * size);

	g_pointPtrL = (InterestPoint*)malloc_check(sizeof(InterestPoint) * size);
	g_pointPtrR = (InterestPoint*)malloc_check(sizeof(InterestPoint) * size);

	g_coordPtr = (Coord*)malloc_check(sizeof(Coord) * size);

	g_featMaxPtr = (double*)calloc_check(FEAT_NUM, sizeof(double));
	g_featMinPtr = (double*)calloc_check(FEAT_NUM, sizeof(double));
	g_featGapPtr = (double*)calloc_check(FEAT_NUM, sizeof(double));

	g_imgStitchPtr = (unsigned char*)malloc_check(sizeof(unsigned char) * size * 2);

	return 0;
}


int alg_control()
{
	g_algPfePar.detHesThresh = 200;
	g_algPfePar.r = 12;
	g_algPfePar.preProcMode = LOCAL_EQUHIST;

	return 0;
}


int alg_process(unsigned char* imgPtr, int imgSeq, short w, short h)
{
	int pointNumL = 0;
	int pointNumR = 0;

	preProcess(imgPtr, g_algPfePar.preProcMode, w, h);

	createIntegImg(imgPtr, g_integImgPtr, w, h);

	createDetHesImgPyr(g_detHesImgPtr, g_integImgPtr, g_markImgPtr, LAYER_NUM, w, h);

	if (imgSeq % 2 == 1)
	{
		pointNumL = getPointLocation(g_pointPtrL, g_markImgPtr, imgPtr, 
			g_detHesImgPtr, LAYER_NUM, g_algPfePar.detHesThresh, w, h);

		getPointsFeats(g_pointPtrL, pointNumL, imgPtr, g_coordPtr, g_algPfePar.r, w);

		drawRect(imgPtr, g_pointPtrL, pointNumL, 255, 2, w);
	}
	else
	{
		pointNumR = getPointLocation(g_pointPtrR, g_markImgPtr, imgPtr, 
			g_detHesImgPtr, LAYER_NUM, g_algPfePar.detHesThresh, w, h);

		getPointsFeats(g_pointPtrR, pointNumR, imgPtr, g_coordPtr, g_algPfePar.r, w);

		drawRect(imgPtr, g_pointPtrR, pointNumR, 255, 2, w);
	}


	return 0;
}


int alg_delete()
{
	free(g_t_imgPtr);
	g_t_imgPtr = NULL;

	free(g_markImgPtr);
	g_markImgPtr = NULL;

	free(g_contourPtr);
	g_contourPtr = NULL;

	free(g_integImgPtr);
	g_integImgPtr = NULL;

	free(g_detHesImgPtr);
	g_detHesImgPtr = NULL;

	free(g_pointPtrL);
	g_pointPtrL = NULL;

	free(g_pointPtrR);
	g_pointPtrR = NULL;

	free(g_coordPtr);
	g_coordPtr = NULL;

	free(g_featMaxPtr);
	g_featMaxPtr = NULL;
		
	free(g_featMinPtr);
	g_featMinPtr = NULL;

	free(g_featGapPtr);
	g_featGapPtr = NULL;

	free(g_imgStitchPtr);
	g_integImgPtr = NULL;
	

	return 0;
}



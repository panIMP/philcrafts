// Copyright [2015] <Phil Hu>

#include "../inc/algBase.h"
#include "../inc/algFunction.h"

/*-------------------------- global variables --------------------------------*/
// algorithm parameters
typedef struct _ALG_PARAM_ {
    int32 respThresh;
    PRE_PROC_MODE mode;
}algParam;
algParam g_algPar;

// left and right image
unsigned char* g_lImgPtr = NULL;
unsigned char* g_rImgPtr = NULL;

// temple image
unsigned char* g_tImgPtr = NULL;

// mark image
unsigned char* g_mImgPtr = NULL;

// pointer to contours of the image
Contour* g_contPtr = NULL;

// integrate image
int32* g_intImgPtr = NULL;

// response image
Response* g_respImgPtr = NULL;

// interest point set of left and right image
InterestPoint* g_lPointPtr = NULL;
InterestPoint* g_rPointPtr = NULL;

// pointer to interest point neighbor pixel coordinates
Coord* g_coordPtr = NULL;

// pointer to moment boundary of interest point descriptor
double* g_moMaxPtr = NULL;
double* g_moMinPtr = NULL;
double* g_moGapPtr = NULL;

// pointer to interest point pairs
PointPair* g_pointPairPtr = NULL;

// image of two matched image with corresponding pixels pointing to each other
uint8* g_matchImgPtr = NULL;

/*------------------------------- functions ----------------------------------*/
int16 alg_create(int16 w, int16 h) {
    int32 size = w * h;

    g_lImgPtr = reinterpret_cast<uint8*>(malloc_check(sizeof(uint8) * size));
    g_rImgPtr = reinterpret_cast<uint8*>(malloc_check(sizeof(uint8) * size));

    g_tImgPtr = reinterpret_cast<uint8*>(malloc_check(sizeof(uint8) * size));
    g_mImgPtr = reinterpret_cast<uint8*>(malloc_check(sizeof(uint8) * size));

    g_contPtr = reinterpret_cast<Contour*>(malloc_check(sizeof(Contour)));
    g_contPtr->coordPtr = reinterpret_cast<Coord*>(malloc_check(sizeof(Coord) *
        size));

    g_intImgPtr = reinterpret_cast<int32*>(malloc_check(sizeof(int) *
        size));
    g_respImgPtr = reinterpret_cast<Response*>(malloc_check(sizeof(Response) *
        size));

    g_lPointPtr = reinterpret_cast<InterestPoint*>(malloc_check(
        sizeof(InterestPoint) * size));
    g_rPointPtr = reinterpret_cast<InterestPoint*>(malloc_check(
        sizeof(InterestPoint) * size));

    g_coordPtr = reinterpret_cast<Coord*>(malloc_check(sizeof(Coord) * size));

    g_moMaxPtr = reinterpret_cast<double*>(malloc_check(sizeof(double) * 4));
    g_moMinPtr = reinterpret_cast<double*>(malloc_check(sizeof(double) * 4));
    g_moGapPtr = reinterpret_cast<double*>(malloc_check(sizeof(double) * 4));

    g_pointPairPtr = reinterpret_cast<PointPair*>(malloc_check(sizeof(PointPair)
        * size));

    g_matchImgPtr = reinterpret_cast<uint8*>(malloc_check(sizeof(unsigned char)
        * size * 2));

    return 0;
}


int32 alg_control() {
    g_algPar.respThresh = 200;
    g_algPar.mode = LOCAL_EQUHIST;

    return 0;
}


int32 alg_process(uint8* imgPtr, int16 w, int16 h) {
    int16 pointNumL = 0;
    int16 pointNumR = 0;
    int32 pairNum = 0;
    int32 size = w * h;
    static int16 imgSeq = 0;

    imgSeq++;

    if (imgSeq % 2 == 1) {
        // process the first image
        memcpy(g_lImgPtr, imgPtr, size);

        preProcess(g_lImgPtr, g_tImgPtr, g_mImgPtr, g_contPtr, g_algPar.mode, w, h);

        /*createIntegImg(g_lImgPtr, g_intImgPtr, w, h);

        createRespImgPyr(g_respImgPtr, g_intImgPtr, g_mImgPtr, 5, w, h);

        pointNumL = getPointLocation(g_lPointPtr, g_mImgPtr, g_lImgPtr,
            g_respImgPtr, 5, g_algPar.respThresh, w, h);

        getPointDescriptor(g_lPointPtr, pointNumL, g_lImgPtr, g_coordPtr, w);

        drawRect(g_lImgPtr, g_lPointPtr, pointNumL, 2, w);*/

        memcpy(imgPtr, g_lImgPtr, size);
    }
    else {
        // process the second image
        memcpy(g_rImgPtr, imgPtr, size);

        preProcess(g_rImgPtr, g_tImgPtr, g_mImgPtr, g_contPtr, g_algPar.mode, w, h);

        createIntegImg(g_rImgPtr, g_intImgPtr, w, h);

        createRespImgPyr(g_respImgPtr, g_intImgPtr, g_mImgPtr, 5, w, h);

        pointNumR = getPointLocation(g_rPointPtr, g_mImgPtr, g_rImgPtr,
            g_respImgPtr, 5, g_algPar.respThresh, w, h);

        getPointDescriptor(g_rPointPtr, pointNumR, imgPtr, g_coordPtr, w);

        drawRect(imgPtr, g_rPointPtr, pointNumR, 2, w);

        matchInterestPoints(g_lPointPtr, pointNumL, g_rPointPtr, pointNumR,
            g_pointPairPtr, &pairNum, g_moMinPtr, g_moMaxPtr, g_moGapPtr, w, h);
    }

    return 0;
}


int alg_delete() {
    free(g_lImgPtr);
    g_lImgPtr = NULL;
    free(g_rImgPtr);
    g_rImgPtr = NULL;

    free(g_tImgPtr);
    g_tImgPtr = NULL;

    free(g_mImgPtr);
    g_mImgPtr = NULL;

    free(g_contPtr);
    g_contPtr = NULL;

    free(g_intImgPtr);
    g_intImgPtr = NULL;

    free(g_respImgPtr);
    g_respImgPtr = NULL;

    free(g_lPointPtr);
    g_lPointPtr = NULL;

    free(g_rPointPtr);
    g_rPointPtr = NULL;

    free(g_coordPtr);
    g_coordPtr = NULL;

    free(g_moMaxPtr);
    g_moMaxPtr = NULL;
    free(g_moMinPtr);
    g_moMinPtr = NULL;
    free(g_moGapPtr);
    g_moGapPtr = NULL;

    free(g_pointPairPtr);
    g_pointPairPtr = NULL;

    free(g_matchImgPtr);
    g_matchImgPtr = NULL;

    return 0;
}



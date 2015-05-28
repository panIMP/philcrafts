// Copyright [2015] <Phil Hu>
// main algorithm specific functions

#ifndef ALG_FUNCTION_H
#define ALG_FUNCTION_H

#include "../inc/algParam.h"
#include "../inc/algPreproc.h"

// number of layer is 5
typedef struct RESPONSE {
    int32 val[RESPONSE_LAYER_NUM];
} Response;

typedef struct DESCRIPTOR {
    double mo[4];
} Descriptor;

typedef struct INTEREST_POINT {
    Coord c;
    Descriptor d;
    int16 r;
} InterestPoint;

typedef struct POINT_PAIR {
    Coord pL;
    Coord pR;
} PointPair;

/*The 6 unknown coefficients for projection matrix*/
/*
|   m1  m2  m3  |       |   x   |       |   x^  |
|               |       |       |       |       |
|   m4  m5  m6  |   *   |   y   |   =   |   y^  |
|               |       |       |       |       |
|   0   0   1   |       |   1   |       |   1   |
*/
typedef struct AFFINE_MATRIX {
    double m1;      double m2;      double m3;
    double m4;      double m5;      double m6;
    double m7 = 0;  double m8 = 0;  double m9 = 1;
} AffineMat;

int32 createRespImgPyr(
    Response* respImgPtr,
    const int32* intImgPtr,
    const uint8* mImgPtr,
    int16 layNum,
    int16 w,
    int16 h);

int32 getPointLocation(
    InterestPoint* pointPtr,
    uint8* marImgPtr,
    const uint8* imgPtr,
    const Response* respImgPtr,
    int16 layNum,
    double respThresh,
    int16 w,
    int16 h);

// draw the rectangle around the interest point
int32 drawRect(
    uint8* imgPtr,
    const InterestPoint* pointPtr,
    int32 pointNum,
    int16 r,
    int16 w);

int32 getPointDescriptor(
    InterestPoint* pointPtr,
    int32 pointNum,
    const uint8* imgPtr,
    Coord* coordPtr,
    int16 w);    

AffineMat matchInterestPoints(
    InterestPoint* lPointsPtr,
    int32 lPointNum,
    InterestPoint* rPointsPtr,
    int32 rPointNum,
    PointPair* pairPtr,
    int32* pairNumRef,
    double* moMinPtr,
    double* moMaxPtr,
    double* moGapPtr,
    int16 w,
    int16 h);

#endif  // ALG_FUNCTION_H

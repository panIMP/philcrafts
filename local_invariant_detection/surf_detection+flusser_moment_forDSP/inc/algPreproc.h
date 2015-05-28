// Copyright [2015] <Phil Hu>
// shared preprocessing functions that can aid different algorithms

#ifndef ALG_PREPROC_H
#define ALG_PREPROC_H

#include "../inc/algBase.h"

typedef struct COORD {
    int16 x;
    int16 y;
} Coord;

typedef struct CONTOUR {
    Coord* coordPtr;
    int32 num[1000];
} Contour;

typedef struct LINE {
    int16 dist;
    int16 sitaAngle;
} Line;

enum PIXEL_MARK_TYPE {
    BACK_GROUND_PIXEL = 0,
    DETECT_MARK1,
    DETECT_MARK2,
    DETECT_MARK3,
    DETECT_MARK4,
    FORE_GROUND_PIXEL = 255,
};

enum PRE_PROC_MODE {
    ONLY_GLOBAL_GUASSIN = 0,
    GLOBAL_EUQHIST,
    GET_PREGROUND,
    LOCAL_EQUHIST,
};

void drawLine(
    uint8* imgPtr,
    int16 w,
    int16 h,
    Line line);

// calculate the neighbor pixel coordinates
int32 calcDiskTmplArray(
    Coord* coordPtr,
    int16 r);

void stitchImg(
    const uint8* lImgPtr,
    const uint8* rImgPtr,
    uint8* stitchImgPtr,
    int16 w,
    int16 h);

// create integral image of one image
int32 createIntegImg(
    const uint8 *imgPtr,
    int32* intImgPtr,
    int16 w,
    int16 h);

int32 preProcess(
    uint8* imgPtr,
    uint8* tImgPtr,
    uint8* mImgPtr,
    Contour* contourPtr,
    PRE_PROC_MODE mode,
    int16 w,
    int16 h);

#endif  // ALG_PREPROC_H

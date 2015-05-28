// Copyright [2015] <Phil Hu>

#include "../inc/algPreproc.h"

extern int16 sinValue[360];
extern int16 cosValue[360];

void drawLine(
    uint8* imgPtr,
    int16 w,
    int16 h,
    Line line) {
    int16 i, j;

    for (i = 0; i < w; ++i) {
        for (j = 0; j < h; ++j) {
            if (((i *cosValue[line.sitaAngle] + j * sinValue[line.sitaAngle]
                + 512) >> 10) == line.dist) {
                *(imgPtr + j*w + i) = 255;
            }
        }
    }
}

// calculate the neighbor pixel coordinates
int32 calcDiskTmplArray(
    Coord* coordPtr,
    int16 r) {
    int32 count = 0;
    int32 r2 = r * r;

    int16 x = 0;
    int16 y = 0;

    if (coordPtr == NULL)
        return -1;

    for (x = -r; x <= r; ++x) {
        for (y = -r; y <= r; ++y) {
            if ((x * x + y * y) <= r2) {
                coordPtr->x = x;
                coordPtr->y = y;

                ++coordPtr;
                ++count;
            }
        }
    }

    return count;
}

void stitchImg(
    const uint8* lImgPtr,
    const uint8* rImgPtr,
    uint8* stitchImgPtr,
    int16 w,
    int16 h) {
    int16 i;

    if (lImgPtr == NULL || rImgPtr == NULL || stitchImgPtr == NULL) {
        DEBUG_PRINT_DETAILED("null input of mat arrays");
        exit(-1);
    }

    for (i = 0; i < h; ++i) {
        stitchImgPtr += w;
        memcpy(stitchImgPtr, lImgPtr, w);
        memcpy(stitchImgPtr + w, rImgPtr, w);
    }
}

// create integral image of one image
int32 createIntegImg(
    const uint8 *imgPtr,
    int32* intImgPtr,
    int16 w,
    int16 h) {
    int32 i = 0;
    int32 j = 0;

    if (imgPtr == NULL || intImgPtr == NULL)
        return -1;

    intImgPtr[0] = imgPtr[0];

    for (i = 1; i < w; ++i) {
        intImgPtr[i] = intImgPtr[i - 1] + imgPtr[i];
    }

    for (j = 1; j < h; ++j) {
        int32 sum = 0;
        int32 curRow = j * w;

        for (i = 0; i < w; ++i) {
            sum += imgPtr[curRow + i];
            intImgPtr[curRow + i] = intImgPtr[curRow - w + i] + sum;
        }
    }

    return 0;
}

int32 gaussin(
    const uint8* srcImgPtr,
    uint8* dstImgPtr,
    int16 w,
    int16 h) {
    int i = 0;

    int imgSize = w * h;
    int imgFiltSize = w * (h - 1) - 1;

    if (srcImgPtr == NULL || dstImgPtr == NULL)
        return -1;

    // Iterate over entire image as a single, continuous raster line.
    for (i = w + 1; i < imgFiltSize; ++i) {
        // Read in the required 3x3 region from the input.
        uint8 i00 = srcImgPtr[i - w - 1];
        uint8 i01 = srcImgPtr[i - w];
        uint8 i02 = srcImgPtr[i - w + 1];

        uint8 i10 = srcImgPtr[i - 1];
        uint8 i11 = srcImgPtr[i];
        uint8 i12 = srcImgPtr[i + 1];

        uint8 i20 = srcImgPtr[i + w - 1];
        uint8 i21 = srcImgPtr[i + w];
        uint8 i22 = srcImgPtr[i + w + 1];

        // Apply horizontal and vertical filter masks.  The final filter
        // output is the sum of the absolute values of these filters.
        int16 filtVal = i00 + 2 * i01 + i02 +
                        2 * i10 + 4 * i11 + 2 * i12 +
                        i20 + 2 * i21 + i22;

        filtVal = filtVal >> 4;

        // Clamp to 8-bit range.
        if (filtVal > 255)
            filtVal = 255;

        // Store it.
        dstImgPtr[i] = (uint8)filtVal;
    }

    // boundary pixels remains the same
    for (i = 0; i <= w; ++i) {
        dstImgPtr[i] = srcImgPtr[i];
        dstImgPtr[imgSize - 1 - i] = srcImgPtr[i];
    }

    return 0;
}

uint8 getThreshByFirstVally(
    uint8* imgPtr,
    int16 w,
    int16 h) {
    int32 i = 0;
    int32 imgSize = w * h;
    uint8 dixVal = 0;
    int32 maxFreq = 0;
    int32 hist[256] = { 0 };

    if (imgPtr == NULL)
        return -1;

    for (i = 0; i < imgSize; ++i) {
        hist[imgPtr[i]]++;
    }

    for (i = 0; i < 255; ++i) {
        if (hist[i] > maxFreq) {
            maxFreq = hist[i];
            dixVal = i;
        }
    }

    i = dixVal;
    while ((i < 256) && (hist[i + 1] < hist[i])) {
        i++;
    }

    return i;
}

int32 binary(
    uint8* imgPtr,
    uint8 thresh,
    uint8 fVal,  // fVal -- foreground value
    int16 w,
    int16 h) {
    int32 i = 0;
    int32 imgSize = w * h;

    if (imgPtr == NULL)
        return -1;

    for (i = 0; i < imgSize; ++i) {
        if (imgPtr[i] > thresh)
            imgPtr[i] = fVal;
        else
            imgPtr[i] = 0;
    }

    return 0;
}

int32 elate(
    const uint8* srcImgPtr,
    uint8* dstImgPtr,
    uint8 fVal,
    int16 w,
    int16 h) {
    int32 i = 0;
    int32 imgSize = w * h;
    int32 imgSizeFilt = w * (h - 1) - 1;

    if (srcImgPtr == NULL || dstImgPtr == NULL)
        return -1;

    for (i = w + 1; i < imgSizeFilt; ++i) {
        if (srcImgPtr[i - w - 1] == fVal ||
            srcImgPtr[i - w] == fVal ||
            srcImgPtr[i - w + 1] == fVal ||
            srcImgPtr[i - 1]    == fVal ||
            srcImgPtr[i] == fVal ||
            srcImgPtr[i + 1] == fVal ||
            srcImgPtr[i + w - 1] == fVal ||
            srcImgPtr[i + w] == fVal ||
            srcImgPtr[i + w + 1] == fVal) {
            dstImgPtr[i] = fVal;
        } else {
            dstImgPtr[i] = 0;
        }
    }

    // boundary pixels remains the same
    for (i = 0; i <= w; ++i) {
        dstImgPtr[i] = srcImgPtr[i];
        dstImgPtr[imgSize - 1 - i] = srcImgPtr[i];
    }

    return 0;
}

int32 subtract(
    uint8* imgPtr,
    const uint8* minuendImgPtr,
    int16 w,
    int16 h) {
    int32 i = 0;
    int32 imgSize = w * h;

    if (imgPtr == NULL || minuendImgPtr == NULL)
        return -1;

    for (i = 0; i < imgSize; ++i) {
        imgPtr[i] = abs(imgPtr[i] - minuendImgPtr[i]);
    }

    return 0;
}

int32 equHist(
    uint8* imgPtr,
    const uint8* mImgPtr,
    uint8 bVal,
    int16 w,
    int16 h) {
    int32 i = 0;
    int32 imgSize = w * h;
    int32 imgSizeEquHist = 0;
    int32 hist[256] = { 0 };

    if (imgPtr == NULL || mImgPtr == NULL)
        return -1;

    for (i = 0; i < imgSize; ++i) {
        if (mImgPtr[i] == bVal)
            continue;

        hist[imgPtr[i]]++;
        imgSizeEquHist++;
    }

    for (i = 1; i < 256; ++i) {
        hist[i] += hist[i - 1];
    }

    for (i = 0; i < imgSize; ++i) {
        if (mImgPtr[i] == bVal)
            continue;

        imgPtr[i] = (hist[imgPtr[i]] * 255) / imgSizeEquHist;
    }

    return 0;
}

int32 markOutContour(
    uint8* imgPtr,
    Contour* contourPtr,
    int16 w,
    int16 h) {
    int16 x = 0;
    int16 y = 0;

    int16 xStart = 0;
    int16 yStart = 0;

    int16 xCur = 0;
    int16 yCur = 0;

    int16 xTmp = 0;
    int16 yTmp = 0;

    int16 offsetSeq = 0;
    int16 searchTime = 0;
    Coord coordOffsetOf8Conn[8] = { { 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 },
     { -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 } };

    int8 atStartPos = 0;

    int32 contourSeq = 0;

    int32 i = 0;

    if (imgPtr == NULL || contourPtr == NULL)
        return -1;

    for (y = 1; y < h - 1; ++y) {
        for (x = 1; x < w - 1; ++x) {
            if (imgPtr[y * w + x] == 0)
                continue;

            xStart = x;
            yStart = y;
            xCur = x;
            yCur = y;
            atStartPos = 1;
            offsetSeq = 0;
            contourPtr->num[contourSeq] = 0;

            while ((xCur != xStart || yCur != yStart || atStartPos) &&
                   (xCur > 1 && xCur < w - 1 && yCur > 1 && yCur < h - 1)) {
                atStartPos = 0;

                xTmp = xCur + coordOffsetOf8Conn[offsetSeq].x;
                yTmp = yCur + coordOffsetOf8Conn[offsetSeq].y;

                searchTime = 1;

                while (imgPtr[yTmp * w + xTmp] == 0 && searchTime <= 8) {
                    offsetSeq++;
                    searchTime++;
                    if (offsetSeq >= 8)
                        offsetSeq -= 8;

                    xTmp = xCur + coordOffsetOf8Conn[offsetSeq].x;
                    yTmp = yCur + coordOffsetOf8Conn[offsetSeq].y;
                }

                // singular point
                if (searchTime > 8) {
                    imgPtr[yCur * w + xCur] = 0;
                    break;
                }

                // add the newly founded point into the "contour"
                imgPtr[yTmp * w + xTmp] = 0;
                contourPtr->coordPtr[i].x = xTmp;
                contourPtr->coordPtr[i].y = yTmp;
                contourPtr->num[contourSeq]++;
                i++;
                xCur = xTmp;
                yCur = yTmp;

                offsetSeq += 6;
                if (offsetSeq >= 8)
                    offsetSeq -= 8;
            }

            if (xCur != xStart || yCur != yStart ||
                contourPtr->num[contourSeq] <= 3) {
                // if there is only one pixel size gap between starting point
                // and ending point, tolerate it to be a closed contour
                /*if (abs(xCur - xStart) < 3 && abs(yCur - yStart) < 3 
                && p_contours->num[contourSeq] > 3)
                {
                contourSeq++;
                continue;
                }*/

                // indicate that this stored edge points are not edge of
                // a closed contour, clear this useless stored data
                i -= contourPtr->num[contourSeq];
                contourPtr->num[contourSeq] = 0;

                // clear the starting point
                imgPtr[y * w + x] = 0;

                continue;
            }

            if (searchTime > 8)
                continue;

            // if this stored edge are of a closed contour,
            // go to store another contour.
            contourSeq++;
        }
    }

    return contourSeq;
}

int32 markMaxOutContour(
    uint8* imgPtr,
    Contour* contoursPtr,
    int32 contourNum,
    uint8 markVal,
    int16 w,
    int16 h) {
    int32 c = 0;
    int32 maxLen = 0;
    int32 i = 0;

    if (imgPtr == NULL || contoursPtr == NULL)
        return -1;

    for (c = 0; c < contourNum; ++c) {
        if (contoursPtr->num[c] > maxLen) {
            maxLen = contoursPtr->num[c];
        }
    }

    for (i = 0; i < maxLen; ++i) {
        imgPtr[contoursPtr->coordPtr[i].y * w + contoursPtr->coordPtr[i].x]
        = markVal;
    }

    return 0;
}

int32 fillRegion(
    uint8* imgPtr,
    int16 w,
    int16 h,
    uint8 fillVal) {
    int16 x = 0;
    int16 y = 0;
    int32 pos = 0;
    int32 t_pos = 0;
    int32 posLeft = 0;
    int32 posRight = 0;
    int32 numOfRegionPixels = 0;

    if (imgPtr == NULL)
        return -1;

    for (y = 1; y < h - 1; ++y) {
        int8 posLeftFound = 0;
        int8 posRightFound = 0;

        for (x = 1; x < w - 1; ++x) {
            pos = y * w + x;

            if (imgPtr[pos] == 0)
                continue;

            if (!posLeftFound && !posRightFound) {
                while (imgPtr[pos + 1] != 0) {
                    pos++;
                    x++;
                }

                if (imgPtr[pos - w - 1] != 0 || imgPtr[pos - w] != 0
                    || imgPtr[pos - w + 1] != 0 || imgPtr[pos - 1] != 0
                    || imgPtr[pos + w - 1] != 0 || imgPtr[pos + w] != 0
                    || imgPtr[pos + w + 1] != 0) {
                    posLeftFound = 1;
                    posLeft = pos;
                }
            } else if (posLeftFound) {
                if (imgPtr[pos - w - 1] != 0 || imgPtr[pos - w] != 0
                    || imgPtr[pos - w + 1] != 0 || imgPtr[pos - 1] != 0
                    || imgPtr[pos + 1] != 0 || imgPtr[pos + w - 1] != 0
                    || imgPtr[pos + w] != 0 || imgPtr[pos + w + 1] != 0) {
                    posLeftFound = 0;
                    posRightFound = 1;
                    posRight = pos;
                    for (t_pos = posLeft + 1; t_pos < posRight; ++t_pos) {
                        imgPtr[t_pos] = fillVal;
                    }

                    while (imgPtr[pos + 1] != 0) {
                        x++;
                        pos++;
                    }
                    x--;
                    posRightFound = 0;
                }
            }
        }
    }

    for (pos = 0; pos < w * h; ++pos) {
        if (imgPtr[pos] != 0)
            numOfRegionPixels++;
    }

    return numOfRegionPixels;
}


int32 preProcess(
    uint8* imgPtr,
    uint8* tImgPtr,
    uint8* mImgPtr,
    Contour* contourPtr,
    PRE_PROC_MODE mode,
    int16 w,
    int16 h) {
    int32 imgSize = w * h;
    int32 contourNum = 0;
    int32 foregroundImgSize = 0;

    // local foreground histogram equalization
    if (mode == LOCAL_EQUHIST) {
        gaussin(imgPtr, tImgPtr, w, h);

        binary(tImgPtr, getThreshByFirstVally(tImgPtr, w, h), FORE_GROUND_PIXEL, w, h);

        elate(tImgPtr, mImgPtr, FORE_GROUND_PIXEL, w, h);

        subtract(mImgPtr, tImgPtr, w, h);

        contourNum = markOutContour(mImgPtr, contourPtr, w, h);
        markMaxOutContour(mImgPtr, contourPtr, contourNum, FORE_GROUND_PIXEL,
                          w, h);
        foregroundImgSize = fillRegion(mImgPtr, w, h, FORE_GROUND_PIXEL);
        equHist(imgPtr, mImgPtr, BACK_GROUND_PIXEL, w, h);
    } else if (mode == GET_PREGROUND) {
        gaussin(imgPtr, tImgPtr, w, h);
        memcpy(imgPtr, tImgPtr, imgSize);

        binary(tImgPtr, getThreshByFirstVally(imgPtr, w, h), FORE_GROUND_PIXEL,
               w, h);

        elate(tImgPtr, mImgPtr, FORE_GROUND_PIXEL, w, h);

        subtract(mImgPtr, tImgPtr, w, h);

        contourNum = markOutContour(mImgPtr, contourPtr, w, h);
        markMaxOutContour(mImgPtr, contourPtr, contourNum, FORE_GROUND_PIXEL,
                          w, h);
        foregroundImgSize = fillRegion(mImgPtr, w, h, FORE_GROUND_PIXEL);
    } else if (mode == GLOBAL_EUQHIST) {
        gaussin(imgPtr, tImgPtr, w, h);
        memcpy(imgPtr, tImgPtr, imgSize);

        equHist(imgPtr, mImgPtr, BACK_GROUND_PIXEL, w, h);
    } else if (mode == ONLY_GLOBAL_GUASSIN) {
        gaussin(imgPtr, tImgPtr, w, h);
        memcpy(imgPtr, tImgPtr, imgSize);
    }

    return foregroundImgSize;
}




#include "../inc/imgMath.h"
#include "../inc/error.h"
#include <string.h>


unsigned char* g_t_imgPtr = NULL;
unsigned char* g_markImgPtr = NULL;
Contour* g_contourPtr = NULL;


/*geometry*/
// calculate the neighbor pixel coordinates
int calcDiskTmplArray(Coord* coordPtr, short r)
{
	int count = 0;
	int r2 = r * r;

	int x = 0;
	int y = 0;

	if (coordPtr == NULL)
		return -1;

	for (x = -r; x <= r; ++x)
	{
		for (y = -r; y <= r; ++y)
		{
			if ((x * x + y * y) <= r2)
			{
				coordPtr->x = x;
				coordPtr->y = y;

				++coordPtr;
				++count;
			}
		}
	}

	return count;
}

// create integral image of one image
int createIntegImg(const unsigned char *imgPtr, int* integImgPtr, short w, short h)
{
	int i = 0;
	int j = 0;

	if (imgPtr == NULL || integImgPtr == NULL)
		return -1;

	integImgPtr[0] = imgPtr[0];

	for (i = 1; i < w; ++i)
	{
		integImgPtr[i] = integImgPtr[i - 1] + imgPtr[i];
	}

	for (j = 1; j < h; ++j)
	{
		int sum = 0;
		int curRow = j * w;

		for (i = 0; i < w; ++i)
		{
			sum += imgPtr[curRow + i];
			integImgPtr[curRow + i] = integImgPtr[curRow - w + i] + sum;
		}
	}

	return 0;
}

/*filtering*/
int gaussin(const unsigned char* srcImgPtr, unsigned char* dstImgPtr, short w, short h)
{
	int i = 0;

	int filtVal = 0;
	unsigned char  i00, i01, i02;
	unsigned char  i10, i11, i12;
	unsigned char  i20, i21, i22;

	int imgSize = w * h;
	int imgFiltSize = w * (h - 1) - 1;

	if (srcImgPtr == NULL || dstImgPtr == NULL)
		return -1;

	// Iterate over entire image as a single, continuous raster line.
	for (i = w + 1; i < imgFiltSize; ++i)
	{
		// Read in the required 3x3 region from the input.
		i00 = srcImgPtr[i - w - 1]; 		i01 = srcImgPtr[i - w]; 		i02 = srcImgPtr[i - w + 1];
		i10 = srcImgPtr[i - 1];			i11 = srcImgPtr[i];			i12 = srcImgPtr[i + 1];
		i20 = srcImgPtr[i + w - 1];		i21 = srcImgPtr[i + w]; 	i22 = srcImgPtr[i + w + 1];

		// Apply horizontal and vertical filter masks.  The final filter
		// output is the sum of the absolute values of these filters.
		filtVal =		i00 + 2 * i01 + i02 +
					2 * i10 + 4 * i11 + 2 * i12 +
						i20 + 2 * i21 + i22;

		filtVal = filtVal >> 4;

		// Clamp to 8-bit range.
		if (filtVal > 255)
			filtVal = 255;

		// Store it.
		dstImgPtr[i] = filtVal;
	}

	// boundary pixels remains the same
	for (i = 0; i <= w; ++i)
	{
		dstImgPtr[i] = srcImgPtr[i];
		dstImgPtr[imgSize - 1 - i] = srcImgPtr[i];
	}

	return 0;
}


unsigned char getThreshByFirstVally(unsigned char* imgPtr, short w, short h)
{
	int i = 0;
	int imgSize = w * h;
	unsigned char dixVal = 0;
	int maxFreq = 0;
	int hist[256] = { 0 };

	if (imgPtr == NULL)
		return -1;

	for (i = 0; i < imgSize; ++i)
	{
		hist[imgPtr[i]] ++;
	}

	for (i = 0; i < 255; ++i)
	{
		if (hist[i] > maxFreq)
		{
			maxFreq = hist[i];
			dixVal = i;
		}
	}

	i = dixVal;
	while (hist[i + 1] < hist[i] && i < 256)
	{
		i++;
	}

	return i;
}

// fVal -- foreground value
int binary(unsigned char* imgPtr, unsigned char thresh, unsigned char fVal, short w, short h)
{
	int i = 0;
	int imgSize = w * h;

	if (imgPtr == NULL)
		return -1;

	for (i = 0; i < imgSize; ++i)
	{
		if (imgPtr[i] > thresh)
			imgPtr[i] = fVal;
		else
			imgPtr[i] = 0;
	}

	return 0;
}

// fVal -- foreground value
int elate(const unsigned char* srcImgPtr, unsigned char* dstImgPtr, unsigned char fVal, short w, short h)
{
	int i = 0;
	int imgSize = w * h;
	int imgSizeFilt = w * (h - 1) - 1;

	if (srcImgPtr == NULL || dstImgPtr == NULL)
		return -1;

	for (i = w + 1; i < imgSizeFilt; ++i)
	{
		if (srcImgPtr[i - w - 1] == fVal || 
			srcImgPtr[i - w] == fVal || 
			srcImgPtr[i - w + 1] == fVal ||
			srcImgPtr[i - 1]	== fVal || 
			srcImgPtr[i] == fVal || 
			srcImgPtr[i + 1] == fVal ||
			srcImgPtr[i + w - 1] == fVal || 
			srcImgPtr[i + w] == fVal || 
			srcImgPtr[i + w + 1] == fVal)
		{
			dstImgPtr[i] = fVal;
		}
		else
			dstImgPtr[i] = 0;
	}

	// boundary pixels remains the same
	for (i = 0; i <= w; ++i)
	{
		dstImgPtr[i] = srcImgPtr[i];
		dstImgPtr[imgSize - 1 - i] = srcImgPtr[i];
	}

	return 0;
}


int subtract(unsigned char* imgPtr, const unsigned char* minuendImgPtr, short w, unsigned short h)
{
	int i = 0;
	int imgSize = w * h;

	if (imgPtr == NULL || minuendImgPtr == NULL)
		return -1;

	for (i = 0; i < imgSize; ++i)
	{
		imgPtr[i] = abs(imgPtr[i] - minuendImgPtr[i]);
	}

	return 0;
}


int equHist(unsigned char* imgPtr, const unsigned char* markImgPtr, unsigned char bVal, short w, short h)
{
	int i = 0;
	int imgSize = w * h;
	int imgSizeEquHist = 0;
	int hist[256] = { 0 };

	if (imgPtr == NULL || markImgPtr == NULL)
		return -1;

	for (i = 0; i < imgSize; ++i)
	{
		if (markImgPtr[i] == bVal)
			continue;

		hist[imgPtr[i]]++;
		imgSizeEquHist++;
	}

	for (i = 1; i < 256; ++i)
	{
		hist[i] += hist[i - 1];
	}

	for (i = 0; i < imgSize; ++i)
	{
		if (markImgPtr[i] == bVal)
			continue;

		imgPtr[i] = (hist[imgPtr[i]] * 255) / imgSizeEquHist;
	}

	return 0;
}


int markOutContour(unsigned char* imgPtr, Contour* contourPtr, short w, short h)
{
	short x = 0;
	short y = 0;

	short xStart = 0;
	short yStart = 0;

	short xCur = 0;
	short yCur = 0;

	short xTmp = 0;
	short yTmp = 0;

	short offsetSeq = 0;
	short searchTime = 0;
	Coord coordOffsetOf8Conn[8] = { { 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 }, { -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 } };

	char atStartPos = 0;

	int contourSeq = 0;

	int i = 0;
	int j = 0;

	if (imgPtr == NULL || contourPtr == NULL)
		return -1;

	for (y = 1; y < h - 1; ++y)
	{
		for (x = 1; x < w - 1; ++x)
		{
			if (imgPtr[y * w + x] == 0)
				continue;

			xStart = x;
			yStart = y;
			xCur = x;
			yCur = y;
			atStartPos = 1;
			offsetSeq = 0;
			contourPtr->num[contourSeq] = 0;

			while ((xCur != xStart || yCur != yStart || atStartPos) && (xCur > 1 && xCur < w - 1 && yCur > 1 && yCur < h - 1))
			{
				atStartPos = 0;

				xTmp = xCur + coordOffsetOf8Conn[offsetSeq].x;
				yTmp = yCur + coordOffsetOf8Conn[offsetSeq].y;

				searchTime = 1;

				while (imgPtr[yTmp * w + xTmp] == 0 && searchTime <= 8)
				{
					offsetSeq++;
					searchTime++;
					if (offsetSeq >= 8)
						offsetSeq -= 8;

					xTmp = xCur + coordOffsetOf8Conn[offsetSeq].x;
					yTmp = yCur + coordOffsetOf8Conn[offsetSeq].y;
				}

				// singular point
				if (searchTime > 8)
				{
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

			if (xCur != xStart || yCur != yStart || contourPtr->num[contourSeq] <= 3)
			{
				// if there is only one pixel size gap between starting point and ending point, tolerate it to be a closed contour
				/*if (abs(xCur - xStart) < 3 && abs(yCur - yStart) < 3 && p_contours->num[contourSeq] > 3)
				{
				contourSeq++;
				continue;
				}*/

				// indicate that this stored edge points are not edge of a closed contour, clear this useless stored data
				i -= contourPtr->num[contourSeq];
				contourPtr->num[contourSeq] = 0;

				// clear the starting point
				imgPtr[y * w + x] = 0;

				continue;
			}

			if (searchTime > 8)
				continue;

			// if this stored edge are of a closed contour, go to store another contour.
			contourSeq++;
		}
	}

	return contourSeq;
}


int markMaxOutContour(unsigned char* imgPtr, Contour* contoursPtr, int contourNum, unsigned char markVal, short w, short h)
{
	int c = 0;
	int maxLen = 0;
	int cMax = 0;
	int i = 0;

	if (imgPtr == NULL || contoursPtr == NULL)
		return -1;

	for (c = 0; c < contourNum; ++c)
	{
		if (contoursPtr->num[c] > maxLen)
		{
			maxLen = contoursPtr->num[c];
			cMax = c;
		}
	}

	for (i = 0; i < maxLen; ++i)
	{
		imgPtr[contoursPtr->coordPtr[i].y * w + contoursPtr->coordPtr[i].x] = markVal;
	}

	return 0;
}


int fillRegion(unsigned char* imgPtr, short w, short h, unsigned char fillVal)
{
	short x = 0;
	short y = 0;
	int pos = 0;
	int t_pos = 0;
	int posLeft = 0;
	int posRight = 0;
	char posLeftFound = 0;
	char posRightFound = 0;
	int numOfRegionPixels = 0;

	if (imgPtr == NULL)
		return -1;

	for (y = 1; y < h - 1; ++y)
	{
		posLeftFound = 0;
		posRightFound = 0;

		for (x = 1; x < w - 1; ++x)
		{
			pos = y * w + x;

			if (imgPtr[pos] == 0)
				continue;

			if (!posLeftFound && !posRightFound)
			{
				while (imgPtr[pos + 1] != 0)
				{
					pos++;
					x++;
				}

				if (imgPtr[pos - w - 1] != 0 || imgPtr[pos - w] != 0 || imgPtr[pos - w + 1] != 0
					|| imgPtr[pos - 1] != 0
					|| imgPtr[pos + w - 1] != 0 || imgPtr[pos + w] != 0 || imgPtr[pos + w + 1] != 0)
				{
					posLeftFound = 1;
					posLeft = pos;
				}
			}
			else if (posLeftFound)
			{
				if (imgPtr[pos - w - 1] != 0 || imgPtr[pos - w] != 0 || imgPtr[pos - w + 1] != 0
					|| imgPtr[pos - 1] != 0 || imgPtr[pos + 1] != 0
					|| imgPtr[pos + w - 1] != 0 || imgPtr[pos + w] != 0 || imgPtr[pos + w + 1] != 0)
				{
					posLeftFound = 0;
					posRightFound = 1;
					posRight = pos;
					for (t_pos = posLeft + 1; t_pos < posRight; ++t_pos)
					{
						imgPtr[t_pos] = fillVal;
					}

					while (imgPtr[pos + 1] != 0)
					{
						x++;
						pos++;
					}
					x--;
					posRightFound = 0;
				}
			}
		}
	}

	for (pos = 0; pos < w * h; ++pos)
	{
		if (imgPtr[pos] != 0)
			numOfRegionPixels++;
	}

	return numOfRegionPixels;
}


int preProcess(unsigned char* imgPtr, PRE_PROC_MODE mode, short w, short h)
{
	int imgSize = w * h;
	int contourNum = 0;
	int foregroundImgSize = 0;

	// local foreground historgram equalization
	if (mode == LOCAL_EQUHIST)
	{
		gaussin(imgPtr, g_t_imgPtr, w, h);
		memcpy(imgPtr, g_t_imgPtr, imgSize);

		binary(g_t_imgPtr, getThreshByFirstVally(imgPtr, w, h), FORE_GROUND_PIXEL, w, h);

		elate(g_t_imgPtr, g_markImgPtr, FORE_GROUND_PIXEL, w, h);

		subtract(g_markImgPtr, g_t_imgPtr, w, h);

		contourNum = markOutContour(g_markImgPtr, g_contourPtr, w, h);
		markMaxOutContour(g_markImgPtr, g_contourPtr, contourNum, FORE_GROUND_PIXEL, w, h);
		foregroundImgSize = fillRegion(g_markImgPtr, w, h, FORE_GROUND_PIXEL);
		equHist(imgPtr, g_markImgPtr, BACK_GROUND_PIXEL, w, h);
	}
	// local foreground extraction only
	else if (mode == GET_PREGROUND)
	{
		gaussin(imgPtr, g_t_imgPtr, w, h);
		memcpy(imgPtr, g_t_imgPtr, imgSize);

		binary(g_t_imgPtr, getThreshByFirstVally(imgPtr, w, h), FORE_GROUND_PIXEL, w, h);

		elate(g_t_imgPtr, g_markImgPtr, FORE_GROUND_PIXEL, w, h);

		subtract(g_markImgPtr, g_t_imgPtr, w, h);

		contourNum = markOutContour(g_markImgPtr, g_contourPtr, w, h);
		markMaxOutContour(g_markImgPtr, g_contourPtr, contourNum, FORE_GROUND_PIXEL, w, h);
		foregroundImgSize = fillRegion(g_markImgPtr, w, h, FORE_GROUND_PIXEL);
	}
	// global foreground histogram equalization
	else if (mode == GLOBAL_EUQHIST)
	{
		gaussin(imgPtr, g_t_imgPtr, w, h);
		memcpy(imgPtr, g_t_imgPtr, imgSize);

		equHist(imgPtr, g_markImgPtr, BACK_GROUND_PIXEL, w, h);
	}
	// global foreground guassin only
	else if (mode == ONLY_GLOBAL_GUASSIN)
	{
		gaussin(imgPtr, g_t_imgPtr, w, h);
		memcpy(imgPtr, g_t_imgPtr, imgSize);
	}

	return foregroundImgSize;
}


// combine the images into one single image
cv::Mat mergeMats(const cv::Mat *p_matArr, unsigned short matNum, Orientation orient)
{
	if (p_matArr == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of mat arrays");
		exit(-1);
	}

	cv::Size size = getMergeSize(p_matArr, matNum, orient);
	cv::Mat mergedImg(size, CV_8UC3);

	unsigned short leftTopX = 0;
	unsigned short leftTopY = 0;
	for (unsigned short i = 0; i < matNum; ++i)
	{
		cv::Mat tmpMat = mergedImg(cv::Rect(leftTopX, leftTopY, p_matArr[i].cols, p_matArr[i].rows));
		p_matArr[i].copyTo(tmpMat);

		leftTopX += orient == horizontal ? p_matArr[i].cols : 0;
		leftTopY += orient == vertical ? p_matArr[i].rows : 0;
	}

	return mergedImg;
}

void stitchImage
(
unsigned char* p_imgL,
unsigned char* p_imgR,
unsigned char* p_imgStick,
short w,
short h
)
{

}
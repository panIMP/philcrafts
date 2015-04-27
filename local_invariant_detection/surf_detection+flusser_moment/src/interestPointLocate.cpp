#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

#include <DTool\Error\error.h>
#include <MTool\EquationSolve\EquationSolve.h>
#include <MTool\FeatDist\MahaDist.h>
#include <MTool\FeatDist\EuroDist.h>
#include <footstone\ImgFilter\imgMath.h>

#include <malloc.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <iostream>

#include "interestPointLocate.h"
#include "imgIO.h"


double** matCovarInv = NULL;


using namespace std;


// filters parameters used to generate the Det(Hessin) pyramid for interest point location
// outer array is: layer number
// inner array is: filter number(xx, yy, xy) multiplies parameter number
// parameter from left to right are:
// start =============================
// box filter number,
// first box top left corner coordinate, first box bottom right corner coordinate,
// first box area, first box weight,
// second box top left corner coordinate, ...
// ...
// last box top left corner coordinate, ...
// ...
// correspording deata value of the simulated box filter of second order derivative of guassin.
// end ===============================
const Filter g_filts[LAYER_NUM] =
{
	// 0st layer -- box size: 9 * 9
	{
		3, 0, 2, 2, 6, 15, 1, 3, 2, 5, 6, 15, -2, 6, 2, 8, 6, 15, 1, 0, 0, 0, 0, 0, 0,
		3, 2, 0, 6, 2, 15, 1, 2, 3, 6, 5, 15, -2, 2, 6, 6, 8, 15, 1, 0, 0, 0, 0, 0, 0,
		4, 1, 1, 3, 3, 9, 1, 5, 1, 7, 3, 9, -1, 1, 5, 3, 7, 9, -1, 5, 5, 7, 7, 9, 1,
	},

	// 1st layer -- box size: 15 * 15
	{
		3, 0, 3, 4, 11, 45, 1, 5, 3, 9, 11, 45, -2, 10, 3, 14, 11, 45, 1, 0, 0, 0, 0, 0, 0,
		3, 3, 0, 11, 4, 45, 1, 3, 5, 11, 9, 45, -2, 3, 10, 11, 14, 45, 1, 0, 0, 0, 0, 0, 0,
		4, 2, 2, 6, 6, 25, 1, 8, 2, 12, 6, 25, -1, 2, 8, 6, 12, 25, -1, 8, 8, 12, 12, 25, 1,
	},

	// 2nd layer -- box size: 21 * 21
	{
		3, 0, 4, 6, 16, 91, 1, 7, 4, 13, 16, 91, -2, 14, 4, 20, 16, 91, 1, 0, 0, 0, 0, 0, 0,
		3, 4, 0, 16, 6, 91, 1, 4, 7, 16, 13, 91, -2, 4, 14, 16, 20, 91, 1, 0, 0, 0, 0, 0, 0,
		4, 3, 3, 9, 9, 49, 1, 11, 3, 17, 9, 49, -1, 3, 11, 9, 17, 49, -1, 11, 11, 17, 17, 49, 1,
	},

	// 3rd layer -- box size: 27 * 27
	{
		3, 0, 5, 8, 21, 153, 1, 9, 5, 17, 21, 153, -2, 18, 5, 26, 21, 153, 1, 0, 0, 0, 0, 0, 0,
		3, 5, 0, 21, 8, 153, 1, 5, 9, 21, 17, 153, -2, 5, 18, 21, 26, 153, 1, 0, 0, 0, 0, 0, 0,
		4, 4, 4, 12, 12, 81, 1, 14, 4, 22, 12, 81, -1, 4, 14, 12, 22, 81, -1, 14, 14, 22, 22, 81, 1,
	},

	// 4th layer -- box size: 39 * 39
	{
		3, 0, 7, 12, 31, 325, 1, 13, 7, 25, 31, 325, -2, 26, 7, 38, 31, 325, 1, 0, 0, 0, 0, 0, 0,
		3, 7, 0, 31, 12, 325, 1, 7, 13, 31, 25, 325, -2, 7, 26, 31, 38, 325, 1, 0, 0, 0, 0, 0, 0,
		4, 6, 6, 18, 18, 169, 1, 20, 6, 32, 18, 169, -1, 6, 20, 18, 32, 169, -1, 20, 20, 32, 32, 169, 1,
	},

	// 5th layer -- box size: 51 * 51
	{
		3, 0, 9, 16, 41, 561, 1, 17, 9, 33, 41, 561, -2, 34, 9, 50, 41, 561, 1, 0, 0, 0, 0, 0, 0,
		3, 9, 0, 41, 16, 561, 1, 9, 17, 41, 33, 561, -2, 9, 34, 41, 50, 561, 1, 0, 0, 0, 0, 0, 0,
		4, 8, 8, 24, 24, 289, 1, 26, 8, 42, 24, 289, -1, 8, 26, 24, 42, 289, -1, 26, 26, 42, 42, 289, 1,
	},

/*	// 6th layer -- box size: 75 * 75
	{
		3, 0, 13, 24, 61, 1225, 1, 25, 13, 49, 61, 1225, -2, 50, 13, 74, 61, 1225, 1, 0, 0, 0, 0, 0, 0,
		3, 13, 0, 61, 24, 1225, 1, 13, 25, 61, 49, 1225, -2, 13, 50, 61, 74, 1225, 1, 0, 0, 0, 0, 0, 0,
		4, 12, 12, 36, 36, 625, 1, 38, 12, 62, 36, 625, -1, 12, 38, 36, 62, 625, -1, 38, 38, 62, 62, 625, 1,
	},

	// 7th player -- box size: 99 * 99
	{
		3, 0, 17, 32, 81, 2145, 1, 33, 17, 65, 81, 2145, -2, 66, 17, 98, 81, 2145, 1, 0, 0, 0, 0, 0, 0,
		3, 17, 0, 81, 32, 2145, 1, 17, 33, 81, 65, 2145, -2, 17, 66, 81, 98, 2145, 1, 0, 0, 0, 0, 0, 0,
		4, 16, 16, 48, 48, 1089, 1, 50, 16, 82, 48, 1089, -1, 16, 50, 48, 82, 1089, -1, 50, 50, 82, 82, 1089, 1,
	}*/
};

const unsigned char g_upOffset[] = { 0, 1, 1, 1, 1, 1, 1, 0 };
const unsigned char g_downOffset[] = { 0, 1, 1, 2, 1, 2, 1, 0 };
const double g_sigma[] = {1.2, 2, 2.8, 3.6, 5.2};

// init the pointers of the four corners of each box of certain filter
void initFiltPtrs(const int *p_integImg, FilterPtrs* p_filterPtrs, const Filter* p_filter, short w)
{
	if (p_integImg == NULL || p_filterPtrs == NULL || p_filter == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of integral image pointer or filter corner pointers or filter template pointer");
		exit(-1);
	}

	for (int m = 0; m < MODE_NUM; ++m)
	{
		int boxNum = p_filter->mode[m].boxNum;
		p_filterPtrs->mptrs[m].boxNum = boxNum;

		for (int b = 0; b < boxNum; ++b)
		{
			p_filterPtrs->mptrs[m].bptrs[b].p_boxTL = p_integImg + w * p_filter->mode[m].box[b].tLY + p_filter->mode[m].box[b].tLX;
			p_filterPtrs->mptrs[m].bptrs[b].p_boxTR = p_integImg + w * p_filter->mode[m].box[b].tLY + p_filter->mode[m].box[b].bRX + 1;
			p_filterPtrs->mptrs[m].bptrs[b].p_boxBL = p_integImg + w * (p_filter->mode[m].box[b].bRY + 1) + p_filter->mode[m].box[b].tLX;
			p_filterPtrs->mptrs[m].bptrs[b].p_boxBR = p_integImg + w * (p_filter->mode[m].box[b].bRY + 1) + p_filter->mode[m].box[b].bRX + 1;
		}
	}
}

// increase the pointers of the four corners of each box of certain filter by certain value
void incFilterPtrs(FilterPtrs* p_filterPtrs, short val)
{
	if (p_filterPtrs == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of filter corner pointers");
		exit(-1);
	}

	for (unsigned short m = 0; m < MODE_NUM; ++m)
	{
		unsigned short boxNum = p_filterPtrs->mptrs[m].boxNum;
		for (unsigned short b = 0; b < boxNum; ++b)
		{
			p_filterPtrs->mptrs[m].bptrs[b].p_boxTL += val;
			p_filterPtrs->mptrs[m].bptrs[b].p_boxTR += val;
			p_filterPtrs->mptrs[m].bptrs[b].p_boxBL += val;
			p_filterPtrs->mptrs[m].bptrs[b].p_boxBR += val;
		}
	}
}

// calculate the det(Hessin) response of one pixel
void calcDetHes(const FilterPtrs *p_filterPtrs, const Filter* p_filter, hesMat* p_detHesImg, char layerSeq)
{
	if (p_filterPtrs == NULL || p_detHesImg == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of filter corner pointers or Det(Hessin) image pointer");
		exit(-1);
	}

	double deriVal[MODE_NUM] = {0.0, 0.0, 0.0};

	for (unsigned short m = 0; m < MODE_NUM; ++m)
	{
		unsigned short boxNum = p_filterPtrs->mptrs[m].boxNum;
		unsigned int val = 0;
		for (unsigned short b = 0; b < boxNum; ++b)
		{
			val = *(p_filterPtrs->mptrs[m].bptrs[b].p_boxBR) + *(p_filterPtrs->mptrs[m].bptrs[b].p_boxTL) - *(p_filterPtrs->mptrs[m].bptrs[b].p_boxTR) - *(p_filterPtrs->mptrs[m].bptrs[b].p_boxBL);

			deriVal[m] += (double)val * (double)(p_filter->mode[m].box[b].wei) / (double)(p_filter->mode[m].box[b].s);
		}
	}

	//p_detHesImg->dxx[layOrder] = deriVal[0];
	//p_detHesImg->dyy[layOrder] = deriVal[1];
	//p_detHesImg->dxy[layOrder] = deriVal[2];

	p_detHesImg->val[layerSeq] = deriVal[0] * deriVal[1] - 0.912 * 0.912 * deriVal[2] * deriVal[2];
}

// create the det(Hessin) image of the input integral image at one specific octave and layer
void createDetHesImg(const int* p_integImg, hesMat* p_detHesImg, const unsigned char* p_markImg, short layOrder, short w, short h)
{
	if (p_integImg == NULL || p_detHesImg == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of integral image pointer or Det(Hessin) image");
		exit(-1);
	}

	// pointer to filter of current layer
	const Filter* p_filter = &g_filts[layOrder];

	// coordinates of the starting processing pixle of current layer 
	// must move to the next diagonal pixel to prevent boundary except
	unsigned short stX = (p_filter->mode[0].box[1].tLX + p_filter->mode[0].box[1].bRX) / 2 + 1;
	unsigned short stY = (p_filter->mode[0].box[1].tLY + p_filter->mode[0].box[1].bRY) / 2 + 1;
	unsigned short enX = w - stX;
	unsigned short enY = h - stY;

	// init pointers of the filter box corners
	FilterPtrs filterPtrs;
	initFiltPtrs(p_integImg, &filterPtrs, p_filter, w);

	hesMat* t_p_detHesImg;
	FilterPtrs t_filterPtrs;
	for (unsigned short y = stY; y < enY; ++y, incFilterPtrs(&filterPtrs, w))
	{
		t_p_detHesImg = p_detHesImg + y * w + stX;
		t_filterPtrs = filterPtrs;

		for (unsigned short x = stX; x < enX; ++x, ++t_p_detHesImg, incFilterPtrs(&t_filterPtrs, 1))
		{
			if (p_markImg[y * w + x] != FORE_GROUND_PIXEL)
				continue;

			calcDetHes(&t_filterPtrs, p_filter, t_p_detHesImg, layOrder);
		}
	}
}

// create the det(Hessin) image pyramid of certain number of octaves and layers
void createDetHesImgPyr(hesMat *p_detHesImgPyr, const int *p_integImg, const unsigned char* p_markImg, short layNum, short w, short h)
{
	if (p_integImg == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of the pointer of integral image");
		exit(-1);
	}

	unsigned int totalSize = w * h;

	// create pyramid
	for (unsigned short layOrder = 0; layOrder < layNum; ++layOrder)
	{
		//time_t start = clock();
		createDetHesImg(p_integImg, p_detHesImgPyr, p_markImg, layOrder, w, h);
		//time_t end = clock();
		//cout << "Time for one layer of detHes image: " << end - start << endl;
	}
}


// find the interest point
// returns 0: current pixel is not the regional maximum point
// returns 1: current pixel is the regional maximum point
int isRegionMaximum(const hesMat* p_in, short w, char layOrder)
{
	if (p_in == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of image pointer");
		exit(-1);
	}

	double i00, i01, i02;
	double i10, i11, i12;
	double i20, i21, i22;

	// current layer of image
	unsigned char curLayer = layOrder;

	i00 = (p_in-w-1)->val[curLayer];    i01 = (p_in-w)->val[curLayer];    i02 = (p_in-w+1)->val[curLayer];
	i10 = (p_in  -1)->val[curLayer];    i11 = (p_in  )->val[curLayer];    i12 = (p_in  +1)->val[curLayer];
	i20 = (p_in+w-1)->val[curLayer];    i21 = (p_in+w)->val[curLayer];    i22 = (p_in+w+1)->val[curLayer];

	if (i11 <= i00 || i11 <= i01 || i11 <= i02
			|| i11 <= i10 || i11 <= i12
			|| i11 <= i20 || i11 <= i21 || i11 <= i22)
		return 0;

	double val = p_in->val[layOrder];

	// upper layer of image
	curLayer = layOrder + g_upOffset[layOrder];

	i00 = (p_in-w-1)->val[curLayer];    i01 = (p_in-w)->val[curLayer];    i02 = (p_in-w+1)->val[curLayer];
	i10 = (p_in  -1)->val[curLayer];    i11 = (p_in  )->val[curLayer];    i12 = (p_in  +1)->val[curLayer];
	i20 = (p_in+w-1)->val[curLayer];    i21 = (p_in+w)->val[curLayer];    i22 = (p_in+w+1)->val[curLayer];

	if (val <= i00 || val <= i01 || val <= i02
			|| val <= i10 || val <= i11 || val <= i12
			|| val <= i20 || val <= i21 || val <= i22)
		return 0;

	// lower layer of image
	curLayer = layOrder - g_downOffset[layOrder];

	i00 = (p_in-w-1)->val[curLayer];    i01 = (p_in-w)->val[curLayer];    i02 = (p_in-w+1)->val[curLayer];
	i10 = (p_in  -1)->val[curLayer];    i11 = (p_in  )->val[curLayer];    i12 = (p_in  +1)->val[curLayer];
	i20 = (p_in+w-1)->val[curLayer];    i21 = (p_in+w)->val[curLayer];    i22 = (p_in+w+1)->val[curLayer];

	if (val <= i00 || val <= i01 || val <= i02
			|| val <= i10 || val <= i11 || val <= i12
			|| val <= i20 || val <= i21 || val <= i22)
		return 0;

	// current pixel is the maximum of 26 neighbour pixels
	return 1;
}


// get the interest points location
// returns the number of the founded interst points
unsigned int getPointsLocations(InterestPoint* p_points, unsigned char* p_markImg, const unsigned char* p_img, const hesMat* p_detHesImgPyr, short layNum, double detHesThresh, short w, short h)
{
	if (p_points == NULL || p_detHesImgPyr == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of interest pointe set pointer or Det(Hessin) pointer");
		exit(-1);
	}

	unsigned int totalSize = w * h;
	unsigned int filtSize = w * (h - 1) - 1;
	unsigned int pointNum = 0;

	// former implementation of finding interest point locations
	const hesMat* p_detHesImgCur = p_detHesImgPyr + w + 1;

	double maxVal = FEAT_MIN;
	double curVal = 0;
	unsigned char maxLayOrder = 0;
	int c = 0;

	for (c = w + 1; c != filtSize; ++c, ++p_detHesImgCur)
	{
		if (p_markImg[c] != FORE_GROUND_PIXEL)
			continue;

		// check if the extreme point exists or not
		maxVal = FEAT_MIN;
		for (unsigned short layOrder = 1; layOrder < LAYER_NUM - 1; ++layOrder)
		{
			curVal = p_detHesImgCur->val[layOrder];

			if (curVal > maxVal)
			{
				maxVal = curVal;
				maxLayOrder = layOrder;
			}
		}

		if (maxVal < detHesThresh)
			continue;

		if (isRegionMaximum(p_detHesImgCur, w, maxLayOrder))
		{
			short y = c / w;
			short x = c - y * w;
			short r = g_sigma[maxLayOrder] * 6;

			if (x - r < 0 || x + r >= w || y - r < 0 || y + r >= h)
				continue;

			// mark the interest points first
			p_markImg[c] = MARKED_TRUE;

			// if current pixel is maximum, then its neighbours are not maximum
			p_markImg[c + 1] = MARKED_FALSE;
			p_markImg[c + w] = MARKED_FALSE;
			p_markImg[c + w + 1] = MARKED_FALSE;

			// store the interest point
			p_points->c.y = y;
			p_points->c.x = x;
			p_points->r = r;

			p_points++;
			pointNum++;

			++c;
			++p_detHesImgCur;
		}
	}

	return pointNum;
}


// calculate the feature of one interest point
void calcFeat(InterestPoint* p_point, const unsigned char* p_img, const Coord* p_coords, int neighPointNum, short w)
{
	if (p_point == NULL || p_img == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of interest pointer set pointer or image pointer or array pointer");
		exit(-1);
	}

	short x0 = p_point->c.x;
	short y0 = p_point->c.y;

	unsigned int x, y, x2, y2, xy, x3, y3, x2y, xy2;

	double val, valEven, valSigma;
	double sum = 0;
	double sumSigma = 0;
	double sumX = 0;
	double sumXSigma = 0;
	double sumY = 0;
	double sumYSigma = 0;
	double sumX2 = 0;
	double sumX2Sigma = 0;
	double sumY2 = 0;
	double sumY2Sigma = 0;
	double sumXY = 0;
	double sumXYSigma = 0;
	double sumX3 = 0;
	double sumX3Sigma = 0;
	double sumY3 = 0;
	double sumY3Sigma = 0;
	double sumX2Y = 0;
	double sumX2YSigma = 0;
	double sumXY2 = 0;
	double sumXY2Sigma = 0;

	const Coord* p_coordsCur = p_coords;
	const Coord* p_coordsEnd = p_coords + neighPointNum;
	for (p_coordsCur = p_coords; p_coordsCur != p_coordsEnd; ++p_coordsCur)
	{
		sum += p_img[(p_coordsCur->y + y0) * w + p_coordsCur->x + x0];
	}
	valEven = sum / (double)neighPointNum;
	sum = 0;

	for (p_coordsCur = p_coords; p_coordsCur != p_coordsEnd; ++p_coordsCur)
	{
		x = p_coordsCur->x + x0;
		y = p_coordsCur->y + y0;
		xy = x * y;
		x2 = x * x;
		x3 = x2 * x;
		y2 = y * y;
		y3 = y2 * y;
		x2y = x2 * y;
		xy2 = x * y2;

		val = p_img[(unsigned long long)(y * w + x)];
		val -= valEven;
		valSigma = val * val;

		sum += val;
		sumSigma += valSigma;

		sumX += (val * (double)x);
		sumXSigma += (valSigma * (double)x);

		sumY += (val * (double)y);
		sumYSigma += (valSigma * (double)y);

		sumX2 += (val * (double)x2);
		sumX2Sigma += (valSigma * (double)x2);

		sumY2 += (val * (double)y2);
		sumY2Sigma += (valSigma * (double)y2);

		sumXY += (val * (double)xy);
		sumXYSigma += (valSigma * (double)xy);

		sumX3 += (val * (double)x3);
		sumX3Sigma += (valSigma * (double)x3);

		sumY3 += (val * (double)y3);
		sumY3Sigma += (valSigma * (double)y3);

		sumX2Y += (val * (double)x2y);
		sumX2YSigma += (valSigma * (double)x2y);

		sumXY2 += (val * (double)xy2);
		sumXY2Sigma += (valSigma * (double)xy2);
	}

	double xEven = ((double)sumXSigma) / ((double)sumSigma);
	double yEven = ((double)sumYSigma) / ((double)sumSigma);
	double xEven2 = xEven * xEven;
	double yEven2 = yEven * yEven;
	double xEvenyEven = xEven * yEven;
	double xEven3 = xEven2 * xEven;
	double yEven3 = yEven2 * yEven;
	double xEven2yEven = xEven2 * yEven;
	double xEvenyEven2 = xEven * yEven2;

	double u00 = sum;
	double u00Sigma = sumSigma;
	double u20 = sumX2 - 2 * xEven * sumX + xEven * xEven * u00;
	double u20Sigma = sumX2Sigma - 2 * xEven * sumX2Sigma + xEven2 * u00Sigma;
	double u02 = sumY2 - 2 * yEven * sumY + yEven * yEven * u00;
	double u02Sigma = sumY2Sigma - 2 * yEven * sumY2Sigma + yEven2 * u00Sigma;
	double u11 = sumXY - xEven * sumY - yEven * sumX + xEven * yEven * u00;
	double u11Sigma = sumXYSigma - xEven * sumYSigma - yEven * sumXSigma + xEvenyEven * u00Sigma;
	double u30 = sumX3 - 3 * sumX2 * xEven + 3 * sumX * xEven2 - xEven3 * u00;
	double u30Sigma = sumX3Sigma - 3 * xEven * sumX2Sigma + 3 * xEven2 * sumXSigma - xEven3 * u00Sigma;
	double u03 = sumY3 - 3 * sumY2 * yEven + 3 * sumY * yEven2 - yEven3 * u00;
	double u03Sigma = sumY3Sigma - 3 * yEven * sumY2Sigma + 3 * yEven2 * sumYSigma - yEven3 * u00Sigma;
	double u21 = sumX2Y - 2 * sumXY * xEven + sumY * xEven2 - sumX2 * yEven + 2 * sumX * xEvenyEven - xEven2yEven * u00;
	double u21Sigma = sumX2YSigma - 2 * xEven * sumXYSigma + xEven2 * sumYSigma - yEven * sumX2Sigma + 2 * xEvenyEven * sumXSigma - xEven2yEven * u00Sigma;
	double u12 = sumXY2 - 2 * sumXY * yEven + sumX * yEven2 - sumY2 * xEven + 2 * sumY * xEvenyEven - xEvenyEven2 * u00;
	double u12Sigma = sumXYSigma - 2 * yEven * sumXYSigma + yEven2 * sumXSigma - xEven * sumY2Sigma + 2 * xEvenyEven * sumYSigma - xEvenyEven2 * u00Sigma;

	double m1 = (u20*u02 - u11*u11) / pow(u00Sigma, 1);
	double m2 = (u30*u30*u03*u03 - 6 * u30*u21*u12*u03 + 4 * u30*u12*u12*u12 + 4 * u03*u21*u21*u21 - 3 * u21*u21*u12*u12) / pow(u00Sigma, 2);
	double m3 = (u20*u21*u03 - u20*u12*u12 - u11*u30*u03 + u11*u21*u12 + u02*u30*u12 - u02*u21*u21) / pow(u00Sigma, 1.5);
	double m4 = (-u20*u20*u20*u03*u03 + 6 * u20*u20*u11*u12*u03 - 3 * u20*u20*u02*u12*u12 - 6 * u20*u11*u11*u21*u03 -
		6 * u20*u11*u11*u12*u12 + 12 * u20*u11*u02*u21*u12 - 3 * u20*u02*u02*u21*u21 + 2 * u11*u11*u11*u30*u03 + 6 * u11*u11*u11*u21*u12 - 6 * u11*u11*u02*u30*u12 - 6 * u11*u11*u02*u21*u21 + 6 * u11*u02*u02*u30*u21 - u02*u02*u02*u30*u30) / pow(u00Sigma, 2.5);
	double m1Sigma = (u20Sigma*u02Sigma - u11Sigma*u11Sigma) / pow(u00Sigma, 2);
	double m2Sigma = (u30Sigma*u30Sigma*u03Sigma*u03Sigma - 6 * u30Sigma*u21Sigma*u12Sigma*u03Sigma + 4 * u30Sigma*u12Sigma*u12Sigma*u12Sigma + 4 * u03Sigma*u21Sigma*u21Sigma*u21Sigma - 3 * u21Sigma*u21Sigma*u12Sigma*u12Sigma) / pow(u00Sigma, 4);
	double m3Sigma = (u20Sigma*u21Sigma*u03Sigma - u20Sigma*u12Sigma*u12Sigma - u11Sigma*u30Sigma*u03Sigma + u11Sigma*u21Sigma*u12Sigma + u02Sigma*u30Sigma*u12Sigma - u02Sigma*u21Sigma*u21Sigma) / pow(u00Sigma, 3);
	double m4Sigma = (-u20Sigma*u20Sigma*u20Sigma*u03Sigma*u03Sigma + 6 * u20Sigma*u20Sigma*u11Sigma*u12Sigma*u03Sigma - 3 * u20Sigma*u20Sigma*u02Sigma*u12Sigma*u12Sigma - 6 * u20Sigma*u11Sigma*u11Sigma*u21Sigma*u03Sigma -
		6 * u20Sigma*u11Sigma*u11Sigma*u12Sigma*u12Sigma + 12 * u20Sigma*u11Sigma*u02Sigma*u21Sigma*u12Sigma - 3 * u20Sigma*u02Sigma*u02Sigma*u21Sigma*u21Sigma + 2 * u11Sigma*u11Sigma*u11Sigma*u30Sigma*u03Sigma + 6 * u11Sigma*u11Sigma*u11Sigma*u21Sigma*u12Sigma - 6 * u11Sigma*u11Sigma*u02Sigma*u30Sigma*u12Sigma - 6 * u11Sigma*u11Sigma*u02Sigma*u21Sigma*u21Sigma + 6 * u11Sigma*u02Sigma*u02Sigma*u30Sigma*u21Sigma - u02Sigma*u02Sigma*u02Sigma*u30Sigma*u30Sigma) / pow(u00Sigma, 5);

	p_point->mat.feat[0] = m1;
	p_point->mat.feat[1] = m2;
	p_point->mat.feat[2] = m3;
	p_point->mat.feat[3] = m4;
	p_point->mat.feat[4] = m1Sigma;
	p_point->mat.feat[5] = m2Sigma;
	p_point->mat.feat[6] = m3Sigma;
	p_point->mat.feat[7] = m4Sigma; 
}

// calculate the features of all the located interest points
void getPointsFeats(InterestPoint* p_points, int pointNum, const unsigned char *p_img, short r, short w)
{
	if (p_points == NULL || p_img == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of points set pointer");
		exit(-1);
	}

	Coord* p_coords = (Coord*)calloc_check(4 * r * r, sizeof(Coord));
	unsigned neighPointNum = calcDiskTmplArray(p_coords, r);

	// get the points feat
	InterestPoint* p_pointsCur = p_points;
	InterestPoint* p_pointsEnd = p_points + pointNum;
	unsigned int curPointSeq = 1;
	for (; p_pointsCur != p_pointsEnd; ++p_pointsCur, ++curPointSeq)
	{
		calcFeat(p_pointsCur, p_img, p_coords, neighPointNum, w);

		//DEBUG_PRINT_SIMPLIFIED("Point%4d: (%4d, %4d), Feat: (", curPointSeq, p_pointsCur->c.x, p_pointsCur->c.y);
		//for (int f = 0; f < FEAT_NUM; ++f)
		//{
			DEBUG_PRINT_SIMPLIFIED("%lf ", p_pointsCur->mat.feat[0]);
		//}
		//DEBUG_PRINT_SIMPLIFIED(")\n");
	    DEBUG_PRINT_SIMPLIFIED("\n");
	}

	free(p_coords);
	p_coords = NULL;
}


// get the most similar(nearest) point of current point
const InterestPoint* getNearestPoint(const InterestPoint *p_pointCur, const InterestPoint *p_pointsRef, int pointNumRef, double minDist)
{
	if (p_pointsRef == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		exit(-1);
	}

	if (p_pointCur == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		exit(-1);
	}

	const InterestPoint* p_pointsRefNearest = NULL;
	const InterestPoint* p_pointsRefEnd = p_pointsRef + pointNumRef;

	for (; p_pointsRef != p_pointsRefEnd; ++p_pointsRef)
	{
		double dist = 0.0;

#ifdef _EURO_DIST_
		dist = getEuroDist2((const double*)(&p_pointCur->mat.feat[0]), (const double*)&p_pointsRef->mat.feat[0], FEAT_NUM);
#endif
#ifdef _MAHA_DIST_
		calcMahaDistance2((const double*)(&p_pointCur->mat.feat[0]), (const double*)&p_pointsRef->mat.feat[0], matCovarInv, FEAT_NUM, &dist);
#endif
#ifdef _ANGLE_DIST_
		dist = getAngleDist((const double*)(&p_pointCur->mat.feat[0]), (const double*)&p_pointsRef->mat.feat[0], FEAT_NUM);
#endif

		if (dist < minDist)
		{
			p_pointsRefNearest = p_pointsRef;
			minDist = dist;
		}
	}

	return p_pointsRefNearest;
}

// normalize all the feats
void normalizePointsFeats(InterestPoint* p_pointsL, int pointNumL, InterestPoint* p_pointsR, int pointNumR)
{
	// get the max and min value of different channel of feats
	featElemType* p_featMax = (featElemType*)calloc_check(FEAT_NUM, sizeof(featElemType));
	featElemType* p_featMin = (featElemType*)calloc_check(FEAT_NUM, sizeof(featElemType));
	featElemType* p_featGap = (featElemType*)calloc_check(FEAT_NUM, sizeof(featElemType));

	featElemType* p_featMaxCur = p_featMax;
	featElemType* p_featMinCur = p_featMin;
	const featElemType* p_featMinEnd = p_featMin + FEAT_NUM;
	for (; p_featMinCur != p_featMinEnd; ++p_featMaxCur, ++p_featMinCur)
	{
		*p_featMaxCur = FEAT_MIN;
		*p_featMinCur = FEAT_MAX;
	}
	featElemType* p_featGapCur = p_featGap;

	InterestPoint* p_pointsLCur = p_pointsL;
	const InterestPoint* p_pointsLEnd = p_pointsL + pointNumL;
	InterestPoint* p_pointsRCur = p_pointsR;
	const InterestPoint* p_pointsREnd = p_pointsR + pointNumR;

	// get the max and min value of different channel of feats
	unsigned short f = 0;

	for (p_pointsLCur = p_pointsL; p_pointsLCur != p_pointsLEnd; ++p_pointsLCur)
	{
		for (p_featMinCur = p_featMin, p_featMaxCur = p_featMax, f = 0; p_featMinCur != p_featMinEnd; ++p_featMaxCur, ++p_featMinCur, ++f)
		{
			featElemType val = p_pointsLCur->mat.feat[f];

			if (*p_featMaxCur < val)
				*p_featMaxCur = val;
			if (*p_featMinCur > val)
				*p_featMinCur = val;
		}
	}

	for (p_pointsRCur = p_pointsR; p_pointsRCur != p_pointsREnd; ++p_pointsRCur)
	{
		for (p_featMinCur = p_featMin, p_featMaxCur = p_featMax, f = 0; p_featMinCur != p_featMinEnd; ++p_featMaxCur, ++p_featMinCur, ++f)
		{
			featElemType val = p_pointsRCur->mat.feat[f];

			if (*p_featMaxCur < val)
				*p_featMaxCur = val;
			if (*p_featMinCur > val)
				*p_featMinCur = val;
		}
	}

	for (p_featMinCur = p_featMin, p_featMaxCur = p_featMax, p_featGapCur = p_featGap; p_featMinCur != p_featMinEnd; ++p_featMaxCur, ++p_featMinCur, ++p_featGapCur)
	{
		*p_featGapCur = *p_featMaxCur - *p_featMinCur;
	}

	// normalize the feats
	unsigned int curPointSeq = 1;
	for (curPointSeq = 1, p_pointsLCur = p_pointsL; p_pointsLCur != p_pointsLEnd; ++p_pointsLCur, ++curPointSeq)
	{
		p_featMinCur = p_featMin;
		p_featGapCur = p_featGap;

		for (f = 0; p_featMinCur != p_featMinEnd; ++p_featMinCur, ++p_featGapCur, ++f)
		{
			p_pointsLCur->mat.feat[f] = (p_pointsLCur->mat.feat[f] - *p_featMinCur) / (*p_featGapCur);
		}

		DEBUG_PRINT_SIMPLIFIED("L Point%4d: (%4d, %4d), Feat: (", curPointSeq, p_pointsLCur->c.x, p_pointsLCur->c.y);
		for (int f = 0; f < FEAT_NUM; ++f)
		{
			DEBUG_PRINT_SIMPLIFIED("%llf ", p_pointsLCur->mat.feat[f]);
		}
		DEBUG_PRINT_SIMPLIFIED(")\n");
	}
	for (curPointSeq = 1, p_pointsRCur = p_pointsR; p_pointsRCur != p_pointsREnd; ++p_pointsRCur, ++curPointSeq)
	{
		p_featMinCur = p_featMin;
		p_featGapCur = p_featGap;

		for (f = 0; p_featMinCur != p_featMinEnd; ++p_featMinCur, ++p_featGapCur, ++f)
		{
			p_pointsRCur->mat.feat[f] = (p_pointsRCur->mat.feat[f] - *p_featMinCur) / (*p_featGapCur);
		}

		DEBUG_PRINT_SIMPLIFIED("R Point%4d: (%4d, %4d), Feat: (", curPointSeq, p_pointsRCur->c.x, p_pointsRCur->c.y);
		for (int f = 0; f < FEAT_NUM; ++f)
		{
			DEBUG_PRINT_SIMPLIFIED("%llf ", p_pointsRCur->mat.feat[f]);
		}
		DEBUG_PRINT_SIMPLIFIED(")\n");
	}
}

// rough match based on mutual-minimum-distance
// returns the matched pair number
int roughMatch(const InterestPoint* p_pointsL, int pointNumL, const InterestPoint* p_pointsR, int pointNumR, PointPair* p_pairs)
{
	if (p_pointsL == NULL || p_pointsR == NULL || p_pairs == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		exit(-1);
	}

	PointPair* p_pairsCur = p_pairs;

	const InterestPoint* p_pointsLCur = p_pointsL;
	const InterestPoint* p_pointsLStart = p_pointsL;
	const InterestPoint* p_pointsLEnd = p_pointsL + pointNumL;

	const InterestPoint* p_pointsRStart = p_pointsR;

	double minDist = FEAT_MAX;

	for (; p_pointsLCur != p_pointsLEnd; ++p_pointsLCur)
	{
		const InterestPoint* p_matchedR = getNearestPoint(p_pointsLCur, p_pointsRStart, pointNumR, minDist);
		if (p_matchedR == NULL)
		{
			DEBUG_PRINT_DETAILED("Left image Point %ld has no nearest Right image point", p_pointsLCur - p_pointsLStart + 1);
		}
		else
		{
			const InterestPoint* p_matchedL = getNearestPoint(p_matchedR, p_pointsLStart, pointNumL, minDist);
			if (p_matchedL == NULL)
			{
				DEBUG_PRINT_DETAILED("Right image Point %ld has no nearest Left image point", p_matchedR - p_pointsRStart + 1);
			}
			else
			{
				if (p_matchedL == p_pointsLCur)
				{
					p_pairsCur->pL = p_matchedL->c;
					p_pairsCur->pR = p_matchedR->c;
					p_pairsCur++;
				}
			}
		}
   }

	return (p_pairsCur - p_pairs);
}


// using ransac to get the best projection matrix based on the coarse matching pairs
ProjectMat getProjMatByRansac(const PointPair* p_pairs, int pairNum, double distThresh, short wL, short hL, short wR, short hR)
{
	if (p_pairs == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		exit(-1);
	}

	// threshold for ending the iteration
	// > innerPointNumThresh -- correct enough projection matrix coefficiency has been found
	// < innerPointNumThresh -- not yet
	unsigned int maxInnerPointNum = 0;

	// projection matrix coefficiency
	ProjectMat curMat, suitMat;

	unsigned int iterateNum = 100 * pairNum;
	double** matSrc = NULL;
	mallocMat(&matSrc, 3, 3);
	double** matDst = NULL;
	mallocMat(&matDst, 3, 3);
	double** matT = NULL;
	mallocMat(&matT, 3, 3);

	for (unsigned int i = 0; i < iterateNum; ++i)
	{
		unsigned int start1 = rand() % pairNum;
		unsigned int start2 = rand() % pairNum;
		unsigned int start3 = rand() % pairNum;
		const PointPair* p_pair1 = p_pairs + start1;
		const PointPair* p_pair2 = p_pairs + start2;
		const PointPair* p_pair3 = p_pairs + start3;

		// get the affine transformation
		matSrc[0][0] = p_pair1->pL.x;
		matSrc[0][1] = p_pair2->pL.x;
		matSrc[0][2] = p_pair3->pL.x;
		matSrc[1][0] = p_pair1->pL.y;
		matSrc[1][1] = p_pair2->pL.y;
		matSrc[1][2] = p_pair3->pL.y;
		matSrc[2][0] = 1;
		matSrc[2][1] = 1;
		matSrc[2][2] = 1;

		matDst[0][0] = p_pair1->pR.x;
		matDst[0][1] = p_pair2->pR.x;
		matDst[0][2] = p_pair3->pR.x;
		matDst[1][0] = p_pair1->pR.y;
		matDst[1][1] = p_pair2->pR.y;
		matDst[1][2] = p_pair3->pR.y;
		matDst[2][0] = 1;
		matDst[2][1] = 1;
		matDst[2][2] = 1;

		if (calcMatTransformation(matSrc, matDst, matT, 3) < 0)
			continue;

		curMat.m1 = matT[0][0];
		curMat.m2 = matT[0][1];
		curMat.m3 = matT[0][2];
		curMat.m4 = matT[1][0];
		curMat.m5 = matT[1][1];
		curMat.m6 = matT[1][2];

		// calculate the inner point number under current coefficients
		const PointPair* p_pairCur = p_pairs;
		const PointPair* p_pairEnd = p_pairs + pairNum;
		unsigned int innerPointNum = 0;

		for (; p_pairCur != p_pairEnd; ++p_pairCur)
		{
			double expectedRx = curMat.m1 * (double)p_pairCur->pL.x + curMat.m2 * (double)p_pairCur->pL.y + curMat.m3;
			double expectedRy = curMat.m4 * (double)p_pairCur->pL.x + curMat.m5 * (double)p_pairCur->pL.y + curMat.m6;
			double dist = pow((expectedRx - p_pairCur->pR.x), 2) + pow((expectedRy - p_pairCur->pR.y), 2);
			if (dist < distThresh)
			{
				++innerPointNum;
			}
		}

		// only record the coefficients that generates maximum inner point num
		if (innerPointNum > maxInnerPointNum)
		{
			maxInnerPointNum = innerPointNum;
			suitMat = curMat;
		}
	}

	free(matSrc);
	matSrc = NULL;
	free(matDst);
	matDst = NULL;
	free(matT);
	matT = NULL;

	cout << "maxInnerPointNum: " << maxInnerPointNum << endl;

	return suitMat;
}


// match the interest points of two images
// returns the matched pairs of interest points
ProjectMat matchInterestPoints(InterestPoint *p_pointsL, int pointNumL, InterestPoint *p_pointsR, int pointNumR, PointPair* p_pairs, int* p_pairNum, double thresh, short wL, short hL, short wR, short hR)
{
	if (p_pointsL == NULL || p_pointsR == NULL || p_pairNum == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		exit(-1);
	}

	ProjectMat mat;

	double** matCovarInv = NULL;
	double** matXs = NULL;
	double** matCovar = NULL;

	int pointTotalNum = pointNumL + pointNumR;
	if (callocMat(&matXs, pointTotalNum, FEAT_NUM) < 0)
		exit(-1);
	if (callocMat(&matCovar, FEAT_NUM, FEAT_NUM) < 0)
		exit(-1);
	if (callocMat(&matCovarInv, FEAT_NUM, FEAT_NUM) < 0)
		exit(-1);

	// step 0: normalize all these feats
	normalizePointsFeats(p_pointsL, pointNumL, p_pointsR, pointNumR);

#ifdef _MAHA_DIST_
	for (int p = 0; p < pointNumL; ++p)
	{
		for (int f = 0; f < FEAT_NUM; ++f)
		{
			matXs[p][f] = p_pointsL[p].mat.feat[f];
		}
	}
	for (int p = pointNumL; p < pointTotalNum; ++p)
	{
		for (int f = 0; f < FEAT_NUM; ++f)
		{
			matXs[p][f] = p_pointsR[p].mat.feat[f];
		}
	}

	if (calcMatCovar(matXs, matCovar, FEAT_NUM, pointTotalNum) < 0)
		exit(-1);

	time_t start = clock();
	if (inverseMat(matCovar, matCovarInv, FEAT_NUM) < 0)
		exit(-1);
	time_t end = clock();
	cout << "time for calculating mat covar: " << end - start << endl;
#endif

	// step 1: rough match based on mutual-minimum-distance
	*p_pairNum = roughMatch(p_pointsL, pointNumL, p_pointsR, pointNumR, p_pairs);
	cout << "rought matched pair number: " << *p_pairNum << endl;

#ifndef _ONLY_ROUGH_
	// step 2: get the projection match based on ransac
	//clock_t start = clock();
	mat = getProjMatByRansac(p_pairs, *p_pairNum, thresh, wL, hL, wR, hR);
	//clock_t end = clock();
	//cout << "ransac time: " << end - start << endl;
#endif


	free(matXs);
	matXs = NULL;
	free(matCovar);
	matCovar = NULL;
	free(matCovarInv);
	matCovarInv = NULL;

	return mat;
}


// draw the link of matched points of two images
void showMatchResult(const cv::Mat& matL, const cv::Mat& matR, const ProjectMat& realMat, const ProjectMat& suitMat, const InterestPoint* p_pointsL, int pointNumL, const PointPair* p_pairs, int pairNum, double dThresh, short step)
{
	if (p_pairs == NULL)
	{
		DEBUG_PRINT_DETAILED("NULL input of pointers");
		exit(-1);
	}

	// merge the input images into one image
	cv::Mat_<cv::Vec3b> matArr[2] = { matL, matR };
	cv::Mat_<cv::Vec3b> mergedMat = mergeMats(matArr, 2, horizontal);

	unsigned short wL = matL.cols;

#ifndef _ONLY_ROUGH_
	const PointPair* p_pairsEnd = p_pairs + pairNum;
	const PointPair* p_pairsCur = p_pairs;
	for (p_pairsCur = p_pairs; p_pairsCur < p_pairsEnd; p_pairsCur += step)
	{
		if (p_pairsCur >= p_pairsEnd)
			break;

		double xL = p_pairsCur->pL.x;
		double yL = p_pairsCur->pL.y;
		double xR = xL * suitMat.m1 + yL * suitMat.m2 + suitMat.m3 + wL;
		double yR = xL * suitMat.m4 + yL * suitMat.m5 + suitMat.m6;

		cv::line(mergedMat, cv::Point(xL, yL), cv::Point(xR, yR), cv::Scalar(255, 255, 255), 1);
	}
#else
	const PointPair* p_pairsEnd = p_pairs + pairNum;
	const PointPair* p_pairsCur = p_pairs;
	for (p_pairsCur = p_pairs; p_pairsCur < p_pairsEnd; p_pairsCur += step)
	{
		if (p_pairsCur >= p_pairsEnd)
			break;

		double xL = p_pairsCur->pL.x;
		double yL = p_pairsCur->pL.y;
		double xR = p_pairsCur->pR.x + wL;
		double yR = p_pairsCur->pR.y;

		cv::line(mergedMat, cv::Point(xL, yL), cv::Point(xR, yR), cv::Scalar(255, 255, 255), 1);
	}
#endif

#ifdef _MODULATE_DATA_
	// show affine mat
	const InterestPoint* p_pointsLEnd = p_pointsL + pointNumL;
	const InterestPoint* p_pointsLCur = p_pointsL;
	int matchedNum = 0;
	for (; p_pointsLCur != p_pointsLEnd; ++p_pointsLCur)
	{
		double xReal = (double)p_pointsLCur->c.x * realMat.m1 + (double)p_pointsLCur->c.y * realMat.m2 + realMat.m3;
		double yReal = (double)p_pointsLCur->c.x * realMat.m4 + (double)p_pointsLCur->c.y * realMat.m5 + realMat.m6;

		double xCalc = (double)p_pointsLCur->c.x * suitMat.m1 + (double)p_pointsLCur->c.y * suitMat.m2 + suitMat.m3;
		double yCalc = (double)p_pointsLCur->c.x * suitMat.m4 + (double)p_pointsLCur->c.y * suitMat.m5 + suitMat.m6;

		double dist = pow((xReal - xCalc), 2) + pow((yReal - yCalc), 2);
		if (dist <= dThresh)
			matchedNum++;
	}
	cout << "realMat: " << endl;
	cout << realMat.m1 << "\t" << realMat.m2 << "\t" << realMat.m3 << endl;
	cout << realMat.m4 << "\t" << realMat.m5 << "\t" << realMat.m6 << endl;
	cout << "suitMat: " << endl;
	cout << suitMat.m1 << "\t" << suitMat.m2 << "\t" << suitMat.m3 << endl;
	cout << suitMat.m4 << "\t" << suitMat.m5 << "\t" << suitMat.m6 << endl;
	cout << "Matched pairs number: " << pairNum << endl;
	cout << "matchedNum: " << matchedNum << "\n" << "precision rate: " << (double)matchedNum / (double)pointNumL << endl;
#endif

	cv::imshow("merged initial images", mergedMat);
	cv::imwrite("C:\\Users\\ÅÊ\\Desktop\\´óÂÛÎÄ\\" + string("yidong50") + string(".png"), mergedMat);
}

// draw the rectangle around the interest point
void drawRect(cv::Mat& mat, const InterestPoint *p_points, int pointNum, short step, cv::Scalar color)
{
	if (p_points == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of image pointer or interest points pointer");
		exit(-1);
	}

	const InterestPoint* p_pointsCur = p_points;
	const InterestPoint* p_pointsEnd = p_points + pointNum;
	for (; p_pointsCur < p_pointsEnd; p_pointsCur += step)
	{
		cv::Point center(p_pointsCur->c.x, p_pointsCur->c.y);
		//cv::circle(mat, center, p_pointsCur->r, color);
		cv::circle(mat, center, 2, color);
	}
}
































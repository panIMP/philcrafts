#include "pointLocate.h"
#include "error.h"


const Filter g_filters[] =
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
};

const unsigned char g_prevFilterSeqOffset[] = { 0, 1, 1, 1, 1, 1, 1, 0 };
const unsigned char g_nextFilterSeqOffset[] = { 0, 1, 1, 2, 1, 2, 1, 0 };
const double g_sigma[] = { 1.2, 2, 2.8, 3.6, 5.2 };


// draw the rectangle around the interest point
int drawRect
(
unsigned char* imgPtr, 
const InterestPoint* pointPtr, 
int pointNum,
unsigned char pixVal, 
short r,
short w
)
{
	unsigned short p;
	unsigned int y, x;

	if (imgPtr == NULL || pointPtr == NULL)
		return -1;

	for (p = 0; p < pointNum; ++p)
	{
		for (y = pointPtr[p].c.y - r / 2; y <= pointPtr[p].c.y + r / 2; ++y)
		{
			for (x = pointPtr[p].c.x - r / 2; x <= pointPtr[p].c.x + r / 2; ++x)
			{
				imgPtr[y * w + x] = pixVal;
			}
		}
	}

	return 0;
}


// init the pointers of the four corners of each box of certain filter
int initFiltPtrs
(
const int *integImgPtr, 
FilterPtrs* filterPtrsPtr, 
const Filter* filterPtr, 
short w
)
{
	int m = 0;
	int b = 0;

	if (integImgPtr == NULL || filterPtrsPtr == NULL || filterPtr == NULL)
		return -1;

	for (m = 0; m < MODE_NUM; ++m)
	{
		int boxNum = filterPtr->mode[m].boxNum;
		filterPtrsPtr->mptrs[m].boxNum = boxNum;

		for (b = 0; b < boxNum; ++b)
		{
			filterPtrsPtr->mptrs[m].bptrs[b].p_boxTL = integImgPtr 
				+ w * filterPtr->mode[m].box[b].tLY 
				+ filterPtr->mode[m].box[b].tLX;
			filterPtrsPtr->mptrs[m].bptrs[b].p_boxTR = integImgPtr
				+ w * filterPtr->mode[m].box[b].tLY 
				+ filterPtr->mode[m].box[b].bRX + 1;
			filterPtrsPtr->mptrs[m].bptrs[b].p_boxBL = integImgPtr 
				+ w * (filterPtr->mode[m].box[b].bRY + 1) 
				+ filterPtr->mode[m].box[b].tLX;
			filterPtrsPtr->mptrs[m].bptrs[b].p_boxBR = integImgPtr
				+ w * (filterPtr->mode[m].box[b].bRY + 1) 
				+ filterPtr->mode[m].box[b].bRX + 1;
		}
	}

	return 0;
}


// increase the pointers of the four corners of each box of certain filter by certain value
int incFilterPtrs(FilterPtrs* filterPtrsPtr, short val)
{
	int m = 0;
	int b = 0;

	if (filterPtrsPtr == NULL)
		return -1;

	for (m = 0; m < MODE_NUM; ++m)
	{
		int boxNum = filterPtrsPtr->mptrs[m].boxNum;

		for (b = 0; b < boxNum; ++b)
		{
			filterPtrsPtr->mptrs[m].bptrs[b].p_boxTL += val;
			filterPtrsPtr->mptrs[m].bptrs[b].p_boxTR += val;
			filterPtrsPtr->mptrs[m].bptrs[b].p_boxBL += val;
			filterPtrsPtr->mptrs[m].bptrs[b].p_boxBR += val;
		}
	}

	return 0;
}

// calculate the det(Hessian) response of one pixel
int calcDetHes
(
const FilterPtrs *filterPtrsPtr, 
const Filter* filterPtr, 
hesMat* detHesImgPtr, 
char layerSeq
)
{
	int deriVal[MODE_NUM] = { 0 };

	short m = 0;
	short b = 0;

	if (filterPtrsPtr == NULL || detHesImgPtr == NULL)
		return -1;

	for (m = 0; m < MODE_NUM; ++m)
	{
		unsigned short boxNum = filterPtrsPtr->mptrs[m].boxNum;
		int val = 0;

		for (b = 0; b < boxNum; ++b)
		{
			val = *(filterPtrsPtr->mptrs[m].bptrs[b].p_boxBR) 
				+ *(filterPtrsPtr->mptrs[m].bptrs[b].p_boxTL) 
				- *(filterPtrsPtr->mptrs[m].bptrs[b].p_boxTR) 
				- *(filterPtrsPtr->mptrs[m].bptrs[b].p_boxBL);

			deriVal[m] += val * (filterPtr->mode[m].box[b].wei) 
				/ (filterPtr->mode[m].box[b].s);
		}
	}

	detHesImgPtr->val[layerSeq] = deriVal[0] * deriVal[1] - deriVal[2] * deriVal[2];

	return 0;
}

// create the det(Hessian) image of the input integral image at one specific octave and layer
int createDetHesImg
(
const int* integImgPtr, 
hesMat* detHesImgPtr, 
const unsigned char* markImgPtr, 
short layOrder, 
short w, 
short h
)
{
	// pointer to filter of current layer
	const Filter* filterPtr = &g_filters[layOrder];

	// coordinates of the starting processing pixel of current layer 
	// must move to the next diagonal pixel to prevent boundary except
	short stX = (filterPtr->mode[0].box[1].tLX + filterPtr->mode[0].box[1].bRX) / 2 + 1;
	short stY = (filterPtr->mode[0].box[1].tLY + filterPtr->mode[0].box[1].bRY) / 2 + 1;
	short enX = w - stX;
	short enY = h - stY;
	short y = 0;
	short x = 0;

	// init pointers of the filter box corners
	FilterPtrs filterPtrs;
	initFiltPtrs(integImgPtr, &filterPtrs, filterPtr, w);

	hesMat* t_detHesImgPtr;
	FilterPtrs t_filterPtrs;

	if (integImgPtr == NULL || detHesImgPtr == NULL)
		return -1;

	for (y = stY; y < enY; ++y, incFilterPtrs(&filterPtrs, w))
	{
		t_detHesImgPtr = detHesImgPtr + y * w + stX;
		t_filterPtrs = filterPtrs;

		for (x = stX; x < enX; ++x, ++t_detHesImgPtr, incFilterPtrs(&t_filterPtrs, 1))
		{
			if (markImgPtr[y * w + x] != FORE_GROUND_PIXEL)
				continue;

			calcDetHes(&t_filterPtrs, filterPtr, t_detHesImgPtr, layOrder);
		}
	}

	return 0;
}


// create the Det(Hessian) image pyramid of certain number of octaves and layers
int createDetHesImgPyr
(
hesMat *detHesImgPtr, 
const int *integImgPtr, 
const unsigned char* markImgPtr, 
short layNum,
short w,
short h
)
{
	int totalSize = w * h;
	short layOrder = 0;

	if (integImgPtr == NULL || detHesImgPtr == NULL || markImgPtr == NULL)
		return -1;

	// create pyramid
	for (layOrder = 0; layOrder < layNum; ++layOrder)
	{
		createDetHesImg(integImgPtr, detHesImgPtr, markImgPtr, layOrder, w, h);
	}

	return 0;
}


// find the interest point
// returns 0: current pixel is not the regional maximum point
// returns 1: current pixel is the regional maximum point
int isRegionMaximum(const hesMat* imgPtr, short w, char layOrder)
{
	int i00, i01, i02;
	int i10, i11, i12;
	int i20, i21, i22;

	char curLayer = layOrder;
	int val = imgPtr->val[layOrder];

	if (imgPtr == NULL)
		return -1;

	// current layer of image
	i00 = (imgPtr - w - 1)->val[curLayer];		
	i01 = (imgPtr - w)->val[curLayer];		
	i02 = (imgPtr - w + 1)->val[curLayer];
	i10 = (imgPtr - 1)->val[curLayer];			
	i11 = (imgPtr)->val[curLayer];			
	i12 = (imgPtr + 1)->val[curLayer];
	i20 = (imgPtr + w - 1)->val[curLayer];		
	i21 = (imgPtr + w)->val[curLayer];		
	i22 = (imgPtr + w + 1)->val[curLayer];

	if (i11 <= i00 || i11 <= i01 || i11 <= i02
		|| i11 <= i10 || i11 <= i12
		|| i11 <= i20 || i11 <= i21 || i11 <= i22)
		return 0;

	// upper layer of image
	curLayer = layOrder + g_prevFilterSeqOffset[layOrder];

	i00 = (imgPtr - w - 1)->val[curLayer];		
	i01 = (imgPtr - w)->val[curLayer];		
	i02 = (imgPtr - w + 1)->val[curLayer];
	i10 = (imgPtr - 1)->val[curLayer];		
	i11 = (imgPtr)->val[curLayer];			
	i12 = (imgPtr + 1)->val[curLayer];
	i20 = (imgPtr + w - 1)->val[curLayer];		
	i21 = (imgPtr + w)->val[curLayer];	
	i22 = (imgPtr + w + 1)->val[curLayer];

	if (val <= i00 || val <= i01 || val <= i02
		|| val <= i10 || val <= i11 || val <= i12
		|| val <= i20 || val <= i21 || val <= i22)
		return 0;

	// lower layer of image
	curLayer = layOrder - g_nextFilterSeqOffset[layOrder];

	i00 = (imgPtr - w - 1)->val[curLayer];		
	i01 = (imgPtr - w)->val[curLayer];		
	i02 = (imgPtr - w + 1)->val[curLayer];
	i10 = (imgPtr - 1)->val[curLayer];		
	i11 = (imgPtr)->val[curLayer];		
	i12 = (imgPtr + 1)->val[curLayer];
	i20 = (imgPtr + w - 1)->val[curLayer];	
	i21 = (imgPtr + w)->val[curLayer];	
	i22 = (imgPtr + w + 1)->val[curLayer];

	if (val <= i00 || val <= i01 || val <= i02
		|| val <= i10 || val <= i11 || val <= i12
		|| val <= i20 || val <= i21 || val <= i22)
		return 0;

	// current pixel is the maximum of 26 neighbor pixels
	return 1;
}


// get the interest points location
// returns the number of the founded interest points
int getPointLocation
(
InterestPoint* pointPtr,
unsigned char* marImgPtr,
const unsigned char* imgPtr,
const hesMat* detHesImgPtr,
short layNum, 
double detHesThresh, 
short w,
short h
)
{
	unsigned int totalSize = w * h;
	unsigned int filtSize = w * (h - 1) - 1;
	unsigned int pointNum = 0;

	// former implementation of finding interest point locations
	const hesMat* p_detHesImgCur = detHesImgPtr + w + 1;

	double maxVal = FEAT_MIN;
	double curVal = 0;
	unsigned char maxLayOrder = 0;
	int c = 0;
	short layOrder = 0;

	if (pointPtr == NULL || detHesImgPtr == NULL)
		return -1;

	for (c = w + 1; c != filtSize; ++c, ++p_detHesImgCur)
	{
		if (marImgPtr[c] != FORE_GROUND_PIXEL)
			continue;

		// get the max response val and its layer order
		maxVal = FEAT_MIN;
		for (layOrder = 1; layOrder < LAYER_NUM - 1; ++layOrder)
		{
			curVal = p_detHesImgCur->val[layOrder];

			if (curVal > maxVal)
			{
				maxVal = curVal;
				maxLayOrder = layOrder;
			}
		}

		// if the max response is still less than the detHes threshold
		if (maxVal < detHesThresh)
			continue;

		// if point in the maxLayorder is the regional maximum  
		if (isRegionMaximum(p_detHesImgCur, w, maxLayOrder))
		{
			short y = c / w;
			short x = c - y * w;
			//short r = g_sigma[maxLayOrder] * 6;  // ignore! all the descriptor uses the same r
			short r = 12;

			// boundary check
			if (x - r < 0 || x + r >= w || y - r < 0 || y + r >= h)
				continue;

			// mark the interest points first
			marImgPtr[c] = MARKED_TRUE;

			// if current pixel is maximum, then its neighbor are not maximum
			marImgPtr[c + 1] = MARKED_FALSE;
			marImgPtr[c + w] = MARKED_FALSE;
			marImgPtr[c + w + 1] = MARKED_FALSE;

			// store the interest point
			pointPtr->c.y = y;
			pointPtr->c.x = x;
			
			//pointPtr->r = r; // ignore! all the descriptor uses the same r

			// store pointer increase
			++pointPtr;
			// store element increase
			++pointNum;

			// increase image pointer and index
			++c;
			++p_detHesImgCur;
		}
	}

	return pointNum;
}


// calculate the feature of one interest point
int calcFeat
(
InterestPoint* pointPtr,
const unsigned char* imgPtr, 
const Coord* coordPtr,
int neighPointNum, 
short w
)
{
	short x0 = pointPtr->c.x;
	short y0 = pointPtr->c.y;

	int x, y, x2, y2, xy, x3, y3, x2y, xy2;

	double val, valEven, valSigma;
	double sum = 0;
	double sumSigma = 0;
	double sumX = 0;
	double sumXSigma = 0;
	double sumY = 0;
	double sumYSigma = 0;
	double sumX2 = 0;
	double sumY2 = 0;
	double sumXY = 0;
	double sumX3 = 0;
	double sumY3 = 0;
	double sumX2Y = 0;
	double sumXY2 = 0;

	const Coord* coordPtrCur = coordPtr;
	const Coord* coordPtrEnd = coordPtr + neighPointNum;

	double xEven = 0;
	double yEven = 0;
	double xEven2 = 0;
	double yEven2 = 0;
	double xEvenyEven = 0;
	double xEven3 = 0;
	double yEven3 = 0;
	double xEven2yEven = 0;
	double xEvenyEven2 = 0;

	double u00 = 0;
	double u00Sigma = 0;
	double u20 = 0;
	double u02 = 0;
	double u11 = 0;
	double u30 = 0;
	double u03 = 0;
	double u21 = 0;
	double u12 = 0;

	double m1, m2, m3, m4;

	short leftMoveStep = 10;

	if (pointPtr == NULL || imgPtr == NULL)
		return -1;

	for (coordPtrCur = coordPtr; coordPtrCur != coordPtrEnd; ++coordPtrCur)
	{
		sum += imgPtr[(coordPtrCur->y + y0) * w + coordPtrCur->x + x0];
	}
	valEven = (double)sum / (double)neighPointNum; 
	sum = 0;

	for (coordPtrCur = coordPtr; coordPtrCur != coordPtrEnd; ++coordPtrCur)
	{
		x = coordPtrCur->x + x0;
		y = coordPtrCur->y + y0;
		xy = x * y;
		x2 = x * x;
		x3 = x2 * x;
		y2 = y * y;
		y3 = y2 * y;
		x2y = x2 * y;
		xy2 = x * y2;

		val = imgPtr[(y * w + x)];
		val = val - valEven;
		valSigma = val * val;

		sum += val;
		sumSigma += valSigma;

		sumX += (val * x);
		sumXSigma += (valSigma * x);

		sumY += (val * y);
		sumYSigma += (valSigma * y);

		sumX2 += (val * x2);

		sumY2 += (val * y2);

		sumXY += (val * xy);

		sumX3 += (val * x3);

		sumY3 += (val * y3);

		sumX2Y += (val * x2y);

		sumXY2 += (val * xy2);
	}

	xEven = sumXSigma / sumSigma;
	yEven = sumYSigma / sumSigma;
	xEven2 = xEven * xEven;
	yEven2 = yEven * yEven;
	xEvenyEven = xEven * yEven;
	xEven3 = xEven2 * xEven;
	yEven3 = yEven2 * yEven;
	xEven2yEven = xEven2 * yEven;
	xEvenyEven2 = xEven * yEven2;

	u00 = sum;
	u00Sigma = sumSigma;
	u20 = sumX2 - 2 * xEven * sumX + xEven * xEven * sum;
	u02 = sumY2 - 2 * yEven * sumY + yEven * yEven * sum;
	u11 = sumXY - xEven * sumY - yEven * sumX + xEven * yEven * sum;
	u30 = sumX3 - 3 * sumX2 * xEven + 3 * sumX * xEven2 - xEven3 * sum;
	u03 = sumY3 - 3 * sumY2 * yEven + 3 * sumY * yEven2 - yEven3 * sum;
	u21 = sumX2Y - 2 * sumXY * xEven + sumY * xEven2 - sumX2 * yEven 
		+ 2 * sumX * xEvenyEven - xEven2yEven * sum;
	u12 = sumXY2 - 2 * sumXY * yEven + sumX * yEven2 - sumY2 * xEven 
		+ 2 * sumY * xEvenyEven - xEvenyEven2 * sum;

	m1 = (u20*u02 - u11*u11) / pow(u00Sigma, 1);
	m2 = (u30*u30*u03*u03 - 6 * u30*u21*u12*u03 + 4 * u30*u12*u12*u12
		+ 4 * u03*u21*u21*u21 - 3 * u21*u21*u12*u12)
		/ pow(u00Sigma, 2);
	m3 = (u20*u21*u03 - u20*u12*u12 - u11*u30*u03 + u11*u21*u12 
		+ u02*u30*u12 - u02*u21*u21) / pow(u00Sigma, 1.5);
	m4 = (-u20*u20*u20*u03*u03 + 6 * u20*u20*u11*u12*u03 
		- 3 * u20*u20*u02*u12*u12 - 6 * u20*u11*u11*u21*u03 
		-6 * u20*u11*u11*u12*u12 + 12 * u20*u11*u02*u21*u12 
		- 3 * u20*u02*u02*u21*u21 + 2 * u11*u11*u11*u30*u03 
		+ 6 * u11*u11*u11*u21*u12 - 6 * u11*u11*u02*u30*u12 
		- 6 * u11*u11*u02*u21*u21 + 6 * u11*u02*u02*u30*u21 
		- u02*u02*u02*u30*u30) / pow(u00Sigma, 2.5);

	pointPtr->mat.feat[0] = m1;
	pointPtr->mat.feat[1] = m2;
	pointPtr->mat.feat[2] = m3;
	pointPtr->mat.feat[3] = m4;

	return 0;
}


// calculate the features of all the located interest points
int getPointsFeats
(
InterestPoint* pointPtr,
int pointNum,
const unsigned char *imgPtr, 
Coord* coordPtr,
short r, 
short w
)
{
	// get the points feat
	InterestPoint* p_pointsCur = pointPtr;
	InterestPoint* p_pointsEnd = pointPtr + pointNum;
	unsigned int curPointSeq = 1;

	int neighPointNum = calcDiskTmplArray(coordPtr, r);

	if (pointPtr == NULL || imgPtr == NULL)
		return -1;

	for (; p_pointsCur != p_pointsEnd; ++p_pointsCur, ++curPointSeq)
	{
		calcFeat(p_pointsCur, imgPtr, coordPtr, neighPointNum, w);
	}

	return 0;
}


// get the square euro distance
double getEuroDist2(const double* vec1, const double* vec2, int dim)
{
	if (vec1 == NULL || vec2 == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		exit(-1);
	}

	double dist = 0.0;
	double dif = 0.0;
	int d = 0;

	for (d = 0; d < dim; ++d)
	{
		dif = vec1[d] - vec2[d];
		dist += (dif * dif);
	}

	return dist;
}

// get the most similar(nearest) point of current point
const InterestPoint* getNearestPoint
(
const InterestPoint *p_pointCur, 
const InterestPoint *p_pointsRef, 
int pointRefNum
)
{
	const InterestPoint* p_pointsRefNearest = NULL;
	const InterestPoint* p_pointsRefEnd = p_pointsRef + pointRefNum;
	double minDist = FEAT_MAX; // set enough maximum dist square threshold
	double dist = 0.0;

	if (p_pointsRef == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		return NULL;
	}

	if (p_pointCur == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		return NULL;
	}

	for (; p_pointsRef != p_pointsRefEnd; ++p_pointsRef)
	{
		dist = getEuroDist2((const double*)(&p_pointCur->mat.feat[0]), 
			(const double*)&p_pointsRef->mat.feat[0], FEAT_NUM);

		if (dist < minDist)
		{
			minDist = dist;
			p_pointsRefNearest = p_pointsRef;
		}
	}

	return p_pointsRefNearest;
}


// normalize all the feats
void normalizePointsFeats
(
InterestPoint* p_pointsL,
int pointNumL,
InterestPoint* p_pointsR,
int pointNumR,
double* p_featMin,
double* p_featMax,
double* p_featGap
)
{
	unsigned short f = 0;
	double* p_featMaxCur = p_featMax;
	double* p_featMinCur = p_featMin;
	double* p_featGapCur = p_featGap;

	InterestPoint* p_pointsLCur = p_pointsL;
	const InterestPoint* p_pointsLEnd = p_pointsL + pointNumL;
	InterestPoint* p_pointsRCur = p_pointsR;
	const InterestPoint* p_pointsREnd = p_pointsR + pointNumR;
	const double* p_featMinEnd = p_featMin + FEAT_NUM;

	if (p_pointsL == NULL || p_pointsR == NULL
		|| p_featMin == NULL || p_featMax == NULL || p_featGap == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		exit(1);
	}

	// initialize all the max and min value
	for (; p_featMinCur != p_featMinEnd; ++p_featMaxCur, ++p_featMinCur)
	{
		*p_featMaxCur = FEAT_MIN;
		*p_featMinCur = FEAT_MAX;
	}

	// get the max, min and gap value of different channel of feats
	for (p_pointsLCur = p_pointsL; p_pointsLCur != p_pointsLEnd; ++p_pointsLCur)
	{
		for (p_featMinCur = p_featMin, p_featMaxCur = p_featMax, f = 0; 
			p_featMinCur != p_featMinEnd; 
			++p_featMaxCur, ++p_featMinCur, ++f)
		{
			double val = p_pointsLCur->mat.feat[f];

			if (val > *p_featMaxCur)
				*p_featMaxCur = val;
			if (val < *p_featMinCur)
				*p_featMinCur = val;
		}
	}
	for (p_pointsRCur = p_pointsR; p_pointsRCur != p_pointsREnd; ++p_pointsRCur)
	{
		for (p_featMinCur = p_featMin, p_featMaxCur = p_featMax, f = 0; 
			p_featMinCur != p_featMinEnd; 
			++p_featMaxCur, ++p_featMinCur, ++f)
		{
			double val = p_pointsRCur->mat.feat[f];

			if (val > *p_featMaxCur)
				*p_featMaxCur = val;
			if (val < *p_featMinCur)
				*p_featMinCur = val;
		}
	}
	for (p_featMinCur = p_featMin, p_featMaxCur = p_featMax, p_featGapCur = p_featGap; 
		p_featMinCur != p_featMinEnd; 
		++p_featMaxCur, ++p_featMinCur, ++p_featGapCur)
	{
		*p_featGapCur = *p_featMaxCur - *p_featMinCur;
	}

	// normalize the feats
	unsigned int curPointSeq = 1;
	for (curPointSeq = 1, p_pointsLCur = p_pointsL; 
		p_pointsLCur != p_pointsLEnd; 
		++p_pointsLCur, ++curPointSeq)
	{
		for (p_featMinCur = p_featMin, p_featGapCur = p_featGap, f = 0;
			p_featMinCur != p_featMinEnd;
			++p_featMinCur, ++p_featGapCur, ++f)
		{
			p_pointsLCur->mat.feat[f] = (p_pointsLCur->mat.feat[f] - *p_featMinCur) 
				/ (*p_featGapCur);
		}
	}
	for (curPointSeq = 1, p_pointsRCur = p_pointsR;
		p_pointsRCur != p_pointsREnd; 
		++p_pointsRCur, ++curPointSeq)
	{
		for (f = 0, p_featMinCur = p_featMin, p_featGapCur = p_featGap;
			p_featMinCur != p_featMinEnd;
			++p_featMinCur, ++p_featGapCur, ++f)
		{
			p_pointsRCur->mat.feat[f] = (p_pointsRCur->mat.feat[f] - *p_featMinCur) 
				/ (*p_featGapCur);
		}
	}
}


// rough match based on mutual-minimum-distance
// returns the matched pair number
int roughMatch
(
const InterestPoint* p_pointsL, 
int pointNumL, 
const InterestPoint* p_pointsR,
int pointNumR,
PointPair* p_pairs
)
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
	const InterestPoint* p_matchedR = NULL;
	const InterestPoint* p_matchedL = NULL;

	// finding mutual nearest point
	for (; p_pointsLCur != p_pointsLEnd; ++p_pointsLCur)
	{
		if ((p_matchedR = getNearestPoint(p_pointsLCur, p_pointsRStart, pointNumR)) 
			!= NULL)
		{
			if ((p_matchedL = getNearestPoint(p_matchedR, p_pointsLStart, pointNumL))
				!= NULL)
			{
				// if mutual nearest relationship exists
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


/*// using ransac to get the best projection matrix based on the coarse matching pairs
ProjectMat getProjMatByRansac
(
const PointPair* p_pairs,
int pairNum, 
double distThresh, 
short wL, 
short hL, 
short wR,
short hR
)
{


	// threshold for ending the iteration
	// > innerPointNumThresh -- correct enough projection matrix coefficiency has been found
	// < innerPointNumThresh -- not yet
	unsigned int maxInnerPointNum = 0;

	// projection matrix co-efficiency
	ProjectMat curMat, suitMat;

	unsigned int iterateNum = 100 * pairNum;
	double** matSrc = NULL;
	mallocMat(&matSrc, 3, 3);
	double** matDst = NULL;
	mallocMat(&matDst, 3, 3);
	double** matT = NULL;
	mallocMat(&matT, 3, 3);

	if (p_pairs == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		exit(-1);
	}

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
			double expectedRx = curMat.m1 * (double)p_pairCur->pL.x 
				+ curMat.m2 * (double)p_pairCur->pL.y + curMat.m3;
			double expectedRy = curMat.m4 * (double)p_pairCur->pL.x 
				+ curMat.m5 * (double)p_pairCur->pL.y + curMat.m6;
			double dist = pow((expectedRx - p_pairCur->pR.x), 2) 
				+ pow((expectedRy - p_pairCur->pR.y), 2);
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
 */

// match the interest points of two images
// returns the matched pairs of interest points
ProjectMat matchInterestPoints
(
InterestPoint *p_pointsL, 
int pointNumL, 
InterestPoint *p_pointsR, 
int pointNumR, 
PointPair* p_pairs,
int* p_pairNum, 
double thresh, 
double* p_featMin,
double* p_featMax,
double* p_featGap,
short wL, 
short hL, 
short wR, 
short hR
)
{
	ProjectMat mat;

	if (p_pointsL == NULL || p_pointsR == NULL || p_pairNum == NULL)
	{
		DEBUG_PRINT_DETAILED("null input of pointers");
		exit(-1);
	}

	int pointTotalNum = pointNumL + pointNumR;

	// step 0: normalize all these feats
	normalizePointsFeats(p_pointsL, pointNumL, p_pointsR, pointNumR, p_featMin, p_featMax, p_featGap);

	// step 1: rough match based on mutual-minimum-distance
	*p_pairNum = roughMatch(p_pointsL, pointNumL, p_pointsR, pointNumR, p_pairs);

	return mat;
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
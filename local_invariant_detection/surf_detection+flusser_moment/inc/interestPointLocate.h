#ifndef INTERESTPOINTLOCATE_H
#define INTERESTPOINTLOCATE_H

#include "opencv2\core\core.hpp"

#include <footstone\ImgFilter\imgMath.h>

#include "debugMarco.h"

// specific parameter data of the det(Hessin) pyramid construction ========================
// maximum number of boxes
#define BOX_NUM 4
// number of filters(xx, yy, xy)
#define MODE_NUM  3
// number of layersS
#define LAYER_NUM 6

#define DETHES_MAX 1E100
#define DETHES_MIN -1E100


// filter box parameter structure
typedef struct BOX
{
	short tLX; // x coordinate of the top left corner of the box
	short tLY; // y coordinate of the top left corner of the box
	short bRX; // x coordinate of the bottom right corner of the box
	short bRY; // y coordinate of the bottom right corner of the box
	short s;   // area(pixel number) of the first box
	short wei; // weight of the first box
} Box;

typedef struct MODE
{
	short boxNum;
	Box box[BOX_NUM];
} Mode;

typedef struct FILTER
{
	Mode mode[MODE_NUM];
} Filter;

typedef struct BOX_PTRS
{
	const int* p_boxTL;
	const int* p_boxTR;
	const int* p_boxBL;
	const int* p_boxBR;
} BoxPtrs;

typedef struct MODE_PTRS
{
	unsigned short boxNum;
	BoxPtrs bptrs[BOX_NUM];
} ModePtrs;

typedef struct FILTER_PTRS
{
	ModePtrs mptrs[MODE_NUM];
} FilterPtrs;

typedef double featElemType;
#ifdef _NORMAL_FLUSSER_MOMENT_
#define FEAT_NUM 4
#endif

#ifdef _NORMAL_HU_MOMENT_
#define FEAT_NUM 6
#endif

#ifdef _MY_MOMENT_
#define FEAT_NUM 8
#endif


#define FEAT_MAX 1E100
#define FEAT_MIN -1E100
#define FEAT_OFFSET 1E-100

typedef struct HES_MAT
{
	//double dxx[LAYER_NUM];
	//double dyy[LAYER_NUM];
	//double dxy[LAYER_NUM];
	double val[LAYER_NUM];
}hesMat;

typedef struct INVARIANT_MOMENT
{
	featElemType feat[FEAT_NUM];
}InvarMoment;


typedef struct POINT
{
	short x;
	short y;
}Point;

typedef struct INTEREST_POINT
{
	Point c;
	InvarMoment mat;
	unsigned short r;
}InterestPoint;

typedef struct POINT_PAIR
{
	Point pL;
	Point pR;
}PointPair;

/*The 6 unknown coefficients for projection matrix*/
/*
	|	m1	m2	m3	|		|	x	|		|	x^	|
	|				|		|		|		|		|
	|	m4	m5	m6	|	*	| 	y	|	= 	|	y^	|
	|				|		|		|		|		|
	|	0	0	1	|		|	1	|		|	1	|
*/
typedef struct PROJECT_MATRIX
{
	double m1    ; double m2    ; double m3    ;
	double m4    ; double m5    ; double m6    ;
	double m7 = 0; double m8 = 0; double m9 = 1;
}ProjectMat;

// draw the rectangle around the interest point
void drawRect(cv::Mat& mat, const InterestPoint *p_points, int pointNum, short step, cv::Scalar color);

// init the pointers of the four corners of each box of certain filter
void initFiltPtrs(const int *p_integImg, FilterPtrs* p_filterPtrs, const Filter* p_filter, short w);

// increase the pointers of the four corners of each box of certain filter by certain value
void incFilterPtrs(FilterPtrs* p_filterPtrs, short val);

// calculate the det(Hessin) response of one pixel
void calcDetHes(const FilterPtrs *p_filterPtrs, const Filter* p_filter, hesMat* p_detHesImg, char layerSeq);

// create the det(Hessin) image of the input integral image at one specific octave and layer
void createDetHesImg(const int* p_integImg, hesMat* p_detHesImg, const unsigned char* p_markImg, short layOrder, short w, short h);

// create the det(Hessin) image pyramid of certain number of octaves and layers
void createDetHesImgPyr(hesMat *p_detHesImgPyr, const int *p_integImg, const unsigned char* p_markImg, short layNum, short w, short h);

// find the interest point
// returns 0: current pixel is not the regional maximum point
// returns 1: current pixel is the regional maximum point
int isRegionMaximum(const hesMat* p_in, short w, char layOrder);

// get the interest points location
// returns the number of the founded interst points
unsigned int getPointsLocations(InterestPoint* p_points, unsigned char* p_markImg, const unsigned char* p_img, const hesMat* p_detHesImgPyr, short layNum, double detHesThresh, short w, short h);

// calculate the feature of one interest point
void calcFeat(InterestPoint* p_point, const unsigned char* p_img, const Coord* p_coords, int neighPointNum, short w);

// calculate the features of all the located interest points
void getPointsFeats(InterestPoint* p_points, int pointNum, const unsigned char *p_img, short r, short w);

// get the most similar(nearest) point of current point
const InterestPoint* getNearestPoint(const InterestPoint *p_pointCur, const InterestPoint *p_pointsRef, int pointNumRef, double minDist);

// normalize all the feats
void normalizePointsFeats(InterestPoint* p_pointsL, int pointNumL, InterestPoint* p_pointsR, int pointNumR);

// rough match based on mutual-minimum-distance
// returns the matched pair number
int roughMatch(const InterestPoint* p_pointsL, int pointNumL, const InterestPoint* p_pointsR, int pointNumR, PointPair* p_pairs);

// match the interest points of two images
ProjectMat matchInterestPoints(InterestPoint *p_pointsL, int pointNumL, InterestPoint *p_pointsR, int pointNumR, PointPair* p_pairs, int* p_pairNum, double thresh, short wL, short hL, short wR, short hR);

// using ransac to get the best projection matrix based on the coarse matching pairs
ProjectMat getProjMatByRansac(const PointPair* p_pairs, int pairNum, double distThresh, short wL, short hL, short wR, short hR);

// draw the link of matched points of two images
void showMatchResult(const cv::Mat& matL, const cv::Mat& matR, const ProjectMat& realMat, const ProjectMat& suitMat, const InterestPoint* p_pointsL, int pointNumL, const PointPair* p_pairs, int pairNum, double dThresh, short step);


#endif // FPLOCATE_H

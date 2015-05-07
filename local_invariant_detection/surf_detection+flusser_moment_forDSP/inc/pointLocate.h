#ifndef INTERESTPOINTLOCATE_H
#define INTERESTPOINTLOCATE_H

#include "opencv2\core\core.hpp"
#include "imgMath.h"

// specific parameter data of the det(Hessin) pyramid construction ========================
// maximum number of boxes
#define BOX_NUM 4
// number of filters(xx, yy, xy)
#define MODE_NUM  3
// number of layersS
#define LAYER_NUM 3

#define DETHES_MAX 1E100
#define DETHES_MIN -1E100

#define FEAT_NUM 4

#define FEAT_MAX 1E100
#define FEAT_MIN -1E100
#define FEAT_OFFSET 1E-100


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
	short boxNum;
	BoxPtrs bptrs[BOX_NUM];
} ModePtrs;

typedef struct FILTER_PTRS
{
	ModePtrs mptrs[MODE_NUM];
} FilterPtrs;

typedef struct HES_MAT
{
	int val[LAYER_NUM];
}hesMat;

typedef struct INVARIANT_MOMENT
{
	double feat[FEAT_NUM];
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
	short r;
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
	double m1; double m2; double m3;
	double m4; double m5; double m6;
	double m7 = 0; double m8 = 0; double m9 = 1;
}ProjectMat;

int drawRect
(
unsigned char* imgPtr, 
const InterestPoint* pointPtr, 
int pointNum, 
unsigned char pixVal, 
short r, 
short w
);

int createDetHesImgPyr
(
hesMat *detHesImgPtr, 
const int *integImgPtr, 
const unsigned char* markImgPtr, 
short layNum,
short w,
short h
);

int getPointLocation(InterestPoint* pointPtr, unsigned char* marImgPtr, const unsigned char* imgPtr, const hesMat* detHesImgPtr, short layNum, double detHesThresh, short w, short h);

int getPointsFeats(InterestPoint* pointPtr, int pointNum, const unsigned char *imgPtr, Coord* coordPtr, short r, short w);


#endif // FPLOCATE_H

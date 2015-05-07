#ifndef IMGMATH_H
#define IMGMATH_H

enum PRE_PROC_MODE
{
	ONLY_GLOBAL_GUASSIN = 0,
	GLOBAL_EUQHIST,
	GET_PREGROUND,
	LOCAL_EQUHIST,
};

enum PIXEL_MARK_TYPE
{
	BACK_GROUND_PIXEL = 0,
	MARKED_TRUE,
	MARKED_FALSE,
	FORE_GROUND_PIXEL = 255,
};

typedef struct COORD
{
    short x;
    short y;
} Coord;


#define MAX_CONTOUR_NUM 1000
typedef struct CONTOUR
{
	Coord* coordPtr;
	int num[MAX_CONTOUR_NUM];
} Contour;

/*geometry*/
// calculate the neighbor pixel coordinates
int calcDiskTmplArray(Coord* p_coord, short r);

// create integral image of one image
int createIntegImg(const unsigned char *p_img, int* p_integImg, short w, short h);

/*filtering*/
int gaussin(const unsigned char* srcImgPtr, unsigned char* dstImgPtr, short w, short h);

unsigned char getThreshByFirstVally(unsigned char* imgPtr, short w, short h);

// fVal -- foreground value
int binary(unsigned char* imgPtr, unsigned char thresh, unsigned char fVal, short w, short h);

// fVal -- foreground value
int elate(const unsigned char* srcImgPtr, unsigned char* dstImgPtr, unsigned char fVal, short w, short h);

int subtract(unsigned char* imgPtr, const unsigned char* minuendImgPtr, short w, unsigned short h);

int equHist(unsigned char* imgPtr, const unsigned char* markImgPtr, unsigned char bVal, short w, short h);

int markOutContour(unsigned char* imgPtr, Contour* contourPtr, short w, short h);

int markMaxOutContour(unsigned char* imgPtr, Contour* contoursPtr, int contourNum, unsigned char markVal, short w, short h);

int fillRegion(unsigned char* imgPtr, short w, short h, unsigned char fillVal);

int preProcess(unsigned char* imgPtr, PRE_PROC_MODE mode, short w, short h);

#endif // IMGMATH_H

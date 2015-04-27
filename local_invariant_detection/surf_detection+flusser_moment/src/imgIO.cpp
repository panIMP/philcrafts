#include <DTool\Error\error.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <math.h>
#include <iostream>

#include "debugMarco.h"
#include "imgIO.h"

#pragma warning(disable:4996)

using namespace std;


// get size info a sequence of images
cv::Size getMergeSize(const cv::Mat *p_matArray, unsigned short matNum, Orientation orient)
{
    if (p_matArray == NULL)
    {
        DEBUG_PRINT_DETAILED("null input of mat arrays");
        exit(-1);
    }

    unsigned short t_W, t_H;
    unsigned short sumW = 0;
    unsigned short sumH = 0;
    unsigned short maxW = 0;
    unsigned short maxH = 0;

    for (unsigned short i = 0; i < matNum; ++i)
    {
        t_W = p_matArray[i].cols;
        t_H = p_matArray[i].rows;

        sumW += t_W;
        sumH += t_H;

        maxW = t_W > maxW ? t_W : maxW;
        maxH = t_H > maxH ? t_H : maxH;
    }

    cv::Size size;
    if (orient == vertical)
    {
        size.width = maxW;
        size.height = sumH;
    }
    else if (orient == horizontal)
    {
        size.width = sumW;
        size.height = maxH;
    }

    return size;
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


// rotate the image by different angles and scales
void rotateImg(cv::Mat& src, cv::Mat& dst, double angle)
{
	// method1  pure axis rotate
	int h = src.rows;
    int w = src.cols;
    int digonal = sqrt(w * w + h * h);
    int dh = (digonal - h) / 2;
    int dw = (digonal - w) / 2;

    cv::copyMakeBorder(src, dst, dh, dh, dw, dw, cv::BORDER_CONSTANT);

    cv::Point2f center((((double)dst.cols) / 2.0), (((double)dst.rows) / 2.0));

	cv::Mat rotateMat = cv::getRotationMatrix2D(center, angle, 1);

	cout << rotateMat << endl;

	cv::warpAffine(dst, dst, rotateMat, dst.size());

	double radian = (double)(((double)angle) / 180.0 * CV_PI);
	double sinVal = fabs(sin(radian));
	double cosVal = fabs(cos(radian));

	cv::Size targetSize((int)(w * cosVal + h * sinVal),(int)(w * sinVal + h * cosVal));
	int x = (dst.cols - targetSize.width) / 2;
	int y = (dst.rows - targetSize.height) / 2;
	cv::Rect rect(x, y, targetSize.width, targetSize.height);
	dst = cv::Mat(dst, rect);
}


cv::Mat affineImg(cv::Mat& src, double angle)
{
	int h = src.rows;
	int w = src.cols;
	double radian = (double)(((double)angle) / 180.0 * CV_PI);
	int hd = h;
	int wd = w * cos(radian);

	cv::Point2f srcTri[3];
	cv::Point2f dstTri[3];

	srcTri[0] = cv::Point2f(0, 0);
	srcTri[1] = cv::Point2f(src.cols - 1, 0);
	srcTri[2] = cv::Point2f(0, src.rows - 1);

	dstTri[0] = cv::Point2f((w - wd) / 2, 0);
	dstTri[1] = cv::Point2f((w + wd) / 2 - 1, 0);
	dstTri[2] = cv::Point2f((w - wd) / 2, src.rows - 1);

	cv::Mat dst = cv::Mat(h, w, CV_8UC3);

	cv::Mat wrapMat = cv::getAffineTransform(srcTri, dstTri);
	cout << wrapMat << endl;

	cv::warpAffine(src, dst, wrapMat, dst.size());

	cv::Rect rect((w - wd) / 2, 0, wd, hd);
	dst = cv::Mat(dst, rect);

	return dst;
}


// move image
cv::Mat moveImg(cv::Mat& src, double dx, double dy)
{
	int h = src.rows;
	int w = src.cols;
	int hd = h - dy;
	int wd = w - dx;

	cv::Mat dst = cv::Mat(hd, wd, CV_8UC3);
	//cv::Rect rect(dx, dy, wd, hd);
	cv::Rect rect(0, 0, wd, hd);
	dst = cv::Mat(src, rect);

	int hThickness = dy;
	int wThickness = dx;

	cv::copyMakeBorder(dst, dst, hThickness, 0, wThickness, 0, cv::BORDER_REPLICATE);

	return dst;
}


// scale image
cv::Mat scaleImg(cv::Mat& src, double scale)
{
	int h = src.rows;
	int w = src.cols;
	int hd = (double)(h * sqrt(scale));
	int wd = (double)(w * sqrt(scale));

	cv::Mat dst = cv::Mat(hd, wd, CV_8UC3);

	cv::resize(src, dst, dst.size());
	
	int hThickness = (h - hd) / 2;
	int wThickness = (w - wd) / 2;

	cv::copyMakeBorder(dst, dst, hThickness, hThickness, wThickness, wThickness, cv::BORDER_REPLICATE);

	return dst;
}

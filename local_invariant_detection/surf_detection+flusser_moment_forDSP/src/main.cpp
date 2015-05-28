// Copyright [2015] <Phil Hu>

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include "../inc/algManage.h"

int main()
{
    // algorithm memory initialize
    alg_create(640, 480);

    // algorithm parameter assign value
    alg_control();

    // left image process
    cv::Mat_<unsigned char> matL = cv::imread("F:/Pics/juanbidao/pos/juanbidao2.bmp", cv::IMREAD_GRAYSCALE);
    cv::Mat_<unsigned char> matLProc;
    matL.copyTo(matLProc);

    cv::imshow("matL", matL);

    alg_process(matLProc.data, matLProc.cols, matLProc.rows);

    cv::imshow("matLProc", matLProc);

    // right image process
 /*   cv::Mat_<unsigned char> matR = cv::imread("F:/Pics/juanbidao/pos/juanbidao14.bmp", cv::IMREAD_GRAYSCALE);
    cv::Mat_<unsigned char> matRProc;
    matR.copyTo(matRProc);

    cv::imshow("matR", matR);

    alg_process(matRProc.data, matRProc.cols, matRProc.rows);

    cv::imshow("matRProc", matRProc);*/

    // free algorithm memory
    alg_delete();

    cv::waitKey();

    return 0;
}
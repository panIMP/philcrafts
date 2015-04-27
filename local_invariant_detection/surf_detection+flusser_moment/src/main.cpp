#include <opencv2\highgui\highgui.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\legacy\legacy.hpp>

#include <DTool\Error\error.h>
#include <footstone\ImgFilter\imgMath.h>

#include <string.h>
#include <time.h>

#include "imgIO.h"
#include "interestPointLocate.h"
#include "debugMarco.h"


using namespace std;


string g_savepath = "C:\\Users\\攀\\Desktop\\大论文\\";


int preProcess(unsigned char mode, unsigned char* p_img, unsigned char* p_imgMark, unsigned char maxVal, unsigned short w, unsigned short h)
{
	int imgSize = w * h;
	unsigned char* p_imgTmp = (unsigned char*)malloc_check(imgSize * sizeof(unsigned char));
	int imgSizeForeground = 0;

	if (mode == 0)
	{
		gaussin(p_img, p_imgTmp, w, h);
		memcpy(p_img, p_imgTmp, imgSize);

		binary(p_img, p_imgTmp, getThreshByFirstVally(p_img, w, h), FORE_GROUND_PIXEL, w, h);
		
		elate(p_imgTmp, p_imgMark, FORE_GROUND_PIXEL, w, h);

		
		subtract(p_imgMark, p_imgTmp, w, h);

		Contour contours;
		contours.p_coords = (Coord*)malloc_check(w * h * sizeof(Coord));
		int contourNum = markOutContour(p_imgMark, &contours, w, h);
		markMaxOutContour(p_imgMark, &contours, contourNum, FORE_GROUND_PIXEL, w, h);
		imgSizeForeground = fillRegion(p_imgMark, w, h, maxVal);
		equHist(p_img, p_imgMark, w, h);

		free(contours.p_coords);
		contours.p_coords = NULL;
	}
	else if (mode == 1)
	{
		gaussin(p_img, p_imgTmp, w, h);
		memcpy(p_img, p_imgTmp, imgSize);

		equHist(p_img, p_imgMark, w, h);
	}

	return imgSizeForeground;
}


#ifdef _SIFT_SURF_
int main()
{
	cv::Mat img1 = cv::imread("F:/Pics/juanbidao/guangzhao/juanbidao6.bmp", cv::IMREAD_GRAYSCALE);
	cv::Mat img2 = cv::imread("F:/Pics/juanbidao/guangzhao/juanbidao1.bmp", cv::IMREAD_GRAYSCALE);

	double img1Even = 0;
	double img2Even = 0;
	double img1Size = img1.cols * img1.rows;
	double img2Size = img2.cols * img2.rows;
	
	for (int i = 0; i < img1Size; ++i)
	{
		img1Even += img1.data[i];
	}
	img1Even /= img1Size;

	for (int i = 0; i < img2Size; ++i)
	{
		img2Even += img2.data[i];
	}
	img2Even /= img2Size;

	cout << img1Even << "\t" << img2Even << endl;

	cv::SiftFeatureDetector detector;
	//cv::SurfFeatureDetector detector(400);

	std::vector<cv::KeyPoint> keypoints1, keypoints2;

	detector.detect(img1, keypoints1);
	detector.detect(img2, keypoints2);

	cv::Mat img1KeyPoints, img2KeyPoints;

	cv::drawKeypoints(img1, keypoints1, img1KeyPoints, cv::Scalar(255, 255, 255));
	cv::drawKeypoints(img2, keypoints2, img2KeyPoints, cv::Scalar(255, 255, 255));

	cv::imshow("keyPoints1", img1KeyPoints);
	cv::imshow("keyPoints2", img2KeyPoints);
	//cv::imwrite("C:\\Users\\攀\\Desktop\\大论文\\posYkq1.png", img1KeyPoints);

	cv::SiftDescriptorExtractor extractor;
	//cv::SurfDescriptorExtractor extractor;

	cv::Mat descriptors1, descriptors2;

	extractor.compute(img1, keypoints1, descriptors1);
	extractor.compute(img2, keypoints2, descriptors2);

	cv::BruteForceMatcher<cv::L2<float> > matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);

	cout << keypoints1.size() << "\t" << keypoints2.size() << endl;
	int j = 0;
	for (int i = 0; i < matches.size(); ++i)
	{
		int k1Seq = matches[i].queryIdx;
		int k2Seq = matches[i].trainIdx;
		int k1X = keypoints1[k1Seq].pt.x;
		int k2X = keypoints2[k2Seq].pt.x;
		int k1Y = keypoints1[k1Seq].pt.y;
		int k2Y = keypoints2[k2Seq].pt.y;
		int dist = pow(k1X - k2X, 2) + pow(k1Y - k2Y, 2);
		dist = sqrt(dist);
		if (dist <= 4)
			j++;
	}
	cout << (double)j / (double)(MIN(keypoints1.size(), keypoints2.size())) << endl;

	int k = 0;
	for (int i = 0; i < matches.size(); i += 10, k++)
	{
		matches[k] = matches[i];
	}
	matches.resize(k+1);

	cv::namedWindow("matches");
	cv::Mat img_matches;
	cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255));
	cv::imshow("matches", img_matches);
	cv::imwrite(g_savepath + "surfguangzhao.png", img_matches);
	cv::waitKey(0);

	return 0;
}
#endif

#ifdef _MAIN_TEST_
int main()
{
	// ============================================================================================================================
	// left image
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("F:/Pics/jiong/pos/1.bmp"), cv::IMREAD_COLOR);
	cv::Mat_<cv::Vec3b> matLColorSrc = cv::imread(string("F:/Pics/juanbidao/qingxie/juanbidao10.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColorSrc = cv::imread(string("F:/Pics/ykq/src/d20.png"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColorSrc = cv::imread(string("F:/Pics/gongjian/1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("F:/Pics/Dolls/Illum1/Exp0/view0.png"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("F:/Pics/grap/img1.ppm"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("F:/Pics/sift/box.pgm"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColorSrc = cv::imread(string("F:/Pics/dragon/viff.000.ppm"), cv::IMREAD_COLOR);
	if (matLColorSrc.empty())
		return -1;

	// right image
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("F:/Pics/jiong/pos/3.bmp"), cv::IMREAD_COLOR);
	cv::Mat_<cv::Vec3b> matRColorSrc = cv::imread(string("F:/Pics/juanbidao/guangzhao/juanbidao1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColorSrc = cv::imread(string("F:/Pics/ykq/src/1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColorSrc = cv::imread(string("F:/Pics/gongjian/1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("F:/Pics/Dolls/Illum3/Exp2/view0.png"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("F:/Pics/grap/img3.ppm"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColor = cv::imread(string("F:/Pics/sift/scene.pgm"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matRColorSrc = cv::imread(string("F:/Pics/dragon/viff.000.ppm"), cv::IMREAD_COLOR);
	if (matRColorSrc.empty())
		return -1;

	ProjectMat realMat;
	realMat.m1 = 1;
	realMat.m2 = 0;
	realMat.m3 = 0;
	realMat.m4 = 0;
	realMat.m5 = 1;
	realMat.m6 = 0;

#ifdef _MODULATE_DATA_
	double angle1 = 0.0;
	double angle2 = 0.0;
	double angle3 = 0.0;
	double dx = 100;
	double dy = dx;
	double scale = 1;

	// affine transform
	matLColorSrc = scaleImg(matLColorSrc, scale);
	rotateImg(matLColorSrc, matLColorSrc, angle3);
	matLColorSrc = affineImg(matLColorSrc, angle2);
	rotateImg(matLColorSrc, matLColorSrc, angle1);
	matLColorSrc = moveImg(matLColorSrc, dx, dy);
	//cv::GaussianBlur(matLColorSrc, matLColorSrc, cv::Size(matLColorSrc.cols - 1, matLColorSrc.rows - 1), 3.8);

	//cv::imshow("jokesrc", matLColorSrc);
	//cv::imshow("joke", matRColorSrc);
	//cv::imwrite(g_savepath + "mohu3.8.png", matLColorSrc);
#endif

	cv::Mat matLColor;
	matLColorSrc.copyTo(matLColor);
	cv::Mat matRColor;
	matRColorSrc.copyTo(matRColor);

	// ==============================================================================================================================
	cv::Mat_<unsigned char> matL;
	cv::cvtColor(matLColor, matL, CV_BGR2GRAY);
	unsigned char* p_imgL = matL.data;
	unsigned short wL = matL.cols;
	unsigned short hL = matL.rows;
	unsigned int areaL = wL * hL;

	cv::Mat_<unsigned char> matR;
	cv::cvtColor(matRColor, matR, CV_BGR2GRAY);
	unsigned char* p_imgR = matR.data;
	unsigned short wR = matR.cols;
	unsigned short hR = matR.rows;
	unsigned int areaR = wR * hR;

	cv::Mat_<unsigned char> t_matL(hL, wL);
	cv::Mat_<unsigned char> t_matR(hR, wR);
	memset(t_matL.data, FORE_GROUND_PIXEL, wL * hL);
	memset(t_matR.data, FORE_GROUND_PIXEL, wR * hR);
	
#ifdef _PRE_PROCESS_
	//areaL = preProcess(0, matL.data, t_matL.data, FORE_GROUND_PIXEL, wL, hL);
	//cv::imshow("t_matL", t_matL);
	//cv::imwrite(g_savepath + "gongjiangEndge.png", t_matL);

	//areaR = preProcess(0, matR.data, t_matR.data, FORE_GROUND_PIXEL, wR, hR);
	//cv::imshow("t_matR", t_matR);
#endif

	//cv::imshow("matL", matL);
	//cv::imshow("matR", matR);

	// ==============================================================================================================================
	// create the integrate image of the left image
	int* p_integImgL = (int*)malloc_check(wL * hL * sizeof(int));
	createIntegImg(p_imgL, p_integImgL, wL, hL);

	// create the det(Hessin) images of different octaves and layers of the left image
	hesMat* p_detHesImgPyrL = (hesMat*)calloc_check(wL * hL, sizeof(hesMat));
	createDetHesImgPyr(p_detHesImgPyrL, p_integImgL, t_matL.data, LAYER_NUM, wL, hL);

	// locate the interest points in the pyramid of the left image
	InterestPoint* p_pointsL = (InterestPoint*)calloc_check(wL * hL, sizeof(InterestPoint));
	int pointNumL = getPointsLocations(p_pointsL, t_matL.data, p_imgL, p_detHesImgPyrL, LAYER_NUM, 400.0, wL, hL);

	// ==============================================================================================================================
	// create the integrate image of the right image
	int* p_integImgR = (int*)malloc_check(wR * hR * sizeof(int));
	createIntegImg(p_imgR, p_integImgR, wR, hR);

	// create the det(Hessin) images of different octaves and layers of the right image
	hesMat* p_detHesImgPyrR = (hesMat*)calloc_check(wR * hR, sizeof(hesMat));
	createDetHesImgPyr(p_detHesImgPyrR, p_integImgR, t_matR.data, LAYER_NUM, wR, hR);

	// locate the interest points in the pyramid of the right image
	InterestPoint* p_pointsR = (InterestPoint*)calloc_check(wR * hR, sizeof(InterestPoint));
	int pointNumR = getPointsLocations(p_pointsR, t_matR.data, p_imgR, p_detHesImgPyrR, LAYER_NUM, 400.0, wR, hR);

	// ==============================================================================================================================
	double dR = (double)(areaR) / (double)pointNumR;
	short r = sqrt(dR) / 2;
	r = 12;
	double dThresh = r * r / 4;

	cout << "left image interest point number: " << pointNumL << endl;
	time_t start = clock();
	getPointsFeats(p_pointsL, pointNumL, p_imgL, r, wL);
	time_t end = clock();
	cout << end - start << endl;

	cout << "right image interest point number: " << pointNumR << endl;
	getPointsFeats(p_pointsR, pointNumR, p_imgR, r, wR);

	// show compare point location result
	cv::cvtColor(matL, matLColor, cv::COLOR_GRAY2BGR);
	cv::cvtColor(matR, matRColor, cv::COLOR_GRAY2BGR);
	drawRect(matLColorSrc, p_pointsL, pointNumL, 1, cv::Scalar(255, 255, 255));
	//cv::imshow("matLColor", matLColor);
	//cv::imwrite("C:\\Users\\攀\\Desktop\\大论文\\juanbidaodarkpoint.png", matLColor);
	drawRect(matRColorSrc, p_pointsR, pointNumR, 1, cv::Scalar(255, 255, 255));
	//cv::imshow("matRColor", matRColor);
	//cv::imwrite("C:\\Users\\攀\\Desktop\\大论文\\juanbidaobrightpoint.png", matRColor);

	// ==============================================================================================================================
	// match the two images
	int pairNum = 0;
	PointPair* p_pairs = (PointPair*)calloc_check(max(pointNumL, pointNumR), sizeof(PointPair));
	ProjectMat suitMat = matchInterestPoints(p_pointsL, pointNumL, p_pointsR, pointNumR, p_pairs, &pairNum, dThresh, wL, hL, wR, hR);
	showMatchResult(matLColorSrc, matRColorSrc, realMat, suitMat, p_pointsL, pointNumL, p_pairs, pairNum, dThresh, 2);

	int j = 0;
	for (int i = 0; i < pairNum; ++i)
	{
		double xL = p_pairs[i].pL.x;
		double yL = p_pairs[i].pL.y;
		double xR = p_pairs[i].pR.x;
		double yR = p_pairs[i].pR.y;
		double dist = pow(xR - xL, 2) + pow(yR - yL, 2);
		dist = sqrt(dist);

		if (dist <= 4)
			j++;
	}
	cout << (double)j / double(MIN(pointNumL, pointNumR)) << endl;

	// ===============================================================================================================================
	// free memory
	free(p_integImgL);
	free(p_detHesImgPyrL);
	free(p_pointsL);

	free(p_integImgR);
	free(p_detHesImgPyrR);
	free(p_pointsR);

	free(p_pairs);

	cv::waitKey();
	return 0;
}
#endif



#ifdef _BINARY_
int main()
{
	// left image
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("F:/Pics/jiong/pos/1.bmp"), cv::IMREAD_COLOR);
	cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("F:/Pics/juanbidao/guangzhao/juanbidao5.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("F:/Pics/gongjian/1.bmp"), cv::IMREAD_COLOR);
	//cv::Mat_<cv::Vec3b> matLColor = cv::imread(string("F:/Pics/ykq/src/1.bmp"), cv::IMREAD_COLOR);

	cv::Mat_<unsigned char> matL;
	cv::cvtColor(matLColor, matL, CV_BGR2GRAY);
	unsigned char* p_imgL = matL.data;
	unsigned short w = matL.cols;
	unsigned short h = matL.rows;
	cv::Mat_<unsigned char> mat0(h, w);
	cv::Mat_<unsigned char> mat1(h, w);
	cv::Mat_<unsigned char> mat2(h, w);
	cv::Mat_<unsigned char> mat3(h, w);
	cv::Mat_<unsigned char> mat4(h, w);

	gaussin(p_imgL, w, h);

	matL.copyTo(mat0);
	matL.copyTo(mat1);
	matL.copyTo(mat2);
	matL.copyTo(mat3);
	matL.copyTo(mat4);

	cv::imshow("src", matL);
	cv::imwrite("C:/src.png", matL);

	cv::threshold(mat0, mat0, 0, 255, cv::THRESH_OTSU);
	cv::imshow("opencv global otsu", mat0);
	cv::imwrite("C:/opencv_global_otsu.png", mat0);

	otsuBinary(mat1.data, 255, w, h);
	cv::imshow("global otsu", mat1);
	cv::imwrite("C:/global_otsu.png", mat1);

	localOtsuBinary(mat2.data, 255, w, h, 4);
	cv::imshow("local otsu", mat2);
	cv::imwrite("C:/local_otsu.png", mat2);

	localOtsuRecurBinary(mat3.data, 255, w, h, 4);
	cv::imshow("local recur otsu", mat3);
	cv::imwrite("C:/local_recur_otsu.png", mat3);

	binary(mat4.data, threshByFirstVally(mat4.data, w, h), 255, w, h);
	cv::imshow("thresh by triangular", mat4);
	cv::imwrite("thresh_by_triangular.png", mat4);
	
	setBoundaryZero(mat4.data, w, h);
	cv::Mat_<unsigned char> mat4Edge(w, h);
	mat4.copyTo(mat4Edge);
	elate(mat4Edge.data, mat4.data, w, h);
	subtract(mat4.data, mat4Edge.data, w, h);
	cv::imshow("edge", mat4);
	cv::imwrite("C:/edge.png", mat4);

	Contour contours;
	contours.p_coords = (Coord*)malloc_check(mat4.cols * mat4.rows * sizeof(Coord));
	int contourNum = markOutContour(mat4.data, &contours, mat4.cols, mat4.rows);
	markMaxOutContour(mat4.data, &contours, contourNum, mat4.cols, mat4.rows);
	cv::imshow("biggestEdge", mat4);
	cv::imwrite("C:/biggestEdge.png", mat4);

	fillRegion(mat4.data, w, h, 255);
	//cv::floodFill(mat4, cv::Point(347, 118), 255);
	cv::imshow("fill", mat4);
	cv::imwrite("C:/fill.png", mat4);

	equHist(matL.data, mat4.data, w, h);
	cv::imshow("equalized", matL);
	cv::imwrite("C:/equalized1.png", matL);

	cv::waitKey(0);
	return 0;
}

#endif


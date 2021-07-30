#ifndef __QUICK_OPENCV_H__
#define __QUICK_OPENCV_H__

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define PI 3.1415926
#define M_PI 3.14159265358979323846
#define Kernel_size 5
#define sigma 3

class LineFinder {
private:
	std::vector<cv::Vec4i> lines;
	std::vector<cv::Vec4i> line1;
	std::vector<cv::Vec4i> lastline;
	std::vector<cv::Vec4i> linesmid;

	double deltaRho;
	double deltaTheta;
	int minVote;
	double minLength;
	double maxGap;
	
public:

	LineFinder() : deltaRho(1), deltaTheta(PI / 180),
		minVote(10), minLength(0.), maxGap(0.) {}

	void setAccResolution(double dRho, double dTheta);

	void setMinVote(int minv);

	void setLineLengthAndGap(double length, double gap);

	std::vector<cv::Vec4i> findLines(cv::Mat& binary);

	void drawDetectedLines(cv::Mat& image,
		cv::Scalar color = cv::Scalar(255, 255, 255));

	cv::Vec4f fittingLines(std::vector<cv::Vec4i>& line);

	void drawfittingLines(cv::Mat& fittingline, cv::Vec4f liness1);

	double least_method(std::vector<cv::Point> points);  //ÓÐÈ¡Ö··ûºÅµÄÊ±ºò£¬Ã»ÓÐ¶ÔžÃ²Ù×÷œøÐÐÔËÐÐ¡£

	void detectpoint(Mat img, std::vector<cv::Point>& points1);
	
	void detectrect(Mat& img, Mat& imageROI1, Mat& imageROI2);

	void ROIchoose(Mat image1, Mat& r, Rect rect);

	void thresholding(Mat& image, Mat& thresholded, int n);

	Mat Log_filter(Mat img);

	void zhangSkeleton(Mat& srcimage);

	int  pixel_num(Mat& img);

	void fitted_curve(cv::Mat& fittingline, cv::Vec4f& line);

	void finish(Mat image, cv::Vec4f& line);

    // two line cross point
	void final(Mat image);
};

#endif
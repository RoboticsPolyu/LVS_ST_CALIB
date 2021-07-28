#include "quickopencv.h"
#include <opencv2/opencv.hpp>
#include "quickopencv.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

#include <ctime>
#include<cmath>
using namespace cv;
using namespace std;

#define PI 3.1415926
#define M_PI 3.14159265358979323846
#define Kernel_size 5
#define sigma 3

	void LineFinder::setAccResolution(double dRho, double dTheta)
	{
		deltaRho = dRho;
		deltaTheta = dTheta;
	}
	// ÉèÖÃ×îÐ¡Í¶Æ±Êý 
	void  LineFinder::setMinVote(int minv)
	{
		minVote = minv;
	}
	// ÉèÖÃÖ±Ïß³€¶ÈºÍ¿ÕÏ¶
	void  LineFinder::setLineLengthAndGap(double length, double gap)
	{
		minLength = length;
		maxGap = gap;
	}

	// ÓŠÓÃžÅÂÊ»ô·ò±ä»»
	std::vector<cv::Vec4i>  LineFinder::findLines(cv::Mat& binary)
	{
		lines.clear();
		cv::HoughLinesP(binary, lines,
			deltaRho, deltaTheta, minVote,
			minLength, maxGap);
		return lines;
	}

	void  LineFinder::drawDetectedLines(cv::Mat& image,
		cv::Scalar color )
	{
		// »­Ö±Ïß
		std::vector<cv::Vec4i>::const_iterator it2 = lines.begin();
		while (it2 != lines.end())
		{
			cv::Point pt1((*it2)[0], (*it2)[1]);
			cv::Point pt2((*it2)[2], (*it2)[3]);
			cv::line(image, pt1, pt2, color);
			++it2;
		}
	}

	cv::Vec4f  LineFinder::fittingLines(std::vector<cv::Vec4i>& line)
	{
		cv::Vec4f lineaa1;;
		std::vector<cv::Point> points;
		
		std::vector<cv::Vec4i>::const_iterator it2 = line.begin();
		while (it2 != line.end())
		{
			cv::Point pt1((*it2)[0], (*it2)[1]);
			points.push_back(pt1);
			cv::Point pt2((*it2)[2], (*it2)[3]);
			points.push_back(pt2);
			++it2;
		}

		cv::fitLine(points, lineaa1,
			cv::DIST_L2, // ŸàÀëÀàÐÍ
			0, // L2 ŸàÀë²»ÓÃÕâžö²ÎÊý
			0.01, 0.01); // Ÿ«¶È

		return lineaa1;
	}

	void  LineFinder::drawfittingLines(cv::Mat& fittingline, cv::Vec4f liness1)
	{
		cv::Scalar color1 = cv::Scalar(255, 255, 255);
		int x0 = liness1[2]; // Ö±ÏßÉÏµÄÒ»žöµã
		int y0 = liness1[3];
		int x1 = x0 + 2500 * liness1[0]; // ŒÓÉÏ³€¶ÈÎª 100 µÄÏòÁ¿
		int y1 = y0 + 2500 * liness1[1]; //£šÓÃµ¥Î»ÏòÁ¿Éú³É£©
		int x2 = x0 - 2500 * liness1[0]; // ŒÓÉÏ³€¶ÈÎª 100 µÄÏòÁ¿
		int y2 = y0 - 2500 * liness1[1]; //£šÓÃµ¥Î»ÏòÁ¿Éú³É£©

		cv::line(fittingline, cv::Point(x2, y2), cv::Point(x1, y1),
			color1); // ÑÕÉ«ºÍ¿í¶È
	}

	double LineFinder::least_method(std::vector<cv::Point> points)   //ÓÐÈ¡Ö··ûºÅµÄÊ±ºò£¬Ã»ÓÐ¶ÔžÃ²Ù×÷œøÐÐÔËÐÐ¡£

	{
		double n = points.size();//n±íÊŸŒì²âµœµÄµãµÄžöÊý
		double sum_x = 0;
		double sum_y = 0;
		double sum_x2 = 0;
		double sum_xy = 0;
		double i = 0;
		double j = 0;
		double a0, a1, minex = 0;  //minexÎª×îÐ¡ÖÐÖµ


		for (int k = 0; k < n; k++)
		{
			i = points[k].x;
			j = points[k].y;
			sum_x = sum_x + i;
			sum_y = sum_y + j;
			sum_x2 = sum_x2 + i * i;
			sum_xy = sum_xy + i * j;

		}
		a1 = (n * sum_xy - sum_x * sum_y) / (sum_x * sum_x - n * sum_x2);
		a0 = (sum_y - sum_x * a1) / n;
		for (int k = 0; k < n; k++)
		{
			i = points[k].x;
			j = points[k].y;
			minex = minex + (a0 + a1 * i - j) * (a0 + a1 * i - j);//µÃµœ×îÐ¡ÖÐÖµµÄŽóÐ¡
		}
		return minex;
	}

	void  LineFinder::detectpoint(Mat img, std::vector<cv::Point>& points1)
	{
		for (int i = 0; i < img.rows; i++)
		{
			for (int j = 0; j < img.cols; j++)
			{

				int k = img.ptr<uchar>(i)[j];//
				if (k > 100)
				{
					cv::Point pt1(j, i);
					points1.push_back(pt1);

				}

			}
		}

	}

	void  LineFinder::detectrect(Mat& img, Mat& imageROI1, Mat& imageROI2)
	{
		int k;
		Mat imageROI3;
		Mat imageROI4;


		for (int i = 0; i < img.rows; i++)
		{
			for (int j = 0; j < img.cols; j++)
			{

				k = img.ptr<uchar>(i)[j];//
				if (k > 100)
				{
					int high = 600;
					Rect rect1 = Rect(j, i, high, high);
					rect1 &= Rect(0, 0, img.cols, img.rows);//Çóœ»Œ¯
					ROIchoose(img, imageROI1, rect1);
					Rect rect = Rect(j - high, i, high, high);
					rect &= Rect(0, 0, img.cols, img.rows);//Çóœ»Œ¯
					ROIchoose(img, imageROI3, rect);
					int pixelnum = pixel_num(imageROI1);
					int pixelnum1 = pixel_num(imageROI3);
					//±ÈœÏÏñËØµã£¬È·¶šROIÇøÓò
					if (pixelnum < pixelnum1)
					{
						imageROI1 = imageROI3;
					}
					break;
				}

			}
			if (k > 100)
			{

				break;
			}
		}



		for (int i = img.rows; i > 1; i--)
		{
			for (int j = img.cols; j > 1; j--)
			{

				k = img.ptr<uchar>(i)[j];//
				if (k > 100)
				{
					Rect rect;
					Rect rect1;
					int high = 600;
					rect.x = j - high;
					rect.y = i - high;
					rect.height = high;
					rect.width = high;
					rect &= Rect(0, 0, img.cols, img.rows);//Çóœ»Œ¯
					ROIchoose(img, imageROI2, rect);
					rect1 = Rect(j, i - high, high, high);
					rect1 &= Rect(0, 0, img.cols, img.rows);//Çóœ»Œ¯
					ROIchoose(img, imageROI4, rect1);

					int pixelnum = pixel_num(imageROI2);
					int pixelnum1 = pixel_num(imageROI4);
					//±ÈœÏÏñËØµã£¬È·¶šROIÇøÓò
					if (pixelnum < pixelnum1)
					{
						imageROI2 = imageROI4;
					}



					break;
				}

			}
			if (k > 100)
			{

				break;
			}
		}
	}

	void  LineFinder::ROIchoose(Mat image1, Mat& r, Rect rect)
	{
		Mat mask(image1.rows, image1.cols, CV_8UC1, Scalar(0, 0, 0));
		rectangle(mask, rect, Scalar(255, 255, 255), -1);//ºáŸØÐÎ
		const uchar white = 255;

		bitwise_and(image1, mask, r);

	}

	int LineFinder::pixel_num(Mat& img)
	{
		std::vector<cv::Point> points1;
		for (int i = 0; i < img.rows; i++)
		{
			for (int j = 0; j < img.cols; j++)
			{

				int k = img.ptr<uchar>(i)[j];//
				if (k > 100)
				{
					cv::Point pt1(j, i);
					points1.push_back(pt1);

				}

			}
		}
		int n = points1.size();
		points1.clear();
		vector<cv::Point>().swap(points1);
		return n;
	}

	void  LineFinder::thresholding(Mat& image, Mat& thresholded, int n)
	{
		//int n = 230;    //ãÐÖµ»¯³Ì¶Èn
		//cv::Mat thresholded;                         // Êä³ö¶þÖµÍŒÏñ
		cv::threshold(image, thresholded, n,         // ãÐÖµ
			255,                                     // ¶Ô³¬¹ýãÐÖµµÄÏñËØž³Öµ 
			cv::THRESH_BINARY);                          // ãÐÖµ»¯ÀàÐÍ

	}

	// LOGÂË²š
	Mat  LineFinder::Log_filter(Mat img) {
		//¹¹œš3*3ÂË²šÏµÊýŸØÕó
		double K[Kernel_size][Kernel_size];
		int imgrow = img.rows;
		int imgcol = img.cols;
		Mat out = Mat::zeros(imgrow, imgcol, CV_8UC1);
		int dot = floor((float)(Kernel_size / 2));
		double Sum = 0;
		int x = 0;
		int y = 0;
		for (int i = 0; i < Kernel_size; i++) {
			for (int j = 0; j < Kernel_size; j++) {
				x = j - dot;
				y = i - dot;
				K[i][j] = (x * x + y * y - sigma * sigma) / (2 * M_PI * pow((double)sigma, 6)) * exp((-(x * x + y * y) / (2 * sigma * sigma)) * 1.0);
				Sum += K[i][j];
			}
		}
		for (int i = 0; i < Kernel_size; i++) {
			for (int j = 0; j < Kernel_size; j++) {
				K[i][j] /= Sum;
			}
		}
		int v;
		for (int i = floor((float)(Kernel_size / 2)); i < imgrow - floor((float)(Kernel_size / 2)); i++) {
			for (int j = floor((float)(Kernel_size / 2)); j < imgcol - floor((float)(Kernel_size / 2)); j++) {
				v = 0;
				for (int i_ = -dot; i_ < dot + 1; i_++) {
					for (int j_ = -dot; j_ < dot + 1; j_++) {
						v += img.at<uchar>(i + i_, j + j_) * K[i_ + dot][j_ + dot];
					}
				}
				if (v >= 0)
					out.at<uchar>(i, j) = v;
				else
					out.at<uchar>(i, j) = -v;
			}
		}
		return out;
	}



	//ÖÐÖá±ä»»·š
	void  LineFinder::zhangSkeleton(Mat& srcimage)
	{
		int kernel[9];
		int nl = srcimage.rows;
		int nc = srcimage.cols;
		vector<Point> delete_list;
		int A, B;
		while (true)
		{
			for (int j = 1; j < nl - 1; j++)
			{
				uchar* data_pre = srcimage.ptr<uchar>(j - 1);
				uchar* data = srcimage.ptr<uchar>(j);
				uchar* data_next = srcimage.ptr<uchar>(j + 1);
				for (int i = 1; i < (nc - 1); i++)
				{
					if (data[i] == 255)
					{
						kernel[0] = 1;
						if (data_pre[i] == 255) kernel[1] = 1;
						else  kernel[1] = 0;
						if (data_pre[i + 1] == 255) kernel[2] = 1;
						else  kernel[2] = 0;
						if (data[i + 1] == 255) kernel[3] = 1;
						else  kernel[3] = 0;
						if (data_next[i + 1] == 255) kernel[4] = 1;
						else  kernel[4] = 0;
						if (data_next[i] == 255) kernel[5] = 1;
						else  kernel[5] = 0;
						if (data_next[i - 1] == 255) kernel[6] = 1;
						else  kernel[6] = 0;
						if (data[i - 1] == 255) kernel[7] = 1;
						else  kernel[7] = 0;
						if (data_pre[i - 1] == 255) kernel[8] = 1;
						else  kernel[8] = 0;

						B = 0;
						for (int k = 1; k < 9; k++)
						{
							B = B + kernel[k];
						}
						if ((B >= 2) && (B <= 6))
						{
							A = 0;
							if (!kernel[1] && kernel[2]) A++;
							if (!kernel[2] && kernel[3]) A++;
							if (!kernel[3] && kernel[4]) A++;
							if (!kernel[4] && kernel[5]) A++;
							if (!kernel[5] && kernel[6]) A++;
							if (!kernel[6] && kernel[7]) A++;
							if (!kernel[7] && kernel[8]) A++;
							if (!kernel[8] && kernel[1]) A++;
							//
							if (A == 1)
							{
								if ((kernel[1] * kernel[3] * kernel[5] == 0)
									&& (kernel[3] * kernel[5] * kernel[7] == 0))
								{
									delete_list.push_back(Point(i, j));
								}
							}
						}
					}
				}
			}
			int size = delete_list.size();
			if (size == 0)
			{
				break;
			}
			for (int n = 0; n < size; n++)
			{
				Point tem;
				tem = delete_list[n];
				uchar* data = srcimage.ptr<uchar>(tem.y);
				data[tem.x] = 0;
			}
			delete_list.clear();
			for (int j = 1; j < nl - 1; j++)
			{
				uchar* data_pre = srcimage.ptr<uchar>(j - 1);
				uchar* data = srcimage.ptr<uchar>(j);
				uchar* data_next = srcimage.ptr<uchar>(j + 1);
				for (int i = 1; i < (nc - 1); i++)
				{
					if (data[i] == 255)
					{
						kernel[0] = 1;
						if (data_pre[i] == 255) kernel[1] = 1;
						else  kernel[1] = 0;
						if (data_pre[i + 1] == 255) kernel[2] = 1;
						else  kernel[2] = 0;
						if (data[i + 1] == 255) kernel[3] = 1;
						else  kernel[3] = 0;
						if (data_next[i + 1] == 255) kernel[4] = 1;
						else  kernel[4] = 0;
						if (data_next[i] == 255) kernel[5] = 1;
						else  kernel[5] = 0;
						if (data_next[i - 1] == 255) kernel[6] = 1;
						else  kernel[6] = 0;
						if (data[i - 1] == 255) kernel[7] = 1;
						else  kernel[7] = 0;
						if (data_pre[i - 1] == 255) kernel[8] = 1;
						else  kernel[8] = 0;

						B = 0;
						for (int k = 1; k < 9; k++)
						{
							B = B + kernel[k];
						}
						if ((B >= 2) && (B <= 6))
						{
							A = 0;
							if (!kernel[1] && kernel[2]) A++;
							if (!kernel[2] && kernel[3]) A++;
							if (!kernel[3] && kernel[4]) A++;
							if (!kernel[4] && kernel[5]) A++;
							if (!kernel[5] && kernel[6]) A++;
							if (!kernel[6] && kernel[7]) A++;
							if (!kernel[7] && kernel[8]) A++;
							if (!kernel[8] && kernel[1]) A++;
							//
							if (A == 1)
							{
								if ((kernel[1] * kernel[3] * kernel[7] == 0)
									&& (kernel[1] * kernel[5] * kernel[7] == 0))
								{
									delete_list.push_back(Point(i, j));
								}
							}
						}
					}
				}
			}
			if (size == 0)
			{
				break;
			}
			for (int n = 0; n < size; n++)
			{
				Point tem;
				tem = delete_list[n];
				uchar* data = srcimage.ptr<uchar>(tem.y);
				data[tem.x] = 0;
			}
			delete_list.clear();
		}
	}

	void LineFinder::fitted_curve(cv::Mat& fittingline, cv::Vec4f& line)
	{
		std::vector<cv::Point> points;
		std::vector<cv::Vec4i>::const_iterator it2 = lines.begin();
		while (it2 != lines.end())
		{
			cv::Point pt1((*it2)[0], (*it2)[1]);
			points.push_back(pt1);
			cv::Point pt2((*it2)[2], (*it2)[3]);
			points.push_back(pt2);
			++it2;
		}

		cv::fitLine(points, line,
			cv::DIST_L2,
			0, 
			0.01, 0.01); 

		cv::Scalar color1 = cv::Scalar(0, 0, 255);

		// int x0 = line[2]; // start point
		// int y0 = line[3];
		// int x1 = x0 + 1500 * line[0]; // end point
		// int y1 = y0 + 1500 * line[1]; 
        float laser_line_k = line(1) / line(0);
        float laser_line_b = line(3) - line(2)* laser_line_k;

        float u_0 = 0;
        float v_0 = laser_line_k* 0 + laser_line_b;
        float u_1 = 2000; 
        float v_1 = laser_line_k* 2000 + laser_line_b;

		std::cout << "u_0, v_0: " << u_0 << " , " << v_0 << "u_1, v_1： " << u_1 << " , " << v_1 << std::endl;
		cv::line(fittingline, cv::Point(u_0, v_0), cv::Point(u_1, v_1), Scalar(0, 255, 255),
			3);
	}

    void MYRESIZE(cv::Mat& imagesrc, cv::Mat& imagedst)
    {
        double fScale = 0.2;//缩放系数
		//计算目标图像的大小
		cv::Size dsize = cv::Size(imagesrc.cols*fScale, imagesrc.rows*fScale);
		cv::resize(imagesrc, imagedst, dsize);
    }

	void LineFinder::finish(Mat image, cv::Vec4f& line)
	{
		int n = 230;   
		cv::Mat thresholded;  
        cv::Mat image_dst;

        cv::namedWindow("origin_image", 0.2); 
        MYRESIZE(image, image_dst); 
        cv::imshow("origin_image", image_dst);                     
		cv::threshold(image, thresholded, n,         
			255,                                     
			cv::THRESH_BINARY);                          
		cv::Mat image_drawed = image;
		image = thresholded;

		setMinVote(60);
		setLineLengthAndGap(600, 20);
		std::vector<cv::Vec4i> liness = findLines(image);

		Mat fittingline;
		fittingline.create(image.size(), image.type());
		fittingline = Scalar(0, 0, 255);
		fitted_curve(image_drawed, line);
		std::cout << "quickopencv line: " << line << std::endl;
        MYRESIZE(image_drawed, image_dst); 
        cv::namedWindow("drawed_line", 0.2);
        cv::imshow("drawed_line", image_dst);
        cv::waitKey(0);
	}

	void  LineFinder::final(Mat image)
	{
		Mat result;
		Mat image1 = image;
		Mat result1;
		Mat imageROI1;
		Mat imageROI2;
		Mat out1 = Log_filter(image);
		cv::GaussianBlur(image, result, cv::Size(5, 5), 0);   
		Mat thresholded;
		cv::threshold(result, thresholded, 240,        
			255,                                     
			cv::THRESH_BINARY);                          
		Mat res = thresholded;
		zhangSkeleton(res);
		detectrect(res, imageROI1, imageROI2);

		double EX1 = 10000, EX2 = 10000.00;

		std::vector<cv::Point> points1;
		std::vector<cv::Point> points2;

		detectpoint(imageROI1, points1);
		detectpoint(imageROI2, points2);

		cv::Vec4f lineaa1;
		cv::Vec4f lineaa2;
		cv::fitLine(points1, lineaa1,
			cv::DIST_L2,
			0, 
			0.01, 0.01); 
		cv::fitLine(points2, lineaa2,
			cv::DIST_L2, 
			0, 
			0.01, 0.01); 
		Mat zero1;;
		zero1.create(image.size(), image.type());
		zero1 = Scalar(0, 0, 255);
		drawfittingLines(zero1, lineaa1);
		drawfittingLines(zero1, lineaa2);
		drawfittingLines(image, lineaa1);
		drawfittingLines(image, lineaa2);
		drawfittingLines(res, lineaa1);
		drawfittingLines(res, lineaa2);
		namedWindow("line11", 0.5);
		imshow("line11", zero1);
		namedWindow("line", 0.5);
		imshow("line", image);
		namedWindow("li", 0.5);
		imshow("li", res);
		namedWindow("imageROI1", 0.5);
		imshow("imageROI1", imageROI1);
		namedWindow("imageROI2", 0.5);
		imshow("imageROI2", imageROI2);
	}



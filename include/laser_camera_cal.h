#ifndef __XYF_CAL_LIB_H__
#define __XYF_CAL_LIB_H__

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "quickopencv.h"

using namespace std;

#define CV_TERMCRIT_ITER    1
#define CV_TERMCRIT_NUMBER  CV_TERMCRIT_ITER
#define CV_TERMCRIT_EPS     2

namespace calibration
{

class LaserCameraCal
{
	public:
		typedef struct CorssPointGroup
		{
			Eigen::Vector2f cross_point;
			bool point_mode; // true: 2+1  false: 1+2
			Eigen::Vector2f points[3];
		}CorssPointGroup;

	    struct Parameters
	    {
		    int corner_rows;
		    int corner_cols;
		    int cornersize_rows;
		    int cornersize_cols;
		    string fold_path;
			string line_image_path;
			float len_chessborad;

			void operator()(const Parameters & my_struct)
		    {
			   fold_path = my_struct.fold_path;
			   corner_cols = my_struct.corner_cols;
			   corner_rows = my_struct.corner_rows;
			   cornersize_cols = my_struct.cornersize_cols;
			   cornersize_rows = my_struct.cornersize_rows;
			   line_image_path = my_struct.line_image_path;
			   len_chessborad = my_struct.len_chessborad;
		    }
	    };

	   	struct output
	   	{
		   vector<cv::Mat> tvecsMat;
		   vector<cv::Mat> rvecsMat;
		   cv::Mat cameraMatrix;
		   cv::Mat distCoeffs;
	   	};

	   	LaserCameraCal() = delete;
		LaserCameraCal(Parameters my_data)
	    {
			cm_data_.fold_path = my_data.fold_path;
			cm_data_.corner_cols = my_data.corner_cols;
			cm_data_.corner_rows = my_data.corner_rows;
			cm_data_.cornersize_cols = my_data.cornersize_cols;
			cm_data_.cornersize_rows = my_data.cornersize_rows;
			cm_data_.line_image_path = my_data.line_image_path;
			cm_data_.len_chessborad = my_data.len_chessborad;
	    }

		std::vector<vector<cv::Point2f>> GetImageCorners()
		{
			return image_points_seq_;
		}

		std::vector<vector<cv::Point3f>> GetBoardCorners()
		{
			return object_points_;
		}

		void ComputeImageCrossPoints(vector<cv::Point2f> &one_image_point, Eigen::Vector4f light_points, std::vector<cv::Point2f>& cross_points);

		void SelectThreeNeigborPoints(vector<cv::Point2f> one_image_pxpoint, vector<cv::Point3f> one_image_cbpoint, std::vector<cv::Point2f>& cross_points,  
			std::vector<cv::Point2f>& points_group, std::vector<cv::Point2f>& useful_crosspoints, vector<cv::Point3f>& b_cbpoint);

		void ComputeAllImageCorssPoints();

		void Get3dPoints(int index, struct output cm_output, std::vector<vector<cv::Point3f>> object_points_, std::vector<cv::Point2f>& points_group, 
			std::vector<cv::Point2f>& useful_crosspoints, std::vector<Eigen::Vector3f>& cc_points, vector<cv::Point3f>& b_cbpoint);

		void ResizeImage(cv::Mat& imagesrc, cv::Mat& imagedst);

		void DetectLine(cv::Mat imagesrc);

		bool DetectLine();

		output getcmdata(void);

   	private:
		struct Parameters cm_data_;
		vector<cv::Point2f> corners_;
		vector<vector<cv::Point2f>> image_points_seq_;
		vector<vector<cv::Point3f>> object_points_;
		std::vector<std::vector<CorssPointGroup> > cross_points_;

		cv::Mat image_drawcorners_;
	    cv::Mat image_undistort_;
	    vector<cv::Mat> tvecsMat_;  /* transmat */
	    vector<cv::Mat> rvecsMat_;  /* revesmat*/ 
		uint32_t src_image_size_;
		vector<cv::Mat> image_undistorts_;
		// LineFinder line_finder_;
		vector<cv::Vec4f> light_vecs_;
};

}

#endif // __XYF_CAL_LIB_H__
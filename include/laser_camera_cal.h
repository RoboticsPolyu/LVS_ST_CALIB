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

enum CAMERA_MODEL: uint8_t
{
	FISHEYE = 0,
	PINHOLE,
};

class LaserCameraCal
{
	public:
	    struct Parameters
	    {
		    int corner_rows;
		    int corner_cols;
		    int cornersize_rows;
		    int cornersize_cols;
		    string fold_path;
			string line_image_path;
			float len_chessborad;
			uint8_t camera_model;

			void operator()(const Parameters & my_struct)
		    {
			   fold_path = my_struct.fold_path;
			   corner_cols = my_struct.corner_cols;
			   corner_rows = my_struct.corner_rows;
			   cornersize_cols = my_struct.cornersize_cols;
			   cornersize_rows = my_struct.cornersize_rows;
			   line_image_path = my_struct.line_image_path;
			   len_chessborad = my_struct.len_chessborad;
			   camera_model = my_struct.camera_model;
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
		
		LaserCameraCal(Parameters parameter)
	    {
			parameter_(parameter);
	    }

		std::vector<vector<cv::Point2f>> GetImageCorners()
		{
			return image_corners_seq_;
		}

		std::vector<vector<cv::Point3f>> GetBoardCorners()
		{
			return object_points_;
		}

		const Parameters& GetParams() const
		{
			return parameter_;
		};

		void ResizeImage(cv::Mat& imagesrc, cv::Mat& imagedst);

		void DetectLine(cv::Mat imagesrc);

		bool DetectLine();

		output MultiImageCalibrate(void);

		void ComputeLaserPoint(int idx, float u, float v, float& z_c, float& x_c, float& y_c);

		void UndistortPoints(const std::vector<cv::Point2f>& in, std::vector<cv::Point2f> &out) const;

		const int GetValidImageSize() const 
		{
			return image_size_;
		}

		/**
		 * p_a p_b p_c ; plane paramters
		 *  u, v : laser point
		 *  x_c, y_c, z_c: 3d position at camera coordinate 
		 */
		void ComputeLaserPoint(float p_a, float p_b, float p_c, float u, float v, float x_c, float y_c, float z_c);
   	private:
		struct Parameters parameter_;
		vector<vector<cv::Point2f>> image_corners_seq_;
		vector<vector<cv::Point3f>> object_points_;
	    
	    vector<cv::Mat> tvecsMat_;  /* transmat */
	    vector<cv::Mat> rvecsMat_;  /* revesmat*/ 

		// LineFinder line_finder_;
		vector<cv::Vec4f> light_vecs_;
		Eigen::Matrix3f   camera_eigen_matrix_;
		cv::Mat 		  dist_coeffs_;
		cv::Mat           camera_matrix_;

		int image_size_;
		uint8_t camera_model_;
};

}

#endif // __XYF_CAL_LIB_H__ 
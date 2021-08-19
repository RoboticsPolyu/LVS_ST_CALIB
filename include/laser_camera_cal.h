#ifndef __LASER_CAMERA_CAL_H__
#define __LASER_CAMERA_CAL_H__

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

#include "types.h"
#include "laser_line_detector.h"

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
		    float cornersize_rows;
		    float cornersize_cols;
		    string fold_path;
			string line_image_path;
			string intrinsic_file;
			uint8_t camera_model;

			void operator()(const Parameters & my_struct)
		    {
			   fold_path = my_struct.fold_path;
			   corner_cols = my_struct.corner_cols;
			   corner_rows = my_struct.corner_rows;
			   cornersize_cols = my_struct.cornersize_cols;
			   cornersize_rows = my_struct.cornersize_rows;
			   line_image_path = my_struct.line_image_path;
			   camera_model = my_struct.camera_model;
			   intrinsic_file = my_struct.intrinsic_file;
		    }

			void print()
			{
				std::cout << "**********************************************" << std::endl;
				std::cout << "Laser Camera Calibration Parameter: " << std::endl;
				std::cout << "corner_cols: " << corner_cols << std::endl;
				std::cout << "corner_rows: " << corner_rows << std::endl;
				std::cout << "cornersize_cols: " << cornersize_cols << std::endl;
				std::cout << "cornersize_rows: " << cornersize_rows << std::endl;
				std::cout << "fold_path: " << fold_path << std::endl;
				std::cout << "line_image_path: " << line_image_path << std::endl;
				std::cout << "camera_mode: " << (uint32_t)camera_model << std::endl;
				std::cout << "intrinsic file: " << intrinsic_file << std::endl;
				std::cout << "**********************************************" << std::endl;
			}
	    };

	   	LaserCameraCal()
		{
		}
		
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

		bool LoadParameter(const std::string& filename);

		void ResizeImage(cv::Mat& imagesrc, cv::Mat& imagedst);

		void DetectLine(cv::Mat imagesrc);

		bool DetectLine();

		void MultiImageCalibrate(void);

		void ComputeLaserPoint(int idx, float u, float v, float& z_c, float& x_c, float& y_c);

		void UndistortPoints(const std::vector<cv::Point2f>& in, std::vector<cv::Point2f> &out) const;

		const int GetValidImageSize() const 
		{
			return image_size_;
		}

		/**
		 * p_a p_b p_c ; plane paramters
		 * p_a*x + p_b*y + p_c = z
		 *  u, v : laser point
		 *  x_c, y_c, z_c: 3d position at camera coordinate 
		 */
		void ComputeLaserPoint(float p_a, float p_b, float p_c, float u, float v, float x_c, float y_c, float z_c);
		
		double ComputeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
												const vector<vector<Point2f> >& imagePoints,
												const vector<Mat>& rvecs, const vector<Mat>& tvecs,
												const Mat& cameraMatrix , const Mat& distCoeffs,
												vector<float>& perViewErrors, bool fisheye);

   	private:
		struct Parameters parameter_;
		vector<vector<cv::Point2f>> image_corners_seq_;
		vector<vector<cv::Point3f>> object_points_;
	    
	    vector<cv::Mat> tvecsMat_;  /* transmat */
	    vector<cv::Mat> rvecsMat_;  /* revesmat*/ 

		// LineFinder line_finder_;
		vector<cv::Vec4f> light_vecs_;
		Eigen::Matrix3f   camera_eigen_matrix_;
		Eigen::VectorXf   dist_eigen_coeffs_;
		cv::Mat 		  dist_coeffs_;

		int image_size_;

		bool LoadCameraMatrix(Eigen::Matrix3f& camera_matrix);

		bool SaveCameraMatrix(const Eigen::Matrix3f& camera_matrix);
};

}

#endif // __LASER_CAMERA_CAL_H__ 
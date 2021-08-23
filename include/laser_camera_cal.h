#ifndef __LASER_CAMERA_CAL_H__
#define __LASER_CAMERA_CAL_H__

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

#include "types.h"

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
		    int     corner_rows;
		    int     corner_cols;
		    float   cornersize_rows;
		    float   cornersize_cols;
		    string  fold_path;
			string  line_image_path;
			uint8_t camera_model;
			bool    use_intrinsic;

			void operator()(const Parameters & my_struct)
		    {
			   fold_path = my_struct.fold_path;
			   corner_cols = my_struct.corner_cols;
			   corner_rows = my_struct.corner_rows;
			   cornersize_cols = my_struct.cornersize_cols;
			   cornersize_rows = my_struct.cornersize_rows;
			   line_image_path = my_struct.line_image_path;
			   camera_model = my_struct.camera_model;
			   use_intrinsic = my_struct.use_intrinsic;
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
				std::cout << "use intrinsic: " << (use_intrinsic ? "true" : "false") << std::endl;
				std::cout << "**********************************************" << std::endl;
			}
	    };

		struct StraightLine
		{
			float k;
			float b;
		};

	   	LaserCameraCal()
		{
			dist_coeffs_ = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
			camera_eigen_matrix_.setZero();
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

		void MultiImageCalibration(void);

		void ComputeLaserPoint(int idx, float u, float v, float& z_c, float& x_c, float& y_c);

		void UndistortPoints(const std::vector<cv::Point2f>& in, std::vector<cv::Point2f> &out) const;

		const int GetValidImageSize() const 
		{
			return image_size_;
		}

		void LaserLineDetector();
		
		void GetLaserLines(std::vector<StraightLine>& straight_lines);

		bool LaserLineDetector(cv::Mat& src_image, StraightLine& straght_line);
		
		/**
		 * p_a p_b p_c ; plane paramters
		 * p_a*x + p_b*y + p_c = z
		 *  u, v : laser point
		 *  x_c, y_c, z_c: 3d position at camera coordinate 
		 */
		void ComputeLaserPoint(float p_a, float p_b, float p_c, float u, float v, float x_c, float y_c, float z_c);
		
		double ComputeReprojectionErrors( const vector<vector<cv::Point3f> >& objectPoints,
												const vector<vector<cv::Point2f> >& imagePoints,
												const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
												const cv::Mat& cv_K , const cv::Mat& distCoeffs,
												vector<float>& perViewErrors, bool fisheye);

   	private:
		bool SaveCameraMatrix(const Eigen::Matrix3f& camera_matrix);

	    struct Parameters parameter_;
		
		std::vector<vector<cv::Point2f>> image_corners_seq_;
		std::vector<vector<cv::Point3f>> object_points_;
	    
	    std::vector<cv::Mat> tvecsMat_;  /* transmat */
	    std::vector<cv::Mat> rvecsMat_;  /* revesmat*/ 

		std::vector<StraightLine> straight_lines_;
		
		Eigen::Matrix3f   camera_eigen_matrix_;
		cv::Mat           dist_coeffs_;
		
		float             k1_, k2_, p1_, p2_, k3_;            

		int image_size_;	
	
	public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif // __LASER_CAMERA_CAL_H__ 
#ifndef __XYF_CAL_LIB_H__
#define __XYF_CAL_LIB_H__

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;

namespace calibration
{

class cm_xyf
{
	public:
		typedef struct CorssPointGroup
		{
			Eigen::Vector2f corss_point;
			bool point_mode; // true: 2+1  false: 1+2
			Eigen::Vector2f points[3];
		}CorssPointGroup;

	    struct MyStruct
	    {
		    int corner_rows;
		    int corner_cols;
		    int corners_ize_rows;
		    int corners_ize_cols;
		    string fold_path;

			void operator()(const MyStruct & my_struct)
		    {
			   fold_path = my_struct.fold_path;
			   corner_cols = my_struct.corner_cols;
			   corner_rows = my_struct.corner_rows;
			   corners_ize_cols = my_struct.corners_ize_cols;
			   corners_ize_rows = my_struct.corners_ize_rows;
		    }
	    };

	   	struct output
	   	{
		   vector<cv::Mat> tvecsMat;
		   cv::Mat rvecsMat;
		   cv::Mat cameraMatrix;
		   cv::Mat distCoeffs;
	   	};

	   	cm_xyf() = delete;
		cm_xyf(MyStruct my_data)
	   {
		   cm_data_(my_data);
	   }

		std::vector<vector<cv::Point2f>> GetImageCorners()
		{
			return image_points_seq_;
		}

		void ComputeImageCrossPoints(vector<cv::Point2f> one_image_point, Eigen::Vector4f light_points)
		{

		}

		// ComputeImageCrossPoints(image_points_seq_[0], light_points);

		void ComputeAllImageCorssPoints();

   	private:
		struct MyStruct cm_data_;
		vector<cv::Point2f> corners_;
		vector<vector<cv::Point2f>> image_points_seq_;
		std::vector<std::vector<CorssPointGroup> > cross_points_;

		cv::Mat image_drawcorners_;
	    cv::Mat image_undistort_;
	    vector<cv::Mat> tvecsMat_;  /* transmat */
	    vector<cv::Mat> rvecsMat_;  /* revesmat*/
	    cv::Mat res_mat_; 

	public:
		output getcmdata(void);
};

}

#endif // __XYF_CAL_LIB_H__
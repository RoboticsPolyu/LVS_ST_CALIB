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
			Eigen::Vector2f cross_point;
			bool point_mode; // true: 2+1  false: 1+2
			Eigen::Vector2f points[3];
		}CorssPointGroup;

	    struct MyStruct
	    {
		    int corner_rows;
		    int corner_cols;
		    int cornersize_rows;
		    int cornersize_cols;
		    string fold_path;

			void operator()(const MyStruct & my_struct)
		    {
			   fold_path = my_struct.fold_path;
			   corner_cols = my_struct.corner_cols;
			   corner_rows = my_struct.corner_rows;
			   cornersize_cols = my_struct.cornersize_cols;
			   cornersize_rows = my_struct.cornersize_rows;
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
		   // cm_data_(my_data);
		   	
			   cm_data_.fold_path = my_data.fold_path;
			   cm_data_.corner_cols = my_data.corner_cols;
			   cm_data_.corner_rows = my_data.corner_rows;
			   cm_data_.cornersize_cols = my_data.cornersize_cols;
			   cm_data_.cornersize_rows = my_data.cornersize_rows;

		   std::cout << "cm_data_. cornersize_cols: " << cm_data_.cornersize_rows << std::endl;
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

		void SelectThreeNeigborPoints(vector<cv::Point2f> one_image_pxpoint, vector<cv::Point3f> one_image_cbpoint, std::vector<cv::Point2f>& cross_points,  std::vector<cv::Point2f>& points_group, std::vector<cv::Point2f>& useful_crosspoints, vector<cv::Point3f>& b_cbpoint);
		// ComputeImageCrossPoints(image_points_seq_[0], light_points);

		void ComputeAllImageCorssPoints();
		void Get3dPoints(struct output cm_output, std::vector<vector<cv::Point3f>> object_points_, std::vector<cv::Point2f>& points_group, std::vector<cv::Point2f>& useful_crosspoints, std::vector<Eigen::Vector3f>& cc_points, vector<cv::Point3f>& b_cbpoint);

   	private:
		struct MyStruct cm_data_;
		vector<cv::Point2f> corners_;
		vector<vector<cv::Point2f>> image_points_seq_;
		vector<vector<cv::Point3f>> object_points_;
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
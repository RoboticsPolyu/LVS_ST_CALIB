#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;



namespace calibration
{

struct output
{
	vector<cv::Mat> tvecsMat;
	vector<cv::Mat> rvecsMat;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
}; 

class cm_xyf
{
   public:
	   cv::Mat image_original;
	   cv::Mat image_drawcorners;
	   cv::Mat image_undistort;
	   vector<cv::Mat> tvecsMat;  /* transmat */
	   vector<cv::Mat> rvecsMat;  /* revesmat*/
	   cv::Mat res_mat;

	   struct MyStruct
	   {
		   string image_path;
		   string post_path_1;
		   string post_path_2;
		   int corner_rows;
		   int corner_cols;
		   int cornersize_rows;
		   int cornersize_cols;

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
		   cm_data_.corner_cols = my_data.corner_cols;
		   cm_data_.corner_rows = my_data.corner_rows;
		   cm_data_.image_path = my_data.image_path;
		   cm_data_.post_path_1 = my_data.post_path_1;
		   cm_data_.post_path_2 = my_data.post_path_2;
		   cm_data_.cornersize_rows = my_data.cornersize_rows;
		   cm_data_.cornersize_cols = my_data.cornersize_cols;
	   }

   private:
	  struct MyStruct cm_data_;
	  vector<cv::Point2f> corners;
	  vector<vector<cv::Point2f>> image_points_seq;
	 

	public:
		output getcmdata(void);
};

}
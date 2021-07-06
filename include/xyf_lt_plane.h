#include <iostream>
#include <Eigen/Dense>
#include "xyf_cal_lib.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "light_cal.h"

using namespace std;

namespace calibration{
    class ltplane_xyf
    {
        public:
            typedef struct ltplane_output
            {
             Eigen::Vector3f pc_1;
             Eigen::Vector3f pc_2;
             //Eigen::Vector3f pc_3;   
                
            }lt_output;

            struct ltplane_input
            {
             vector<cv::Mat> tvecsMat_;
		     cv::Mat rvecsMat_;
		     cv::Mat cameraMatrix;
		     cv::Mat distCoeffs;
             cv::Mat points;
            };

            //lt_xyf(const cv::Mat& camera_intrinsic, cv::Mat& rot_matrix, cv::Mat& trans, cv::Mat& points);
            ltplane_xyf(ltplane_input lightplane_cal)
	   {
		   lightplane_cal.tvecsMat_ = light_cal_.tvecsMat_;
		   lightplane_cal.rvecsMat_ = light_cal_.rvecsMat_;
		   lightplane_cal.cameraMatrix = light_cal_.cameraMatrix;
		   lightplane_cal.distCoeffs = light_cal_.distCoeffs;
		   lightplane_cal.points = light_cal_.points;
	   }
       
           lt_output getltdata(void);


        private:
            ltplane_input light_cal_;

    };
}
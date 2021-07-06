#include <iostream>
#include <Eigen/Dense>
#include "xyf_cal_lib.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;

namespace calibration
{
    class lt_xyf
    {
        public:
            typedef struct lt_output
            {
             Eigen::Vector3f pc_1;
             Eigen::Vector3f pc_2;
             //Eigen::Vector3f pc_3;   
                
            }lt_output;

            struct lt_input
            {
             vector<cv::Mat> tvecsMat;
		     cv::Mat rvecsMat;
		     cv::Mat cameraMatrix;
		     cv::Mat distCoeffs;
             cv::Mat points;
            };

            //lt_xyf(const cv::Mat& camera_intrinsic, cv::Mat& rot_matrix, cv::Mat& trans, cv::Mat& points);
            lt_xyf(lt_input light_cal)
	   {
		   light_cal.tvecsMat = light_cal_.tvecsMat;
		   light_cal.rvecsMat = light_cal_.rvecsMat;
		   light_cal.cameraMatrix = light_cal_.cameraMatrix;
		   light_cal.distCoeffs = light_cal_.distCoeffs;
		   light_cal.points = light_cal_.points;
	   }
       
           lt_output getltdata(void);


        private:
            lt_input light_cal_;

    };
}
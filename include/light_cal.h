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
                    vector<cv::Mat> tvecsMat_;
                    cv::Mat rvecsMat_;
                    cv::Mat cameraMatrix;
                    cv::Mat distCoeffs;
                    cv::Mat points;
            };

            //lt_xyf(const cv::Mat& camera_intrinsic, cv::Mat& rot_matrix, cv::Mat& trans, cv::Mat& points);
            lt_xyf(lt_input light_cal, vector<vector<cv::Point2f>> image_points_seq)
            {
                light_cal.tvecsMat_ = light_cal_.tvecsMat_;
                light_cal.rvecsMat_ = light_cal_.rvecsMat_;
                light_cal.cameraMatrix = light_cal_.cameraMatrix;
                light_cal.distCoeffs = light_cal_.distCoeffs;
                light_cal.points = light_cal_.points;

                image_points_seq_.assign(image_points_seq.begin(), image_points_seq.end());
            }
        
            lt_output getltdata(void);

        private:
            lt_input light_cal_;
            vector<vector<cv::Point2f>> image_points_seq_;
    };
}
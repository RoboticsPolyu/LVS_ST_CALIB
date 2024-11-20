#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>

namespace slam
{
    class charuco_create_detect
    {
    private:
        /* data */
    public:
        charuco_create_detect(/* args */)
        {
        }
        
        ~charuco_create_detect()
        {
        }
        
        void createBoard();
        
        void detectCharucoBoardWithCalibrationPose();
        
        void detectCharucoBoardWithoutCalibration();

        void detectCharucoBoard(cv::Mat& image, std::vector<cv::Point2f>& charucoCorners, std::vector<int>& charucoIds);
    };

};

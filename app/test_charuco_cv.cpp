#include "charuco_create_detect.h"

int main()
{
    std::string filename1, filename2;
    filename1 = "/home/yang/Downloads/1.bmp";
    filename2 = "/home/yang/Downloads/2.bmp";
    
    slam::charuco_create_detect charuco_detecter;
    cv::Mat image;
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    image = cv::imread(filename1);
    charuco_detecter.detectCharucoBoard(image, charucoCorners, charucoIds);
    image = cv::imread(filename2);
    charuco_detecter.detectCharucoBoard(image, charucoCorners, charucoIds);
    return 0;
}
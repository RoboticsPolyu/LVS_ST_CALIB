#include "daheng_camera_wrapper.h"

using namespace std;
using namespace sensors::camera;

int main(void)
{
    std::cout << "###################### daheng camera test ######################" << std::endl;
    cv::Mat daheng_camera_image;
    CameraData camera_data;

    DahengCamera daheng_camera;
    daheng_camera.InitCameraLib();
    daheng_camera.OpenCamera();
    daheng_camera.GetImg(camera_data);
    std::cout << "get one image " << std::endl;

    cv::imwrite("daheng_camera_test.jpg", camera_data.img);
    std::cout << "camera frame id: " << camera_data.index << std::endl;

    daheng_camera.GetImg(camera_data);
    std::cout << "get one image " << std::endl;

    cv::imwrite("daheng_camera_test1.jpg", camera_data.img);
    std::cout << "camera frame id: " << camera_data.index << std::endl;

    daheng_camera.CloseCamera();
    daheng_camera.CloseLib();

    return 0;
}
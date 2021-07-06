#include "daheng_camera_wrapper.h"

using namespace std;
using namespace sensors::camera;

int main(void)
{
    std::cout << "###################### daheng camera test ######################" << std::endl;
    cv::Mat daheng_camera_image;
    CameraData camera_data;

    DahengCamera daheng_camera;
    if(!daheng_camera.InitCameraLib())
    {
        std::cout << "failed to init camera lib" << std::endl;
        return 0;
    };
    if(!daheng_camera.OpenCamera())
    {
        std::cout << "failed to open camera" << std::endl;
        return 0;
    };

    uint64_t t_last;
    cv::namedWindow("daheng_camera");
    while(true)
    {
        daheng_camera.GetImg(camera_data);
        cv::imshow("daheng_camera", camera_data.img);

        std::cout << "get one image " << std::endl;
        // cv::imwrite("daheng_camera_test.jpg", camera_data.img);
        std::cout << std::fixed << "camera frame id: " << camera_data.index << std::endl;
        std::cout << std::fixed << "camera frame t: " << camera_data.timestamp << std::endl;
        std::cout << std::fixed << "camera frame t diff: " << camera_data.timestamp - t_last << std::endl;

        t_last = camera_data.timestamp;
    }

    daheng_camera.CloseCamera();
    daheng_camera.CloseLib();

    return 0;
}
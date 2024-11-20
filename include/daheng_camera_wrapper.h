#ifndef DAHENG_CAMERA_H__
#define DAHENG_CAMERA_H__

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "sensors/camera/DxImageProc.h"
#include "sensors/camera/GxIAPI.h"

using namespace std;

namespace sensors
{
    namespace camera
    {
        struct CameraData
        {
            /* data */
            uint64_t timestamp;
            int index;
            cv::Mat img;
        };
        

        class DahengCamera
        {
        private:
            GX_DEV_HANDLE m_hDevice;

            unsigned int iLastError;
            bool m_bOpen;
            int index = 0;

        public:
            DahengCamera(/* args */);
            ~DahengCamera();

            bool InitCameraLib();
            bool CloseLib();
            bool OpenCamera();
            bool CloseCamera();
            cv::Mat GetImg();
            bool GetImg(cv::Mat &pic);
            bool GetImg(CameraData &pic);
            // CameraData GetImg();
            int GetLastError();
            bool GetCameraState();
            int GetCount();
        };
    }
}

#endif
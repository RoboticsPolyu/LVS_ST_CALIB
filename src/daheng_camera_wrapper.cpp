#include "daheng_camera_wrapper.h"

using namespace sensors::camera;

DahengCamera::DahengCamera(/* args */)
{
}

DahengCamera::~DahengCamera()
{
}

//初始化相机的SDK库
bool DahengCamera::InitCameraLib()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    emStatus = GXInitLib();
    if (emStatus != GX_STATUS_SUCCESS)
    {
        return false;
        iLastError = 10001;
    }
    else
    {
        return true;
    }
}

//关闭相机的SDK库
bool DahengCamera::CloseLib()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXCloseDevice(m_hDevice);
    if (status == GX_STATUS_SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
        iLastError = 00000;
    }
}

//开启相机
bool DahengCamera::OpenCamera()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t nDeviceNum = 0;
    GX_OPEN_PARAM stOpenParam;

    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if (status == GX_STATUS_SUCCESS && nDeviceNum > 0)
    {
        // GX_DEVICE_IP_INFO stIPInfo;
        //获取第一台设备的网络信息
        // status = GXGetDeviceIPInfo(1, &stIPInfo);
        // if (status != GX_STATUS_SUCCESS)
        // {
        //     iLastError = 10003;
        //     return false;
        // }

        stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE; //访问设备的方式: 只读、控制、独占等
        stOpenParam.openMode = GX_OPEN_INDEX;         //通过枚举序号打开
        stOpenParam.pszContent = "1";                 //枚举序号

        status = GXOpenDevice(&stOpenParam, &m_hDevice);
        if (status == GX_STATUS_SUCCESS)
        {
            m_bOpen = true;
            return true;
        }
        else
        {
            iLastError = 10004;
            return false;
        }
    }
    else
    {
        iLastError = 10002;
        return false;
    }
}

bool DahengCamera::CloseCamera()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXStreamOff(m_hDevice);
    if (status == GX_STATUS_SUCCESS)
    {
        status = GXCloseDevice(m_hDevice);
        if (status == GX_STATUS_SUCCESS)
	        return true;
        else
        {
            iLastError = 10010;
            return false;
        }
    }
    else
    {
	    iLastError = 10009;
	    return false;
    }
}

//获取图像
cv::Mat DahengCamera::GetImg()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    PGX_FRAME_BUFFER pFrameBuffer;
    cv::Mat a(1, 1, CV_8UC1);
    if (m_bOpen == true)
    {
        status = GXStreamOn(m_hDevice);
        if (status == GX_STATUS_SUCCESS)
        {
            status = GXDQBuf(m_hDevice, &pFrameBuffer, 1000);
            if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
            {
                // QImage img = ShowPic(pFrameBuffer);
                int width = pFrameBuffer->nWidth;
                int height = pFrameBuffer->nHeight;

                cv::Mat img(cv::Size(width, height), CV_8UC1, (void *)pFrameBuffer->pImgBuf, cv::Mat::AUTO_STEP);

                status = GXQBuf(m_hDevice, pFrameBuffer);
                if (status != GX_STATUS_SUCCESS)
                {
                    iLastError = 10008;
                }
                return img;
            }
            else
            {
                iLastError = 10007;
                return a;
            }
        }
        else
        {
            iLastError = 10006;
            return a;
        }
    }
    else
    {
        iLastError = 10005;
        return a;
    }
}

bool DahengCamera::GetImg(cv::Mat &pic)
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    PGX_FRAME_BUFFER pFrameBuffer;
    if (m_bOpen == true)
    {
        status = GXStreamOn(m_hDevice);
        if (status == GX_STATUS_SUCCESS)
        {
            status = GXDQBuf(m_hDevice, &pFrameBuffer, 1000);
            if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
            {
                int width = pFrameBuffer->nWidth;
                int height = pFrameBuffer->nHeight;

                cv::Mat img(cv::Size(width, height), CV_8UC1, (void *)pFrameBuffer->pImgBuf, cv::Mat::AUTO_STEP);
                pic = img;

                status = GXQBuf(m_hDevice, pFrameBuffer);
                if (status != GX_STATUS_SUCCESS)
                {
                    iLastError = 10008;
                }

                return true;
            }
            else
            {
                iLastError = 10007;
                return false;
            }
        }
        else
        {
            iLastError = 10006;
            return false;
        }
    }
    else
    {
        iLastError = 10005;
        return false;
    }
}

bool DahengCamera::GetImg(CameraData &pic)
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    PGX_FRAME_BUFFER pFrameBuffer;
    if (m_bOpen == true)
    {
        status = GXStreamOn(m_hDevice);
        if (status == GX_STATUS_SUCCESS)
        {
            status = GXDQBuf(m_hDevice, &pFrameBuffer, 1000);
            if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
            {
                int width = pFrameBuffer->nWidth;
                int height = pFrameBuffer->nHeight;

                cv::Mat img(cv::Size(width, height), CV_8UC1, (void *)pFrameBuffer->pImgBuf, cv::Mat::AUTO_STEP);

                // GXGetInt(m_hDevice, GX_INT_EVENT_EXPOSUREEND_TIMESTAMP, );
                pic.index = pFrameBuffer->nFrameID;
                pic.img = img;
                pic.timestamp = pFrameBuffer->nTimestamp;

                status = GXQBuf(m_hDevice, pFrameBuffer);
                if (status != GX_STATUS_SUCCESS)
                {
                    iLastError = 10008;
                }

                return true;
            }
            else
            {
                iLastError = 10007;
                return false;
            }
        }
        else
        {
            iLastError = 10006;
            return false;
        }
    }
    else
    {
        iLastError = 10005;
        return false;
    }
}
//获取最后一个错误

int DahengCamera::GetLastError()
{
    return iLastError;
}

bool DahengCamera::GetCameraState()
{
    return m_bOpen;
}

int DahengCamera::GetCount()
{
    return index;
}
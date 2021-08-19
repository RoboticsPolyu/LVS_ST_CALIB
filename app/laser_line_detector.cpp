#include <laser_camera_cal.h>    

int main(void)
{
    calibration::LaserCameraCal::Parameters parameters;
    parameters.corner_rows = 5;
    parameters.corner_cols = 8;
    parameters.cornersize_rows = 7.23;
    parameters.cornersize_cols = 7.23;
    parameters.camera_model = calibration::CAMERA_MODEL::PINHOLE;
    parameters.fold_path = "/home/yang/Dataset/image_fold3";
    parameters.line_image_path = "/home/yang/Dataset/image_fold3/light_fold";
    parameters.intrinsic_file = "../config/config.yaml";

  	calibration::LaserCameraCal LaserCameraCal_instance(parameters);
    LaserCameraCal_instance.MultiImageCalibrate();
    LaserCameraCal_instance.DetectLine();

    return 0;
}
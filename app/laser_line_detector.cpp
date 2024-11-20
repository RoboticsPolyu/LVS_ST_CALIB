#include <laser_camera_cal.h>    

int main(void)
{
    calibration::LaserCameraCalib::Parameters parameters;
    parameters.corner_rows = 7;
    parameters.corner_cols = 10;
    parameters.cornersize_rows = 3;
    parameters.cornersize_cols = 3;
    parameters.camera_model = calibration::CAMERA_MODEL::PINHOLE;
    parameters.fold_path = "/home/yang/Downloads/demo_Laser_welding1/build/dataset/20210915_LC/image";
    parameters.line_image_path = "/home/yang/Downloads/demo_Laser_welding1/build/dataset/20210915_LC/laser";
    
  	calibration::LaserCameraCalib laser_camera_calib(parameters);
    laser_camera_calib.Calibrate();
    laser_camera_calib.LaserLineDetector();

    return 0;
}
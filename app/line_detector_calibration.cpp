#include <laser_camera_cal.h>    

int main(void)
{
    calibration::LaserCameraCal::Parameters parameters;
    ofstream pattern_3d_point_fs;
    pattern_3d_point_fs.open("pattern_3dpoint.txt");

    /* internal paramater calibration*/
    parameters.fold_path = "/home/yang/image_fold";
    parameters.line_image_path = "/home/yang/image_fold/light_fold";
    parameters.corner_rows = 9;
    parameters.corner_cols = 6;
    parameters.cornersize_rows = 6.85;
    parameters.cornersize_cols = 6.85;
    parameters.len_chessborad =  6.85;

  	calibration::LaserCameraCal LaserCameraCal_instance(parameters);
    LaserCameraCal_instance.DetectLine();

    return 0;
}
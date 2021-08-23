/* Laser Camera Calibration 
   Laser Plane : abc_coeff
*/

#include <laser_camera_cal.h>    

int main(void)
{
   calibration::LaserCameraCal laser_camera_calib;
     
   laser_camera_calib.LoadParameter("../config/slc_config.yaml");
   laser_camera_calib.MultiImageCalibration();

   return 0;
}
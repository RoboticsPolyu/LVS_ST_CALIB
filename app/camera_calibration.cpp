/* Laser Camera Calibration 
   Laser Plane : abc_coeff
*/

#include <laser_camera_cal.h>    

int main(void)
{
   calibration::LaserCameraCalib laser_camera_calib;
     
   laser_camera_calib.LoadParameter("../config/config.yaml");
   laser_camera_calib.Calibrate();

   return 0;
}
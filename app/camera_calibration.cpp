/* Laser Camera Calibration 
   Laser Plane : abc_coeff
*/

#include <laser_camera_cal.h>    

int main(void)
{
   calibration::LaserCameraCal LaserCameraCal_instance;
     
   LaserCameraCal_instance.LoadParameter("../config/config.yaml");
   LaserCameraCal_instance.MultiImageCalibrate();

   return 0;
}
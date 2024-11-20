/* Laser Camera Calibration 
   Laser Plane : abc_coeff
*/

#include <laser_camera_cal.h>    

int main(void)
{
    std::ofstream pattern_3d_point_fs;
    pattern_3d_point_fs.open("pattern_3dpoint.txt");

  	calibration::LaserCameraCalib laser_camera_calib;
    laser_camera_calib.LoadParameter("../config/slc_config.yaml");
    laser_camera_calib.Calibrate();

    Eigen::Vector4f light_points;
    int valid_image_size = laser_camera_calib.GetValidImageSize();
    laser_camera_calib.LaserLineDetector();
    
    std::vector<Eigen::Vector3f> lt_3dpoint_all_plane;
    std::vector<calibration::LaserCameraCalib::StraightLine> straight_lines;
    laser_camera_calib.GetLaserLines(straight_lines);
    assert(valid_image_size == straight_lines.size());

    laser_camera_calib.CalibrateWithCrossRatio();

    for(int i = 0; i < valid_image_size; i++)
    {
        std::cout << "------------------------------------- image index: " << i << "-----------------------" << std::endl;

        std::vector<Eigen::Vector3f> lt_3dpoint;
        float x_c, y_c, z_c;
        uint uv_step = 5;
        float laser_line_k = straight_lines[i].k;
        float laser_line_b = straight_lines[i].b;

        std::cout << "laser_k: " << laser_line_k << " ,laser_b: " << laser_line_b << std::endl;

        std::vector<cv::Point2f> point_uv, point_uv_distorted;
        for(int j = 0; j < 2000; j = j+5)
        {
            float u_j = j;
            float v_j = laser_line_k* j + laser_line_b;
            cv::Point2f point_uv_j(u_j, v_j);
            point_uv.push_back(point_uv_j);
        }

        // laser_camera_calib.UndistortPoints(point_uv, point_uv_distorted);

        for(int j = 0; j< point_uv.size(); j++)
        {
            cv::Point2f point_distorted = point_uv[j];
            laser_camera_calib.ComputeLaserPoint(i, point_distorted.x, point_distorted.y, z_c, x_c, y_c);
            pattern_3d_point_fs << x_c << " " << y_c << " " << z_c << std::endl;
            Eigen::Vector3f uv_xyz(x_c, y_c, z_c);
            lt_3dpoint_all_plane.push_back(uv_xyz);
        }
    }

    Eigen::MatrixXf H_matrix = Eigen::MatrixXf::Zero(lt_3dpoint_all_plane.size(), 3);
    Eigen::VectorXf Z_vector = Eigen::VectorXf::Zero(lt_3dpoint_all_plane.size());

    for(int i = 0; i < lt_3dpoint_all_plane.size(); i++)
    {
        Eigen::Vector3f point = lt_3dpoint_all_plane[i];
        H_matrix(i, 0) = point(0);
        H_matrix(i, 1) = point(1);
        H_matrix(i, 2) = 1.0f;
        Z_vector(i) = point(2);
    }
    // pa*x + pb*y + 1 = pc*z

    Eigen::Vector3f abc_coeff = (H_matrix.transpose()* H_matrix).inverse()* H_matrix.transpose()* Z_vector;
    std::cout << "abc_coeff: \n" << abc_coeff <<std::endl;

    return 0;
}
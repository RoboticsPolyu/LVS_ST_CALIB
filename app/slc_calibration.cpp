/* Laser Camera Calibration 
   Laser Plane : abc_coeff
*/

#include <laser_camera_cal.h>    

int main(void)
{
    ofstream pattern_3d_point_fs;
    pattern_3d_point_fs.open("pattern_3dpoint.txt");

    calibration::LaserCameraCal::Parameters parameters;
    parameters.corner_rows = 8;
    parameters.corner_cols = 5;
    parameters.cornersize_rows = 7.23;
    parameters.cornersize_cols = 7.23;
    parameters.len_chessborad =  7.23; //7.23
    parameters.camera_model = calibration::CAMERA_MODEL::PINHOLE;
    parameters.fold_path = "/home/yang/image_fold";
    parameters.line_image_path = "/home/yang/image_fold/light_fold";

  	calibration::LaserCameraCal LaserCameraCal_instance(parameters);

    LaserCameraCal_instance.MultiImageCalibrate();

    cv::FileStorage laser_line_file("detect_line.yaml" ,cv::FileStorage::READ);
    Eigen::Vector4f light_points;
    int valid_image_size = LaserCameraCal_instance.GetValidImageSize();

    std::vector<Eigen::Vector3f> lt_3dpoint_all_plane;
    for(int i = 0; i < valid_image_size; i++)
    {
        std::cout << "------------------------------------- image index: " << i << "-----------------------" << std::endl;
        cv::Vec4f vec4f ;
        laser_line_file["line_vec4f_" + std::to_string(i)] >> vec4f;
        std::cout << "light_points: " << vec4f << std::endl;
        cv::cv2eigen(vec4f, light_points);
        
        std::vector<Eigen::Vector3f> lt_3dpoint;
        float x_c, y_c, z_c;
        uint uv_step = 5;
        float laser_line_k = light_points(1) / light_points(0);
        float laser_line_b = light_points(3) - light_points(2)* laser_line_k;

        std::cout << "laser_k: " << laser_line_k << " ,laser_b: " << laser_line_b << std::endl;

        std::vector<cv::Point2f> point_uv, point_uv_distorted;

        for(int j = 0; j < 2000; j = j+5)
        {
            float u_j = j;
            float v_j = laser_line_k* j + laser_line_b;
            cv::Point2f point_uv_j(u_j, v_j);
            point_uv.push_back(point_uv_j);
        }

        LaserCameraCal_instance.UndistortPoints(point_uv, point_uv_distorted);

        for(int j = 0; j< point_uv_distorted.size(); j++)
        {
            cv::Point2f point_distorted = point_uv_distorted[j];
            LaserCameraCal_instance.ComputeLaserPoint(i, point_distorted.x, point_distorted.y, z_c, x_c, y_c);
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

    Eigen::Vector3f abc_coeff = (H_matrix.transpose()* H_matrix).inverse()* H_matrix.transpose()* Z_vector;
    std::cout << "abc_coeff: \n" << abc_coeff <<std::endl;

    laser_line_file.release();

    return 0;
}
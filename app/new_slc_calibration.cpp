#include <laser_camera_cal.h>    

int main(void)
{
    calibration::LaserCameraCal::Parameters parameters;
    ofstream pattern_3d_point_fs;
    pattern_3d_point_fs.open("pattern_3dpoint.txt");

    /* internal paramater calibration*/
    parameters.fold_path = "/home/yang/image_fold";
    parameters.line_image_path = "/home/yang/image_fold/light_fold";
    parameters.corner_rows = 8; //8
    parameters.corner_cols = 5; //5
    parameters.cornersize_rows = 6.85;
    parameters.cornersize_cols = 6.85;
    parameters.len_chessborad =  6.85; //6.85

  	calibration::LaserCameraCal LaserCameraCal_instance(parameters);

    calibration::LaserCameraCal::output cameramat = LaserCameraCal_instance.MultiImageCalibrate();
    std::cout << "camera valid r/t matrix size: " << cameramat.rvecsMat.size() << std::endl;

    cv::FileStorage temp_file1("detect_line.yaml" ,cv::FileStorage::READ);

    Eigen::Vector4f light_points;
    int valid_image_size = LaserCameraCal_instance.GetValidImageSize();

    std::vector<Eigen::Vector3f> lt_3dpoint_all_plane;
    for(int i = 0; i < valid_image_size; i++)
    {
        std::cout << "------------------------------------- image index: " << i << "-----------------------" << std::endl;
        cv::Vec4f vec4f ;
        temp_file1["line_vec4f_" + std::to_string(i)] >> vec4f;
        std::cout << "light_points: " << vec4f << std::endl;
        cv::cv2eigen(vec4f, light_points);
        
        std::vector<Eigen::Vector3f> lt_3dpoint;
        float x_c, y_c, z_c;
        uint uv_step = 5;
        float laser_line_k = light_points(1) / light_points(0);
        float laser_line_b = light_points(3) - light_points(2)* laser_line_k;

        float u_0 = 0;
        float v_0 = laser_line_k* 0 + laser_line_b;
        float u_1 = 1000; 
        float v_1 = laser_line_k* 1000 + laser_line_b;

        std::cout << "laser_k: " << laser_line_k << " ,laser_b: " << laser_line_b << "v_0: " << v_0 << " , v_1: " << v_1 << std::endl;

        std::vector<cv::Point2f> point_uv;
        std::vector<cv::Point2f> point_uv_distorted;

        for(int j = 0; j < 2000; j = j+5)
        {
            float u_j = j;
            float v_j = laser_line_k* j + laser_line_b;
            cv::Point2f point_uv_j(u_j, v_j);
            // std::cout << "point uv j: " << point_uv_j << std::endl;
            point_uv.push_back(point_uv_j);
        }

        // for(int j = 0; u_0 + j* uv_step < u_1 && v_0 + j* uv_step < v_1; j++)
        // {
        //     float u_j = u_0 + j* uv_step;
        //     float v_j = v_0 + j* uv_step;
        //     cv::Point2f point_uv_j(u_j, v_j);
        //     point_uv.push_back(point_uv_j);
        // }

        // LaserCameraCal_instance.UndistortPoints(point_uv, point_uv_distorted);

        for(int j = 0; j< point_uv.size(); j++)
        {
            cv::Point2f point_distorted = point_uv[j];
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

    temp_file1.release();

    return 0;
}
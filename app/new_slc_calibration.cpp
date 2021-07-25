#include <laser_camera_cal.h>    

int main(void)
{
    calibration::LaserCameraCal::Parameters parameters;
    ofstream pattern_3d_point_fs;
    pattern_3d_point_fs.open("pattern_3dpoint.txt");

    /* internal paramater calibration*/
    parameters.fold_path = "/home/yang/image_fold";
    parameters.line_image_path = "/home/yang/image_fold/light_fold";
    parameters.corner_rows = 8;
    parameters.corner_cols = 6;
    parameters.cornersize_rows = 3; //
    parameters.cornersize_cols = 3;
    parameters.len_chessborad =  3;
  	calibration::LaserCameraCal LaserCameraCal_instance(parameters);
    // LaserCameraCal_instance.DetectLine();

    std::cout << "Start get 3d point " << std::endl;
    calibration::LaserCameraCal::output cameramat = LaserCameraCal_instance.getcmdata();
    
    cv::FileStorage temp_file1("detect_line.yaml" ,cv::FileStorage::READ);

    vector<cv::Point3f> b_cbpoint;
    vector<vector<cv::Point2f>> one_image_corners_point = LaserCameraCal_instance.GetImageCorners();
    vector<vector<cv::Point3f>> one_image_cb_point = LaserCameraCal_instance.GetBoardCorners();
    Eigen::Vector4f light_points;
    std::vector<cv::Point2f> cross_pointsdata;

    for(int i = 0; i < one_image_corners_point.size(); i++)
    {
        std::cout << "------------------------------------- image index: " << i << "-----------------------" << std::endl;

        cv::Vec4f vec4f ;
        temp_file1["line_vec4f_" + std::to_string(i)] >> vec4f;
        std::cout << "light_points: " << vec4f << std::endl;
        cv::cv2eigen(vec4f, light_points);

        // LaserCameraCal_instance.ComputeImageCrossPoints(one_image_corners_point[i], light_points , cross_pointsdata);
    
        // std::vector<cv::Point2f> points_group;
        // std::vector<cv::Point2f> useful_points;
        
        std::vector<Eigen::Vector3f> lt_3dpoint;
        float x_c, y_c, z_c;
        uint uv_step = 100;
        float Laser_line_k = (light_points(3) - light_points(2)) / (light_points(1) - light_points(0));
        float u_0 = light_points(0);
        float v_0 = light_points(1);
        float u_1 = light_points(2);
        float v_1 = light_points(3);

        for(int j = 0; u_0 + j* uv_step < u_1 && v_0 + j* uv_step < v_1; j++)
        {
            LaserCameraCal_instance.ComputeLaserPoint(i, u_0 + j* uv_step, v_0 + j* uv_step, z_c, x_c, y_c);
            pattern_3d_point_fs << x_c << " " << y_c << " " << z_c << std::endl;
        }

        // std::cout << "xyz_c: " << x_c << " " << y_c << " " << z_c << std::endl;

        // LaserCameraCal_instance.SelectThreeNeigborPoints(one_image_corners_point[i], one_image_cb_point[i], cross_pointsdata , points_group , useful_points , b_cbpoint);
        // LaserCameraCal_instance.Get3dPoints(i, cameramat, points_group, useful_points, lt_3dpoint, b_cbpoint);

        // cout << "points_group" << points_group << endl;
        // cout << "useful_points" << useful_points << endl;
        // cout << "b_cbpoint" << b_cbpoint << endl;
        // for(int i = 0; i < lt_3dpoint.size(); i++)
        // {
        //    cout << "lt_3dpoint" << lt_3dpoint[i] << endl; 
        //    // pattern_3d_point_fs << lt_3dpoint[i][0] << " " << lt_3dpoint[i][1] << " " << lt_3dpoint[i][2] << std::endl;
        //    pattern_3d_point_fs << x_c << " " << y_c << " " << z_c << std::endl;
        // }
        
    }

    temp_file1.release();

    return 0;
}
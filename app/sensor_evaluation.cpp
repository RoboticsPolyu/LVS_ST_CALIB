#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "gtsam_wrapper.h"
#include "hand_eye_calibration.h"
#include "laser_camera_cal.h"

using namespace std;
using namespace Eigen;

double cal_stdev(std::vector<double>& resultSet)
{
    double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
    double mean =  sum / resultSet.size(); //均值

    double accum  = 0.0;
    std::for_each (std::begin(resultSet), std::end(resultSet), [&](const double d) {
        accum  += (d-mean)*(d-mean);
    });
    
    double stdev = sqrt(accum/(resultSet.size()-1)); //方差
    return stdev;
}

int main(int argc, char **argv) 
{
    YAML::Node config = YAML::LoadFile("../config/config.yaml");
    std::string egm_file = config["feature_egm_path"].as<std::string>();
    std::string feature_file = config["feature_file_path"].as<std::string>(); // The features of the seam in the image coordiantes 
    int weld_type = config["weld_type"].as<int>();
    
    float p_a= config["PA"].as<float>();
    float p_b= config["PB"].as<float>();
    float p_c= config["PC"].as<float>();

    std::ofstream laser_point_file;
    laser_point_file.open("data_line.txt", std::ios_base::trunc);

    std::ifstream file;
    file.open(egm_file);
    if(!file.is_open())
    {
        std::cout << "Not open Egm file: " << std::endl;
        return 0;
    }

    std::string str;
    std::istringstream iss;
    double timestamp, x, y, z, ex, ey, ez, qw, qx, qy, qz;
    std::vector<gtsam::Pose3> sensors, robots;

    while(std::getline(file, str))
    {
        iss.clear();
        iss.str(str);
        iss >> timestamp >> x >> y >> z >> ex >> ey >> ez >> qw >> qx >> qy >> qz;
        Quaterniond q(qw, qx, qy, qz);
        gtsam::Pose3 robot(gtsam::Rot3(q.matrix()), Eigen::Vector3d(x, y, z));
        robots.push_back(robot);
    }
    file.close();
    std::cout << "robot size " << robots.size() << std::endl;
    gtsam::Vector6 extrinsic;
    float dx = 0, dy = 0, dz = 0; // -1 2 -3
    // extrinsic << -0.576009, 0.566958, -1.48899, -72.6375, -78.8219, -97.0048; // 1125_AM_HE_BEF_OPT laser_point_3
    // extrinsic << -0.576968702274, 0.550427235324, -1.45439924825, -73.9165276181, -76.7874378633, -96.6609315984; // 1125_AM_HE_AFTER_OPT laser_point_2
    // extrinsic << -0.571813596271, 0.582504536057, -1.51798630798, -72.049116392 +dx, -74.4696765119+dy, -87.0965539537+dz; //r1

    // extrinsic << -0.608952120656, 0.592284120089, -1.48827685014, -77.7292887373, -72.8984147222, -84.9920224549; // r2
    // extrinsic << -0.562436596, 0.616238938914, -1.48823465114, -70.3869737546, -76.0375654058, -87.4195790369; // r3
    //extrinsic << -0.5598042514, 0.582083563342, -1.46377790677, -73.979784963 +dx, -75.4260364391+dy, -93.6139079339+dz; // 1023_HE_OPT
    // extrinsic << -0.573411, 0.573156, -1.48322, -73.9481, -75.7107, -93.6198; // 1023_HE_NoOPT
    

    extrinsic << -0.554722, 0.582764, -1.44886, -72.3921, -75.665, -96.1937;
    extrinsic << -0.613005, 0.562166, -1.59555, -269.992, -361.77, 356.383;
    extrinsic << -0.578923, 0.50754, -1.58706, -238.966, -311.298, 405.128;
    
    extrinsic << -0.593319, 0.507215, -1.59645, -237.985, 314.382, 404.964;

    // 20220510ZHEXIAN
    extrinsic <<  -0.0168107, -0.00907457, -1.57754, -82.8209, -133.297, 179.427;

    // 20220601LINE
    extrinsic <<  0.0646647, 0.097089, -1.5925, -84.8866, -137.017, 172.925;
    extrinsic << 0.0820617, 0.0547728, -1.4617, -84.385, -128.897, 175.354;
    // 20220601LINE_OPT
    extrinsic << 0.0188481, -0.00479824, -1.61063, -45.9144, -53.1572, -120.839;
    // extrinsic << -0.0141829, 0.161787, -1.60783, -96.5182, -157.011, 166.219;

    // extrinsic << -0.0303417, 0.0472695, -1.66193, -52.4323, -55.4323, -126.273;
    uint32_t uv_lens = 0; // 48

    // float p_a= -0.00964276;
    // float p_b= 1.81429;
    // float p_c= 114.353;
    // float kx= 3597.8882;
    // float ky= 3599.5422;
    // float u0= 1211.2184;
    // float v0= 1007.1196;
    // float k1= -0.2307;
    // float k2= 0.1734;
    // float k3= -0.2950;
    // float p1= 0;
    // float p2= 0;

    float kx= 3671.816149167293;
    float ky= 3673.054767823902;
    float u0= 1202.05292881891;
    float v0= 1012.724880485216;
    float k1= -0.2247568439561717;
    float k2= 0.1827051018453399;
    float k3= -0.4200880191381967;
    float p1= 0;
    float p2= 0;

    //float ux[28] = {281, 2295, 489, 1868, 592, 576,920,1279,660,1018,388,732,1079,479,829,1174,573,229,920,290,654,1008,387,753,487,849, 135, 2212};
    //float vy[28] = {1015, 1114, 1030, 986, 945, 933,929,933,925,928,955,976,994,961,980,997,967,949,984,889,950,994,903,954,915,969, 961, 983};
    // float ux[uv_lens] = {1841, 1498, 1084, 958, 963, 608, 1434, 1769, 1493, 1148, 1348, 1225, 817, 708, 1200, 1382, 561, 221, 909, 1272, 920, 586, 1082, 1414, 833, 480, 1183, 1383};
    // float vy[uv_lens] = {1034, 1034, 1019, 1008, 1012, 994, 1054, 1070, 1008, 1017, 1015, 1008, 1092, 1104, 1115, 1111, 891, 883, 909, 911, 1094, 1083, 1113, 1124, 900, 916, 900, 886}; // 28
    //float ux[uv_lens] = {676,678,1125,1127,1063,1065,1011,1016,574,581,840,838,558,563,1651,1655,653,654,1561,1563,325,329,1445, 1445,266,2775,1341,1336,412,418,886,876,789,563,567,1436,1434,804,781,1372,1372,960,1431,1421,846,849,996,991};
    //float vy[uv_lens] = {1134,1130,1089,1085,1073,1073,1151,1153,1270,1273,1317,1313,1211,1211,1381,1380,1238,1241,1260,1256,1220, 1219,1349,1345,1225,1228,1086,1084,1100,1106,1079,1074,1104,1056,1057,1011,1004,1210,1107,1129,1129,1123,1040,1038,1097,1099,1134,1125};
    
    // The feature point pixel in the image coordinate
    std::vector<float> ux;
    std::vector<float> vy;

    // Read all feature points
    ifstream fin(feature_file);
    if(!fin) 
    {
        cout << "cannot find features file at " << feature_file << endl;
        return 1;
    }

    if(weld_type == 1)
    {
        std::cout << "weld type is JHF" << std::endl;
    }        
    else if(weld_type == 2)
    {
        std::cout << "weld type is C-type or S-type" << std::endl;
    }

    int index= 0;
    while (!fin.eof()) 
    {   
        if(weld_type == 1)
        {
            float f_u, f_v;
            fin >> f_u >> f_v;
            ux.push_back(f_u);
            vy.push_back(f_v);
        }
        else if(weld_type == 2)
        {
            //feature points of the seam includes left feature, center feature and right feature
            float f_left_u, f_left_v, f_center_u, f_center_v, f_right_u, f_right_v; 
            fin >> f_left_u >> f_left_v >> f_center_u >> f_center_v >> f_right_u >> f_right_v;
            ux.push_back(f_left_u); ux.push_back(f_center_u); ux.push_back(f_right_u);
            vy.push_back(f_left_v); vy.push_back(f_center_v); vy.push_back(f_right_v);
        }
        else
        {

        }

    }
    uv_lens = ux.size();
    std::cout << "laser strip point size " << uv_lens << std::endl;
    std::vector<cv::Point2f> image_points;
    for(uint idx = 0; idx < uv_lens; idx++)
    {
        cv::Point2f point(ux[idx], vy[idx]);
        image_points.push_back(point);
    }

    std::vector<cv::Point2f> undistort_points;
    std::vector<int32_t> inliers;
    cv::Mat cv_K = (cv::Mat_<float>(3, 3) << kx, 0, u0, 0, ky, v0, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) << k1, k2, p1, p2, k3);
    cv::undistortPoints(image_points, undistort_points, cv_K, distCoeffs, cv::Matx33d::eye(), cv_K);

    gtsam::Vector4 cam_P_corner[uv_lens];
    gtsam::Vector4 cam_P_corner_t;

    for(uint idx = 0; idx < uv_lens; idx++)
    {
        float    z_c = p_c* kx* ky /(kx* ky + p_a* ky* (u0- undistort_points[idx].x) + p_b* kx* (v0 - undistort_points[idx].y));
        float    x_c = z_c* (undistort_points[idx].x - u0)/ kx;
        float    y_c = z_c* (undistort_points[idx].y - v0)/ ky;

        cam_P_corner_t[0] = x_c;
        cam_P_corner_t[1] = y_c;
        cam_P_corner_t[2] = z_c;
        cam_P_corner_t[3] = 1;
        cam_P_corner[idx] = cam_P_corner_t;

        // std::cout << " CAM_XYZ: [" << x_c << " " << y_c << " " << z_c << " ]" << std::endl;
    }


    gtsam::Pose3 eTc_ = gtsam::Pose3::Expmap(extrinsic);
    gtsam::Point3 t = eTc_.translation();// + gtsam::Point3(0, 1.5, -2.3);
    gtsam::Pose3 eTc(eTc_.rotation(), t);

    std::vector<double> xs, ys, zs;
    std::vector<gtsam::Vector4> base_P_corners;
    gtsam::Point3 ee(0, -2.5, -0.5);

    for(uint idx = 0; idx < uv_lens; idx++)
    {
        gtsam::Vector4 base_P_corner;
        if(weld_type == 1)
        {
            base_P_corner = (robots[idx]* eTc).matrix()* cam_P_corner[idx];
        }
        else if(weld_type == 2)
        {
            base_P_corner = (robots[idx / 3]* eTc).matrix()* cam_P_corner[idx];
        }
        
        // gtsam::Point3 ee_fix = robots[idx].rotation()* ee;
        // base_P_corner.head(3) = base_P_corner.head(3) - ee_fix;

        // std::cout << "---> BASE_CORNER: [ " << base_P_corner[0] << " " << base_P_corner[1] << " " << base_P_corner[2] << " " << base_P_corner[3] << " ] <---" << std::endl;
        laser_point_file << base_P_corner[0] << " " << base_P_corner[1] << " " << base_P_corner[2] << " " << base_P_corner[3] << std::endl;
        
        // std::cout << robots[uv_lens+ idx%4].translation();
        // robots[uv_lens+ idx%4].translation() = robots[uv_lens+ idx%4].translation();

        // gtsam::Point3 error_ee_ = base_P_corner.head(3) - robots[uv_lens+ idx%4].translation();
        // gtsam::Point3 ee_t = robots[uv_lens+ idx%4].rotation().inverse()* error_ee_;
        // std::cout << "---- error_ee_： " << error_ee_[0] << " " << error_ee_[1] << " " << error_ee_[2] << std::endl;
        // std::cout << "---- ee_: " << ee_t[0] << " " << ee_t[1] << " " << ee_t[2] << std::endl;

        base_P_corners.push_back(base_P_corner);
    }    
    
    for(uint idx = 0; idx < base_P_corners.size()-1; idx++)
    {
        double dis = (base_P_corners[idx+1] - base_P_corners[idx]).norm();
        // std::cout << "dis: " << dis << std::endl;
    }
    // std::cout << "stdev: " << std::endl;
    // std::cout << cal_stdev(xs) << " " << cal_stdev(ys) << " " << cal_stdev(zs) << std::endl;
    return 0;
}
 
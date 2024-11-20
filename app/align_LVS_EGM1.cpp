#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "gtsam_wrapper.h"
#include "hand_eye_calibration.h"
#include "laser_camera_cal.h"
#include "td_factor.h"

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

int search_Nearest_Neighbor(std::vector<double> &egm_timestamps, double image_timestamp)
{
    for(uint i = 1; i < egm_timestamps.size(); i++)
    {
        if(image_timestamp >= egm_timestamps[i-1] && image_timestamp < egm_timestamps[i])
        {
            return i-1;
        }
    }

    return -1;
};

gtsam::Pose3 Interpolation(std::vector<double> &egm_timestamps, std::vector<gtsam::Pose3>& robots, double image_timestamp, uint& index)
{
    for(uint i = 1; i < egm_timestamps.size(); i++)
    {
        if(image_timestamp >= egm_timestamps[i-1] && image_timestamp < egm_timestamps[i])
        {
            index = i-1;
            gtsam::Pose3 inter_pose;
            inter_pose = robots[i-1].interpolateRt(robots[i], (image_timestamp - egm_timestamps[i-1]) / (egm_timestamps[i]-egm_timestamps[i-1]) );
            return inter_pose;
        }
    }

    return gtsam::Pose3::identity();
};

int main(int argc, char **argv) 
{
    YAML::Node config = YAML::LoadFile("../config/config.yaml");
    
    std::string egm_file = config["feature_egm_path"].as<std::string>();
    std::string feature_file = config["feature_file_path"].as<std::string>(); // The features of the seam in the image coordiantes 
    std::string image_timestamp = config["image_timestamp_path"].as<std::string>();

    double dt = config["DT"].as<double>();
    std::cout << "DT :" << dt << std::endl;
    float p_a= config["PA"].as<float>();
    float p_b= config["PB"].as<float>();
    float p_c= config["PC"].as<float>();
    bool enable_ext_opt = config["EXT_OPT"].as<bool>();
    bool enable_pl_opt  = config["PL_OPT"].as<bool>();
    bool enable_td_opt = config["TD_OPT"].as<bool>();

    float ERR_THR = config["ERROR_THR"].as<float>();
    uint32_t ITER_NUM = config["ITER_NUM"].as<uint32_t>();


    double e1 = config["ext1"].as<double>();
    double e2 = config["ext2"].as<double>();
    double e3 = config["ext3"].as<double>();
    double e4 = config["ext4"].as<double>();
    double e5 = config["ext5"].as<double>();
    double e6 = config["ext6"].as<double>();

    float kx= 3671.816149167293;
    float ky= 3673.054767823902;
    float u0= 1202.05292881891;
    float v0= 1012.724880485216;
    float k1= -0.2247568439561717;
    float k2= 0.1827051018453399;
    float k3= -0.4200880191381967;
    float p1= 0;
    float p2= 0;

    

    int weld_type = config["weld_type"].as<int>();
    float dt_test = config["dt_test"].as<float>();
    float kt = config["kt"].as<float>();
    float kp = config["kp"].as<float>();
    float kr = config["kr"].as<float>();
    float km = config["km"].as<float>();
    float ko = config["ko"].as<float>();
    
    std::ofstream laser_point_file;
    laser_point_file.open("data.txt", std::ios_base::trunc);
    std::ofstream opt;
    opt.open("opt.txt", std::ios_base::trunc);

    std::ifstream file;
    file.open(egm_file);

    if(!file.is_open())
    {
        std::cout << "Not open Egm file: " << std::endl;
        return 0;
    }

    std::string str;
    std::istringstream iss;
    double timestamp, x, y, z, ex, ey, ez, qw, qx, qy, qz, timestamp_f; // timestamp_f: ms
    std::vector<gtsam::Pose3> sensors, robots;

    std::vector<gtsam::Vector3> eWs; // robot angular speed and linear speed
    std::vector<gtsam::Vector3> bVs;

    std::vector<double> egm_timestamps;
    std::vector<double> egm_f_timestamps;
    double sec, nsec;

    while(std::getline(file, str))
    {
        iss.clear();
        iss.str(str);
        iss >> sec >> nsec >> x >> y >> z >> ex >> ey >> ez >> qw >> qx >> qy >> qz >> timestamp_f;
        // std::cout << "timestamp_f: " << timestamp_f << std::endl;
        egm_timestamps.push_back(sec + nsec / 1e9);
        egm_f_timestamps.push_back(timestamp_f/ 1e6);

        Quaterniond q(qw, qx, qy, qz);
        gtsam::Pose3 robot(gtsam::Rot3(q.matrix()), Eigen::Vector3d(x, y, z));
        robots.push_back(robot);
    }

    float EGM_T = 0.004; //s

    for(uint i = 0; i < robots.size()-1; i++)
    {
        gtsam::Pose3 roboti = robots[i];
        gtsam::Pose3 roboti_1 = robots[i+1];

        gtsam::Rot3 Ri = roboti.rotation();
        gtsam::Rot3 Ri_1 = roboti_1.rotation();

        gtsam::Vector3 ti = roboti.translation();
        gtsam::Vector3 ti_1 = roboti_1.translation();
        // std::cout << "--------------- wv --------------- \n";
        gtsam::Vector wi = gtsam::Rot3::Logmap(Ri.inverse()* Ri_1)/(EGM_T);
        gtsam::Vector vi = (ti_1 - ti)/EGM_T;

        // std::cout << "ti: \n" << ti << " \n ti+1: \n" << ti_1 << std::endl;

        eWs.push_back(wi);
        bVs.push_back(vi); // mm/s
        
        // std::cout << "vi: \n" << vi << std::endl;
    }

    std::cout << "EGM_TIMESTAMP:" << egm_timestamps.size() << std::endl;

    file.close();
    std::ifstream image_timestamp_file;
    image_timestamp_file.open(image_timestamp);

    if(!image_timestamp_file.is_open())
    {
        std::cout << "Not image timestamp file: " << std::endl;
        return 0;
    }
    
    // double dt = 0.05; //second
    std::vector<double> image_timestamps;
    std::vector<double> image_self_timestamps;
    double timestamp_image_f;
    uint32_t image_index;

    while(std::getline(image_timestamp_file, str))
    {
        // std::cout << str << std::endl;
        iss.clear();
        iss.str(str);
        iss >> sec >> nsec >> image_index >> timestamp_image_f;
        image_timestamps.push_back(sec + nsec/1e9 - dt_test);
        image_self_timestamps.push_back(timestamp_image_f / 1e9);
    }

    // for(uint i = 0; i < egm_f_timestamps.size(); i++)
    // {
    //     egm_timestamps[i] = egm_timestamps[0] + (egm_f_timestamps[i] - egm_f_timestamps[0]);
    // };

    // for(uint i = 0; i < image_timestamps.size(); i++)
    // {
    //     image_timestamps[i] = image_timestamps[0] + (image_self_timestamps[i] - image_self_timestamps[0]);
    // };
    
    std::cout << "image_timestamps:" << image_timestamps.size() << std::endl;

    gtsam::Vector6 extrinsic;
    // extrinsic << -0.576009, 0.566958, -1.48899, -72.6375, -78.8219, -97.0048; // 1125_AM_HE_BEF_OPT laser_point_3
    // extrinsic << -0.576968702274, 0.550427235324, -1.45439924825, -73.9165276181, -76.7874378633, -96.6609315984; // 1125_AM_HE_AFTER_OPT laser_point_2
    // extrinsic << -0.571813596271, 0.582504536057, -1.51798630798, -72.049116392 +dx, -74.4696765119+dy, -87.0965539537+dz; //r1

    // extrinsic << -0.608952120656, 0.592284120089, -1.48827685014, -77.7292887373, -72.8984147222, -84.9920224549; // r2
    // extrinsic << -0.562436596, 0.616238938914, -1.48823465114, -70.3869737546, -76.0375654058, -87.4195790369; // r3
    //extrinsic << -0.5598042514, 0.582083563342, -1.46377790677, -73.979784963 +dx, -75.4260364391+dy, -93.6139079339+dz; // 1023_HE_OPT
    // extrinsic << -0.573411, 0.573156, -1.48322, -73.9481, -75.7107, -93.6198; // 1023_HE_NoOPT
    // extrinsic << -0.554722, 0.582764, -1.44886, -72.3921, -75.665, -96.1937;
    // extrinsic << -0.613005, 0.562166, -1.59555, -269.992, -361.77, 356.383;
    // // extrinsic << -0.611198, 0.546635, -1.58999, -270.778, 361.326, 349.355;
    // extrinsic << -0.578923, 0.50754, -1.58706, -238.966, -311.298, 405.128;
    // extrinsic << -0.593319, 0.507215, -1.59645, -237.985, 314.382, 404.964;
    
    extrinsic << -0.0168107, -0.00907457, -1.57754, -82.8209, -133.297, 179.427;

    // 20220601LINE
    extrinsic <<  0.0646647, 0.097089, -1.5925, -84.8866, -137.017, 172.925;
    uint32_t uv_lens = 0; // 48

    // OPT 
    extrinsic << 0.0586852, 0.0700484, -1.62662, -80.6712, -138.461, 167.845;
    extrinsic <<  0.0590921, 0.0714098, -1.6344, -80.7538, -138.5, 167.402;
    // float p_a= -0.00964276;
    // float p_b= 1.81429;
    // float p_c= 114.353;
    extrinsic << e1, e2, e3, e4, e5, e6;
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
        // std::cout << ux[idx] << " " << vy[idx] << std::endl;
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

    gtsam::Pose3 eTc;
    gtsam::Pose3 eTc_ = gtsam::Pose3::Expmap(extrinsic);
    gtsam::Point3 t;

    t = eTc_.translation();
    eTc = gtsam::Pose3(eTc_.rotation(), t);

    if(enable_ext_opt)
    {
        t = eTc_.translation()  + gtsam::Point3(2, 3, 4);
        eTc = gtsam::Pose3(gtsam::Rot3::Expmap(gtsam::Vector3(0.03, 0.0, 0.03))* eTc_.rotation(), t);
    }
    else
    {
        t = eTc_.translation();
        eTc = gtsam::Pose3(eTc_.rotation(), t);
    }

    std::vector<double> xs, ys, zs;
    std::vector<gtsam::Vector4> base_P_corners;
    gtsam::Point3 ee(0, -2.5, -0.5);

    /*delay calibration test*/
    float k =  -0.02222632194863699;
    float b =  21.732792012553297;

    //20220611LINE
    k =  -0.025889153500810697;
    b =  27.843753407826355;
   // 2070.06 -78.2667 1141.02
   // 2325.91 -78.3193 1142.68
    // float pl_pl_dx = 0, pl_pl_dy = 0, pl_pl_dz = 0;
    // gtsam::Vector3 pl(2112.96+pl_pl_dx, k*2112.96+b + pl_pl_dy, 1137.35 + pl_pl_dz);
    // float pl_end_dx = 0, pl_end_dy = 0, pl_end_dz = 0;
    // gtsam::Vector3 pl_end(2357.49+ pl_end_dx, k*2357.49+b + pl_end_dy, 1136.58 + pl_end_dz);
    
    gtsam::Vector3 pl(2070.06, -78.2667, 1141.02);
    gtsam::Vector3 pl_end(2325.91, -78.3193, 1142.68);
    gtsam::Vector3 v = pl_end - pl;

    v = v/ v.norm();

    gtsam::Vector3 n = pl.cross(v); // - gtsam::Vector3(10, 100, 10);

    std::cout << "init n:\n " << n << std::endl;
    std::cout << "init v:\n " << v << std::endl;

    gtsam::Matrix33 rot3;
    rot3.col(0) = v;
    rot3.col(1) = n/ n.norm();
    rot3.col(2) = v.cross(n) / (n.norm());

    float m = n.norm();

    calibration::PluckerLine pll(gtsam::Rot3(rot3), m);
    uint32_t jsize = 4000;
    /***************************************************LM algrithm*******************************************************/
    gtsam::Matrix13 D1; D1.setZero();
    gtsam::Matrix14 D2; D2.setZero();
    Eigen::VectorXd J_td(jsize);
    Eigen::VectorXd J_error(jsize);
    Eigen::MatrixXd J_lw(jsize, 4);
    Eigen::VectorXd Last_J_error(jsize);
    Eigen::VectorXd Last_J_td(jsize);
    Eigen::MatrixXd Last_J_lw(jsize, 4);
    Eigen::MatrixXd J_R(jsize, 3);
    Eigen::MatrixXd J_P(jsize, 3);

    Eigen::MatrixXd J(jsize, 5);
    J.setZero();
    Eigen::MatrixXd Last_J(jsize, 5);
    Last_J.setZero();

    gtsam::Vector4 new_Lw;
    gtsam::Vector4 Last_delta_Lw;
    float delta_td = dt;
    gtsam::Vector4 dlw;
    gtsam::Vector5 dX;
    dX.setZero();

    new_Lw.head(3) = gtsam::Rot3::Logmap(gtsam::Rot3(rot3));
    new_Lw(3)  = m;
    std::cout << "Init LW: " << new_Lw << std::endl;

    // new_Lw << -0.0230096, -0.0219114, -0.0239913, 1093.34;
    // new_Lw << -0.0219197, 0.00432544, -0.0243542, 1151.09;
    float dw = 0;

    if(enable_pl_opt)
    {
        new_Lw << 0.0689244, -0.0064926, 1.80896e-05, 1130.25 + dw;
    }
    else
    {
        new_Lw <<  0.0689244, -0.0064926, 1.80896e-05, 1130.25;

    }
    

    double miu_td = 0, v_lm_td = 1;
    double tou = 1;
    double ro_td = 0;

    // LM for HandEye
    double miu_p = 0, v_lm_p = 1;
    double ro_p = 0;

    // LM for PL
    double miu_pl = 0, v_lm_pl = 1;
    double ro_pl = 0;

    double miu = 0, v_lm = 0;
    double ro = 0;

    double miu_max = 0;

    for(uint iter_num = 0; iter_num < ITER_NUM; iter_num++)
    {
        std::cout << "********************************** ITER: " << iter_num << "******************************" << std::endl;
        J_error.setZero();
        J_td.setZero();
        J_lw.setZero();
        J.setZero();
        J_R.setZero();
        J_P.setZero();

        std::vector<float> abs_distance;
        uint32_t meas_num = 0;
        float distance_2_sum = 0;

        for(uint i = 0; i < image_timestamps.size(); i++)
        {
            image_timestamps[i] = image_timestamps[i] - delta_td;
        }

        pll.Update(new_Lw);

        for(uint idx = 0; idx < uv_lens-2; idx++)
        {
            
            uint t_l_index = 0;

            gtsam::Vector4 base_P_corner;
            gtsam::Pose3 robot_pose_searched;

            if(weld_type == 1)
            {            
                // int egm_timestamps_idx = search_Nearest_Neighbor(egm_timestamps, image_timestamps[idx]);
                robot_pose_searched = Interpolation(egm_timestamps, robots, image_timestamps[idx], t_l_index);
                // if(robot_pose_searched == gtsam::Pose3::identity)
                // {
                //     std::cout << "Not Found Mearest Neighbor EGM data" << std::endl;
                //     return 0;
                // }
                base_P_corner = (robot_pose_searched* eTc).matrix()* cam_P_corner[idx];
            }
            else if(weld_type == 2)
            {
                base_P_corner = (robots[idx / 3]* eTc).matrix()* cam_P_corner[idx];
            }
            
            // gtsam::Point3 ee_fix = robots[idx].rotation()* ee;
            // base_P_corner.head(3) = base_P_corner.head(3) - ee_fix;
            gtsam::Vector3 pm(base_P_corner[0], base_P_corner[1], base_P_corner[2]);

            if(t_l_index == 0)
                continue;
            
            if(egm_timestamps[t_l_index+1] - egm_timestamps[t_l_index] > 0.005)
                continue;

            // std::cout << "******************************** IDX ********************************************" << std::endl;
            float error = pll.Distance(pm, D1, D2);
            // std::cout << "************** D *************** \n";
            // std::cout << "t_l_index: " << t_l_index << std::endl;
            // std::cout << "D_td: \n" << D1* bVs[t_l_index] << std::endl;
            
            // std::cout << "idx: " << idx <<  " ,pm: \n" << pm << std::endl;
            // std::cout << error << std::endl;

            if(error < ERR_THR)
            {
                // if(error >= 5)
                //     error = 5;
                /*HandEye Jacobian*/
                J_R.row(meas_num) = - D1* robot_pose_searched.rotation().matrix()* 
                                        gtsam::skewSymmetric(eTc.rotation().matrix()* 
                                            cam_P_corner[idx].head(3));

                J_P.row(meas_num) =   D1* robot_pose_searched.rotation().matrix();

                /*camera offset Jacobian*/
                J_td(meas_num) = D1*(-gtsam::skewSymmetric(robot_pose_searched.rotation().matrix()
                                        * eTc.rotation().matrix()* cam_P_corner[idx].head(3))
                                        * robots[t_l_index].rotation().matrix()
                                        * eWs[t_l_index] 
                                        - bVs[t_l_index]);
                J_error(meas_num) = error;

                /*Line Jacobian*/
                J_lw.row(meas_num) = D2;

                
                meas_num++;
                abs_distance.push_back(std::abs(error));
                distance_2_sum += error* error;
            }
            
            if(iter_num == ITER_NUM -1)
            {
            // std::cout << "---> BASE_CORNER: [ " << base_P_corner[0] << " " << base_P_corner[1] << " " << base_P_corner[2] << " " << base_P_corner[3] << " ] <---" << std::endl;
                laser_point_file << base_P_corner[0] << " " << base_P_corner[1] << " " << base_P_corner[2] << " " << base_P_corner[3] << " " << error << std::endl;
            }
            // std::cout << robots[uv_lens+ idx%4].translation();
            // robots[uv_lens+ idx%4].translation() = robots[uv_lens+ idx%4].translation();

            // gtsam::Point3 error_ee_ = base_P_corner.head(3) - robots[uv_lens+ idx%4].translation();
            // gtsam::Point3 ee_t = robots[uv_lens+ idx%4].rotation().inverse()* error_ee_;
            // std::cout << "---- error_ee_： " << error_ee_[0] << " " << error_ee_[1] << " " << error_ee_[2] << std::endl;
            // std::cout << "---- ee_: " << ee_t[0] << " " << ee_t[1] << " " << ee_t[2] << std::endl;

            base_P_corners.push_back(base_P_corner);
        }    
        
        J.block(0, 0, jsize, 1) = J_td;
        J.block(0, 1, jsize, 4) = J_lw;

        // std::cout << "J: \n" << J << std::endl;

        float abs_distance_sum = 0;
        for(uint index  = 0; index < abs_distance.size(); index++)
        {
            abs_distance_sum += abs_distance[index];
        }
        float mean_abs = abs_distance_sum / abs_distance.size();
        
        std::cout << "MEAS_SIZE: " << abs_distance.size() << std::endl;
        std::cout << "MEAN: " << mean_abs << std::endl;

        float rms = 0;
        for(uint index  = 0; index < abs_distance.size(); index++)
        {
            rms += (abs_distance[index] - mean_abs)* (abs_distance[index] - mean_abs);
        }
        std::cout << "RMS: " << std::sqrt(rms / abs_distance.size()) << std::endl;
        std::cout << "E2: " << std::sqrt(distance_2_sum / meas_num) << std::endl;

        for(uint idx = 0; idx < base_P_corners.size()-1; idx++)
        {
            double dis = (base_P_corners[idx+1] - base_P_corners[idx]).norm();
            // std::cout << "dis: " << dis << std::endl;
        }

        float dtt; 

        if(iter_num ==0)
        {
            miu_td = tou* J_td.dot(J_td);
            // Eigen::Matrix4d Hess_lw;
            // Hess_lw.setZero();
            // Hess_lw = J_lw.transpose()* J_lw;
            double max_element = 0;
            // for(int i = 0; i < 4; i++)
            // {false > max_element)
            //     {
            //         max_element = Hess_lw(i,i);
            //     }
            // }
            // miu_pl = max_element;
            
            Eigen::MatrixXd Hess(5, 5);
            Hess.setZero();

            Hess = J.transpose()* J;
            for(int i = 0; i < 5; i++)
            {
                if(Hess(i,i) > max_element)
                {
                    max_element = Hess(i,i);
                }
            }

            miu = max_element;
            miu_max = miu;
        }
        else if(iter_num > 0)
        {
            ro_td = (Last_J_error.dot(Last_J_error) - J_error.dot(J_error) )
            / (delta_td* (miu_td* delta_td - Last_J_td.dot(Last_J_error)) );

            std::cout << (0.5*(Last_J_error.transpose()* Last_J_error - J_error.transpose()* J_error)) << " , " 
            << 0.5* delta_td* (miu_td* delta_td - Last_J_td.dot(Last_J_error)) << std::endl;
            std::cout << "ro_td: \n" << ro_td << std::endl;
            
            if(ro_td > 0)
            {
                miu_td = miu_td* std::max(1.0/3, 1-(2*ro_td - 1)* (2*ro_td - 1)* (2*ro_td - 1));
                v_lm_td = 2;
            }
            else
            {
                miu_td = miu_td* v_lm_td; 
                v_lm_td = 2* v_lm_td;
            }
            
            ro_pl = (Last_J_error.dot(Last_J_error) - J_error.dot(J_error) )
            / (dlw.transpose()*(miu_pl* dlw - Last_J_lw.transpose()* Last_J_error) );
            std::cout << "ro_pl: \n" << ro_pl << std::endl;
            
            if(ro_pl > 0)
            {
                miu_pl = miu_pl* std::max(1.0/3, 1-(2*ro_pl - 1)* (2*ro_pl - 1)* (2*ro_pl - 1));
                v_lm_pl = 2;
            }
            else
            {
                miu_pl = miu_pl* v_lm_pl; 
                v_lm_pl = 2* v_lm_pl;
            }

            ro = (Last_J_error.dot(Last_J_error) - J_error.dot(J_error) )
            / (dX.transpose()*(miu* dX - Last_J.transpose()* Last_J_error) );
            std::cout << "ro: \n" << ro << std::endl;
            
            if(ro > 0)
            {
                miu = miu* std::max(1.0/3, 1-(2*ro - 1)* (2*ro - 1)* (2*ro - 1));
                v_lm = 2;
            }
            else
            {
                miu = miu* v_lm; 
                if(miu > miu_max)
                    miu = miu_max;
                v_lm = 2* v_lm;
            }
        }
        // cout << "miu_td: " << miu_td << " , v_lm_td: " << v_lm_td << std::endl;
        // std::cout << "miu_pl: " << miu_pl << " , v_lm_td: " << v_lm_td << std::endl;
        std::cout << "miu: " << miu << " , v_lm: " << v_lm << std::endl;
        gtsam::Matrix55 _I;
        _I.setIdentity();

        if(enable_td_opt)
        {
           dX = - (J.transpose()* J + miu* _I).inverse()* J.transpose()* J_error;
            delta_td = dX(0);
            dt = dt + delta_td;
            std::cout << "delta_td: " << delta_td << " , DT: " << dt << std::endl;
            std::cout << "dX: " << dX << std::endl; 
        }
        

        // if(enable_td_opt)
        // {
        //     dtt = - (J_td.transpose()* J_td + miu_td* gtsam::I_1x1).inverse()* J_td.transpose()* J_error;
        //     dt = dt + dtt;
        //     delta_td = dtt;
        //     std::cout << "Dtt: " << dtt << " , DT: " << dt << std::endl;
        // }
        // else
        // {
        //     delta_td = 0;
        // }
                
        // if(enable_pl_opt)
        // {
        //     dlw = -(J_lw.transpose()* J_lw + miu_pl* gtsam::I_4x4).inverse()* J_lw.transpose()* J_error;

        //     // dlw = -(J_lw.transpose()* J_lw + 0* gtsam::I_4x4).inverse()* J_lw.transpose()* J_error;

        //     std::cout << "Dlw: \n" << dlw << std::endl;

        //     gtsam::Rot3 new_R = gtsam::Rot3::Expmap(dlw.head(3))* gtsam::Rot3::Expmap(new_Lw.head(3));
        //     // rou = new_Lw(3);
        //     new_Lw(3) = new_Lw(3) + dlw(3);
        //     new_Lw.head(3) = gtsam::Rot3::Logmap(new_R);
        // }

        if(enable_pl_opt)
        {
            gtsam::Vector3 d_pl_r(dX(1), dX(2), dX(3));
            gtsam::Rot3 new_R = gtsam::Rot3::Expmap(d_pl_r)* gtsam::Rot3::Expmap(new_Lw.head(3));
            // rou = new_Lw(3);
            new_Lw(3) = new_Lw(3) + dX(4);
            new_Lw.head(3) = gtsam::Rot3::Logmap(new_R);            
        }


        gtsam::Vector3 dR = - (J_R.transpose()* J_R).inverse()* J_R.transpose()* J_error;
        gtsam::Vector3 dP = - (J_P.transpose()* J_P).inverse()* J_P.transpose()* J_error;
        
        std::cout << "new_Lw: \n" << new_Lw << std::endl;
        std::cout << "dR: \n" << dR << std::endl;
        std::cout << "dP: \n" << dP << std::endl;

        gtsam::Vector3 rotv = gtsam::Rot3::Logmap(eTc.rotation());

        opt << dtt << " " << dt << " " << new_Lw(0) << " " << new_Lw(1) << " " << new_Lw(2) << " " << new_Lw(3) << " " << std::sqrt(distance_2_sum / meas_num) << " " << 
        rotv[0] << " " << rotv[1] << " " << rotv[2] << " " << eTc.translation().x() << " " << eTc.translation().y() << " " << eTc.translation().z() << std::endl;

        Last_J_error  = J_error;
        Last_J_lw     = J_lw;
        Last_J_td     = J_td;
        Last_delta_Lw = dlw;
        // Last_J = J;

        float e_k = 0.2;
        
        if(enable_ext_opt)
        {
            gtsam::Point3 new_eP = eTc.translation()  + dP* kp;
            gtsam::Rot3 new_eR(gtsam::Rot3::Expmap(kr* dR).matrix()* eTc.rotation().matrix());
            eTc = gtsam::Pose3(new_eR, new_eP);
        }
            // gtsam::Point3 new_eP = eTc.translation()  + gtsam::Point3(dX(8), dX(9), dX(10) );
            // gtsam::Rot3 new_eR(gtsam::Rot3::Expmap(gtsam::Vector3(dX(5), dX(6), dX(7))).matrix()* eTc.rotation().matrix());
            // eTc = gtsam::Pose3(new_eR, new_eP);

        std::cout << "eTc Translation error: \n" << eTc_.translation() - eTc.translation() << std::endl;
    }

    std::cout << "eTc Rot3 old: \n" << gtsam::Rot3::Logmap(eTc_.rotation()) << std::endl;
    std::cout << "eTc Rot3 new: \n" << gtsam::Rot3::Logmap(eTc.rotation()) << std::endl;
    std::cout << "eTc t old: \n" << t << std::endl;
    std::cout << "eTc t new: \n" << eTc.translation() << std::endl;
    std::cout << "eTc logmap: \n" << gtsam::Pose3::Logmap(eTc) << std::endl;
    // std::cout << "stdev: "  << std::endl;
    // std::cout << cal_stdev(xs) << " " << cal_stdev(ys) << " " << cal_stdev(zs) << std::endl;
    return 0;
}
 
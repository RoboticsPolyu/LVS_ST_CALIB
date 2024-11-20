#include <bits/stdc++.h>
#include <Eigen/Core>
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "gtsam_wrapper.h"
#include "hand_eye_calibration.h"
#include "laser_camera_cal.h"
#include "td_factor.h"


int main()
{
    YAML::Node config = YAML::LoadFile("../config/config.yaml");
    
    std::string egm_file = config["feature_egm_path"].as<std::string>();
    std::string feature_file = config["feature_file_path"].as<std::string>(); // The features of the seam in the image coordiantes 
    std::string image_timestamp = config["image_timestamp_path"].as<std::string>();


    double dt = config["dt_test"].as<double>();
    std::cout << "dt_test :" << dt << std::endl;
    float p_a= config["PA"].as<float>();
    float p_b= config["PB"].as<float>();
    float p_c= config["PC"].as<float>();

    // float e1 = config["extrinsic1"].as<float>();
    // float e2 = config["extrinsic2"].as<float>();
    // float e3 = config["extrinsic3"].as<float>();
    // float e4 = config["extrinsic4"].as<float>();
    // float e5 = config["extrinsic5"].as<float>();
    // float e6 = config["extrinsic6"].as<float>();

    float kx = 3597.8882;
    float ky = 3599.5422;
    float u0 = 1211.2184;
    float v0 = 1007.1196;
    float k1 = -0.2307;
    float k2 = 0.1734;
    float k3 = -0.2950;
    float p1 = 0;
    float p2 = 0;

    

    int weld_type = config["weld_type"].as<int>();
    
    std::ofstream laser_point_file;
    laser_point_file.open("data.txt", std::ios_base::trunc);
    std::ofstream opt;
    opt.open("opt.txt", std::ios_base::trunc);

    std::ofstream output_egm;
    output_egm.open("output_egm.txt", std::ios_base::trunc);
    std::ofstream output_img;
    output_img.open("output_img.txt", std::ios_base::trunc);

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
    double sec, nsec, sec0, nsec0;
    double timestamp_f_0 = 0;

    bool first_line = true;

    while(std::getline(file, str))
    {
        iss.clear();
        iss.str(str);
        iss >> sec >> nsec >> x >> y >> z >> ex >> ey >> ez >> qw >> qx >> qy >> qz >> timestamp_f; //timestamp_f: us 
        // std::cout << "timestamp_f: " << timestamp_f << std::endl;
        egm_timestamps.push_back(sec + nsec / 1e9);
        egm_f_timestamps.push_back(timestamp_f/ 1e6);
        if(first_line)
        {
            first_line = false;
            sec0 = sec; nsec0 = nsec; timestamp_f_0 = timestamp_f;
        }

        double new_time_sec = sec0 + nsec0 / 1e9 + (timestamp_f - timestamp_f_0)/ 1e6;

        output_egm <<  std::fixed << std::setprecision(3) << static_cast<int>(new_time_sec) << " "
            << (new_time_sec - static_cast<int>(new_time_sec))* 1e9 << " " << x << " " 
            << y << " " << z << " " << ex << " " << ey << " " << ez << " " << qw << " "
            << qx << " " << qy << " " << qz << " " << timestamp_f << std::endl;

        gtsam::Quaternion q(qw, qx, qy, qz);
        gtsam::Pose3 robot(gtsam::Rot3(q.matrix()), Eigen::Vector3d(x, y, z));
        robots.push_back(robot);
    }
    first_line = true;

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

        if(first_line)
        {
            first_line = false;
            sec0 = sec; nsec0 = nsec; timestamp_f_0 = timestamp_image_f;
        }

        double new_time_sec = sec0 + nsec0 / 1e9 + (timestamp_image_f - timestamp_f_0)/ 1e9 - dt;

        output_img << std::fixed << std::setprecision(0) << static_cast<int>(new_time_sec) << " "
            << (new_time_sec - static_cast<int>(new_time_sec))* 1e9 << " " << 
            image_index << " " << timestamp_image_f << std::endl;
    }


    return 0;
}
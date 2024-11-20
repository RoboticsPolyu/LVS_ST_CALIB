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

#define P_S_X       2100.00
#define P_S_Y       0.00
#define P_S_Z       1000.00

#define LVS_DEPTH   100.0
#define WELD_LENGTH 400.0
#define VEL_X      (40.0/1000) // mm/ms

#define P_L_S_X    (P_S_X)
#define P_L_S_Y    (P_S_Y + 5)
#define P_L_S_Z    (P_S_Z - LVS_DEPTH)

#define P_L_E_X    (P_S_X + WELD_LENGTH)
#define P_L_E_Y    (P_S_Y + 20)
#define P_L_E_Z    (P_S_Z - LVS_DEPTH  + 10)

#define P_L_S_X_2    (P_S_X)
#define P_L_S_Y_2    (P_S_Y - 5)
#define P_L_S_Z_2    (P_S_Z - LVS_DEPTH)

#define P_L_E_X_2    (P_S_X + WELD_LENGTH)
#define P_L_E_Y_2    (P_S_Y - 20)
#define P_L_E_Z_2    (P_S_Z - LVS_DEPTH + 10)

#define T_MAX      (WELD_LENGTH/ VEL_X) // ms

#define T_CYCLE_CAM 27.6 // ms
#define T_CYCLE_EGM 4.0 //ms

#define T_D         25.6 //ms
#define EXT_X       10.0
#define EXT_Y       20.0
#define EXT_Z       30.0

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
    
    std::string egm_file = "egm.txt";
    std::string feature_file = "cam.txt"; // The features of the seam in the image coordiantes 

    double dt = config["DT"].as<double>();
    std::cout << "DT :" << dt << std::endl;
    float p_a= config["PA"].as<float>();
    float p_b= config["PB"].as<float>();
    float p_c= config["PC"].as<float>();
    bool enable_ext_opt = config["EXT_OPT"].as<bool>();
    // bool enable_pl      = config["PL_OPT"].as<bool>();
    bool enable_pl_opt  = config["PL_OPT"].as<bool>();
    bool enable_td_opt = config["TD_OPT"].as<bool>();

    float ERR_THR = config["ERROR_THR"].as<float>();
    uint32_t ITER_NUM = config["ITER_NUM"].as<uint32_t>();

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
    double timestamp, x, y, z, e1, e2, e3, e4, e5, e6, timestamp_f; // timestamp_f: ms
    std::vector<gtsam::Pose3> sensors, robots;

    std::vector<gtsam::Vector3> eWs; // robot angular speed and linear speed
    std::vector<gtsam::Vector3> bVs;

    std::vector<double> egm_timestamps;
    std::vector<double> egm_f_timestamps;
    double ms;
    std::vector<gtsam::Vector4> base_P_corners;

    while(std::getline(file, str))
    {
        iss.clear();
        iss.str(str);
        iss >> ms >> e1 >> e2 >> e3 >> e4 >> e5 >> e6;
        egm_timestamps.push_back(ms / 1e3);
        gtsam::Vector6 egm_;
        egm_ << e1, e2, e3, e4, e5, e6;

        gtsam::Pose3 robot = gtsam::Pose3::Expmap(egm_);
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
    image_timestamp_file.open(feature_file);

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
    double cx, cy, cz, cx_2, cy_2, cz_2;

    std::vector<gtsam::Vector4> cam_P_corner;
    std::vector<gtsam::Vector4> cam_P_corner_2;
    gtsam::Vector4 cam_P_corner_t;
    gtsam::Vector4 cam_P_corner_t_2;

    uint idx = 0;
    while(std::getline(image_timestamp_file, str))
    {
        // std::cout << str << std::endl;
        iss.clear();
        iss.str(str);
        iss >> ms >> cx >> cy >> cz >> cx_2 >> cy_2 >> cz_2;
        
        cam_P_corner_t[0] = cx;
        cam_P_corner_t[1] = cy;
        cam_P_corner_t[2] = cz;
        cam_P_corner_t[3] = 1;
        cam_P_corner.push_back(cam_P_corner_t);

        cam_P_corner_t_2[0] = cx_2;
        cam_P_corner_t_2[1] = cy_2;
        cam_P_corner_t_2[2] = cz_2;
        cam_P_corner_t_2[3] = 1;
        cam_P_corner_2.push_back(cam_P_corner_t_2);
        idx++;

        image_timestamps.push_back(ms / 1e3);
    }
    
    std::cout << "image_timestamps:" << image_timestamps.size() << std::endl;

    gtsam::Vector6 extrinsic;
    extrinsic = gtsam::Pose3::Logmap( gtsam::Pose3(gtsam::Rot3::Expmap(gtsam::Vector3(0.0,0.0,0) ), gtsam::Vector3(EXT_X, EXT_Y, EXT_Z) ) );
    
    gtsam::Pose3 eTc;
    gtsam::Pose3 eTc_ = gtsam::Pose3::Expmap(extrinsic);
    gtsam::Point3 t;

    t = eTc_.translation();
    eTc = gtsam::Pose3(eTc_.rotation(), t);

    if(enable_ext_opt)
    {
        t = eTc_.translation() + gtsam::Point3(1, 2, 3);
        eTc =  gtsam::Pose3( gtsam::Rot3::Expmap(gtsam::Vector3(0.01, 0.02, 0.02))* eTc_.rotation(), t);
    }
    else
    {
        t = eTc_.translation();
        eTc = gtsam::Pose3(eTc_.rotation(), t);
    }

    std::cout << "Init Rotation Matrix: " << eTc.rotation().matrix();
    


    float pl_pl_dx = 0, pl_pl_dy = 0.5, pl_pl_dz = 0;
    gtsam::Vector3 pl(P_L_S_X + pl_pl_dx, P_L_S_Y + pl_pl_dy, P_L_S_Z);
    float pl_end_dx = 0, pl_end_dy = -0.5, pl_end_dz = 0;
    gtsam::Vector3 pl_end(P_L_E_X + pl_end_dx, P_L_E_Y + pl_end_dy, P_L_E_Z);

    gtsam::Vector3 pl_2(P_L_S_X_2 + pl_pl_dx, P_L_S_Y_2 + pl_pl_dy, P_L_S_Z_2);
    gtsam::Vector3 pl_end_2(P_L_E_X_2 + pl_end_dx, P_L_E_Y_2 + pl_end_dy, P_L_E_Z_2);

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
    gtsam::Vector4 new_Lw;
    new_Lw.head(3) = gtsam::Rot3::Logmap(gtsam::Rot3(rot3));
    new_Lw(3)  = m;
    std::cout << "Init LW1: " << new_Lw << std::endl;

    gtsam::Vector4 new_Lw_2;
    v = pl_end_2 - pl_2;
    v = v/ v.norm();

    n = pl_2.cross(v); // - gtsam::Vector3(10, 100, 10);

    std::cout << "init n:\n " << n << std::endl;
    std::cout << "init v:\n " << v << std::endl;

    rot3.col(0) = v;
    rot3.col(1) = n/ n.norm();
    rot3.col(2) = v.cross(n) / (n.norm());

    m = n.norm();

    calibration::PluckerLine pll2(gtsam::Rot3(rot3), m);
    new_Lw_2.head(3) = gtsam::Rot3::Logmap(gtsam::Rot3(rot3));
    new_Lw_2(3)  = m;
    std::cout << "Init LW2: " << new_Lw_2 << std::endl;

    uint32_t jsize = 8000;
    /***************************************************LM algrithm*******************************************************/
    gtsam::Matrix3  D1;
    gtsam::Matrix34 D2;
    Eigen::VectorXd J_td(3* jsize);
    Eigen::VectorXd Last_J_td(3* jsize);

    Eigen::VectorXd J_error(3* jsize);
    Eigen::VectorXd Last_J_error(3* jsize);

    Eigen::MatrixXd J_lw(3* jsize, 4);
    Eigen::MatrixXd Last_J_lw(3* jsize, 4);

    Eigen::MatrixXd J_Lr(3* jsize, 3);
    Eigen::VectorXd J_Lm(3* jsize);
    Eigen::MatrixXd Last_J_Lr(3* jsize, 3);
    Eigen::VectorXd Last_J_Lm(3* jsize);

    Eigen::MatrixXd J_Lr_2(3* jsize, 3);
    Eigen::VectorXd J_Lm_2(3* jsize);
    Eigen::MatrixXd Last_J_Lr_2(3* jsize, 3);
    Eigen::VectorXd Last_J_Lm_2(3* jsize);

    Eigen::MatrixXd J_R(3* jsize, 3);
    Eigen::MatrixXd J_P(3* jsize, 3);
    Eigen::MatrixXd Last_J_R(3* jsize, 3);
    Eigen::MatrixXd Last_J_P(3* jsize, 3);

    gtsam::Vector4 Last_delta_Lw;
    float delta_td = dt;

    gtsam::Vector3 dlr;
    double         dlm = 0;
    gtsam::Vector3 dR;
    gtsam::Vector3 dP;
    gtsam::Vector3 dlr_2;
    double         dlm_2 = 0;

    double miu_td = 0, v_lm_td = 1;
    double tou = 1;
    double ro_td = 0;

    // LM for HandEye
    double miu_p = 0, v_lm_p = 1;
    double ro_p = 0;

    // LM for PL
    double miu_pl_r = 0, v_lm_pl_r = 1;
    double ro_pl_r = 0;

    double miu_pl_r_2 = 0, v_lm_pl_r_2 = 1;
    double ro_pl_r_2 = 0;

    double miu_plr_max = 0, miu_plm_max = 0, miu_plr_max_2 = 0, miu_plm_max_2 = 0,miu_td_max = 0, miu_r_max = 0, miu_p_max = 0;

    double miu_pl_m = 0, v_lm_pl_m = 1;
    double ro_pl_m = 0;

    double miu_pl_m_2 = 0, v_lm_pl_m_2 = 1;
    double ro_pl_m_2 = 0;

    double miu_r = 0, v_lm_r = 1;
    double ro_r = 0;

    gtsam::Vector3 rotv = gtsam::Rot3::Logmap(eTc.rotation());

    opt << 0 << " " << dt << " " << new_Lw(0) << " " << new_Lw(1) << " " << new_Lw(2) << " " << new_Lw(3) << " " << 
            new_Lw_2(0) << " " << new_Lw_2(1) << " " << new_Lw_2(2) << " " << new_Lw_2(3) << " " << 0 << " " << 
            rotv[0] << " " << rotv[1] << " " << rotv[2] << " " 
            << eTc.translation().x() << " " << eTc.translation().y() << " " << eTc.translation().z() << std::endl;


    for(uint iter_num = 0; iter_num < ITER_NUM; iter_num++)
    {
        std::cout << "##########################" << iter_num << "###########################\n";
        J_error.setZero();
        J_td.setZero();
        J_lw.setZero();
        J_Lr.setZero();
        J_Lm.setZero();
        J_P.setZero();
        J_R.setZero();
        J_Lr_2.setZero();
        J_Lm_2.setZero();

        float distance_sum = 0;
        uint32_t meas_num = 0;

        for(uint i = 0; i < image_timestamps.size(); i++)
        {
            image_timestamps[i] = image_timestamps[i] - delta_td;
        }

        pll.Update(new_Lw);
        pll2.Update(new_Lw_2);

        for(uint idx = 2; idx < cam_P_corner.size()-1; idx = idx +10)
        {
            uint t_l_index = 0;

            gtsam::Vector4 base_P_corner;
            gtsam::Pose3 robot_pose_searched;


            robot_pose_searched = Interpolation(egm_timestamps, robots, image_timestamps[idx], t_l_index);
            // std::cout << "######################################### \n";
            // std::cout << t_l_index << " " << robot_pose_searched << std::endl;
            base_P_corner = (robot_pose_searched* eTc).matrix()* cam_P_corner[idx];

            gtsam::Vector3 pm(base_P_corner[0], base_P_corner[1], base_P_corner[2]);

            if(t_l_index == 0)
                continue;
            
            if(egm_timestamps[t_l_index+1] - egm_timestamps[t_l_index] > 0.005)
                continue;

            // std::cout << "******************************** IDX ********************************************" << std::endl;
            gtsam::Vector3 error = pll.Distance(pm, D1, D2);
            // std::cout << "************** D *************** \n";
            // std::cout << "t_l_index: " << t_l_index << std::endl;
            // std::cout << "D_td: \n" << D1* bVs[t_l_index] << std::endl;
            
            //std::cout << "idx: " << idx <<  " ,pm: \n" << pm << std::endl;
             //std::cout << "error: " << error << std::endl;

            if(error.norm() < ERR_THR)
            {
                // if(error >= 5)
                //     error = 5;
                /*HandEye Jacobian*/
                J_R.block(3*meas_num, 0, 3, 3) = - D1* robot_pose_searched.rotation().matrix()* 
                                        gtsam::skewSymmetric(eTc.rotation().matrix()* 
                                            cam_P_corner[idx].head(3));

                J_P.block(3*meas_num, 0, 3, 3) =   D1* robot_pose_searched.rotation().matrix();

                /*camera offset Jacobian*/
                J_td.block(3*meas_num, 0, 3, 1) = D1*(-gtsam::skewSymmetric(robot_pose_searched.rotation().matrix()
                                        * eTc.rotation().matrix()* cam_P_corner[idx].head(3))
                                        * robots[t_l_index].rotation().matrix()
                                        * eWs[t_l_index] 
                                        - bVs[t_l_index]);
                J_error.block(3*meas_num, 0, 3, 1) = error;

                /*Line Jacobian*/
                // J_lw.row(meas_num) = D2;
                J_Lr.block(3*meas_num, 0, 3, 3)     = D2.block(0, 0, 3, 3);
                J_Lm.block(3*meas_num, 0, 3, 1)     = D2.block(0, 3, 3, 1);
                //J_Lr_2.block(3*meas_num, 0, 3, 3)  
                //J_Lm_2(meas_num)       = 0;

                meas_num++;
                
                distance_sum += error.dot(error);
            }
            
            gtsam::Vector4  base_P_corner2 = (robot_pose_searched* eTc).matrix()* cam_P_corner_2[idx];

            gtsam::Vector3 pm2(base_P_corner2[0], base_P_corner2[1], base_P_corner2[2]);

            error = pll2.Distance(pm2, D1, D2);

            if(error.norm() < ERR_THR)
            {
                J_R.block(3*meas_num, 0, 3, 3) = - D1* robot_pose_searched.rotation().matrix()* 
                                        gtsam::skewSymmetric(eTc.rotation().matrix()* 
                                            cam_P_corner_2[idx].head(3));

                J_P.block(3*meas_num, 0, 3, 3) =   D1* robot_pose_searched.rotation().matrix();

                /*camera offset Jacobian*/
                J_td.block(3*meas_num, 0, 3, 1) = D1*(-gtsam::skewSymmetric(robot_pose_searched.rotation().matrix()
                                        * eTc.rotation().matrix()* cam_P_corner_2[idx].head(3))
                                        * robots[t_l_index].rotation().matrix()
                                        * eWs[t_l_index] 
                                        - bVs[t_l_index]);
                J_error.block(3*meas_num, 0, 3, 1) = error;

                /*Line Jacobian*/
                // J_lw.row(meas_num) = D2;
                J_Lr_2.block(3*meas_num, 0, 3, 3)     = D2.block(0, 0, 3, 3);
                J_Lm_2.block(3*meas_num, 0, 3, 1)     = D2.block(0, 3, 3, 1);
                //J_Lr_2.block(3*meas_num, 0, 3, 3)  
                //J_Lm_2(meas_num)       = 0;

                meas_num++;
                
                distance_sum += error.dot(error);
            }

            if(iter_num == ITER_NUM -1)
            {
            // std::cout << "---> BASE_CORNER: [ " << base_P_corner[0] << " " << base_P_corner[1] << " " << base_P_corner[2] << " " << base_P_corner[3] << " ] <---" << std::endl;
                laser_point_file << base_P_corner[0] << " " << base_P_corner[1] << " " << base_P_corner[2] << " " << base_P_corner[3] << " " << std::sqrt(distance_sum / meas_num)  << std::endl;
                laser_point_file << base_P_corner2[0] << " " << base_P_corner2[1] << " " << base_P_corner2[2] << " " << base_P_corner2[3] << " " << std::sqrt(distance_sum / meas_num) << std::endl;
            }
            // std::cout << robots[uv_lens+ idx%4].translation();
            // robots[uv_lens+ idx%4].translation() = robots[uv_lens+ idx%4].translation();

            // gtsam::Point3 error_ee_ = base_P_corner.head(3) - robots[uv_lens+ idx%4].translation();
            // gtsam::Point3 ee_t = robots[uv_lens+ idx%4].rotation().inverse()* error_ee_;
            // std::cout << "---- error_ee_： " << error_ee_[0] << " " << error_ee_[1] << " " << error_ee_[2] << std::endl;
            // std::cout << "---- ee_: " << ee_t[0] << " " << ee_t[1] << " " << ee_t[2] << std::endl;

            base_P_corners.push_back(base_P_corner);
            // base_P_corners.push_back(base_P_corner2);
        }    
        
        Eigen::MatrixXd JJ(meas_num, 15);
        JJ.block(0, 0, meas_num, 1) = J_td;
        JJ.block(0, 1, meas_num, 3) = J_Lr;
        JJ.block(0, 4, meas_num, 1) = J_Lm;
        JJ.block(0, 5, meas_num, 3) = J_R;
        JJ.block(0, 8, meas_num, 3) = J_P;
        JJ.block(0, 11, meas_num, 3) = J_Lr_2;
        JJ.block(0, 14, meas_num, 1) = J_Lm_2;

        if(iter_num == 0)
        {

        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(JJ, Eigen::ComputeFullU | Eigen::ComputeFullV);
        std::cout << "------------------------------SVD: -------------------------\n" <<  svd.singularValues() << std::endl;

        std::cout << "MEAS_SIZE: " << meas_num << std::endl;
        std::cout << "RMS: " << std::sqrt(distance_sum / meas_num) << std::endl;

        float dtt; 

        if(iter_num ==0)
        {
            miu_td = tou* J_td.dot(J_td);
            Eigen::Matrix3d Hess_lr;
            Hess_lr.setZero();
            Hess_lr = J_Lr.transpose()* J_Lr;
            double max_element = 0;
            for(int i = 0; i < 3; i++)
            {
                std::cout << "Hess_lw(i,i): " << Hess_lr(i,i) << std::endl;
                if(Hess_lr(i,i) > max_element)
                {
                    max_element = Hess_lr(i,i);
                }
            }
            miu_pl_r = max_element;
            
            miu_pl_m = J_Lm.dot(J_Lm);
            miu_plr_max = miu_pl_r;
            miu_plm_max = miu_pl_m;
            miu_td_max  = miu_td;
            
            Hess_lr.setZero();
            Hess_lr = J_Lr_2.transpose()* J_Lr_2;
            max_element = 0;
            for(int i = 0; i < 3; i++)
            {
                std::cout << "Hess_lw(i,i): " << Hess_lr(i,i) << std::endl;
                if(Hess_lr(i,i) > max_element)
                {
                    max_element = Hess_lr(i,i);
                }
            }
            miu_pl_r_2 = max_element;
            
            miu_pl_m_2 = J_Lm_2.dot(J_Lm_2);
            miu_plr_max_2 = miu_pl_r_2;
            miu_plm_max_2 = miu_pl_m_2;

            Eigen::Matrix3d Hess3;
            Hess3.setZero();

            Hess3 = J_R.transpose()* J_R;
            max_element = 0;
            for(int i = 0; i < 3; i++)
            {
                if(Hess3(i,i) > max_element)
                {
                    max_element = Hess3(i,i);
                }
            }
            miu_r = max_element;
            miu_r_max = 10* miu_r;

            Hess3.setZero();
            Hess3 = J_P.transpose()* J_P;
            max_element = 0;
            for(int i = 0; i < 3; i++)
            {
                if(Hess3(i,i) > max_element)
                {
                    max_element = Hess3(i,i);
                }
            }
            miu_p = max_element;
            miu_p_max = 10* miu_p;
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
                if(miu_td > miu_td_max)
                    miu_td = miu_td_max;
                v_lm_td = 2* v_lm_td;
            }
            
            ro_pl_r = (Last_J_error.dot(Last_J_error) - J_error.dot(J_error) )
            / (dlr.transpose()*(miu_pl_r* dlr - Last_J_Lr.transpose()* Last_J_error) );
            std::cout << "ro_pl_r: \n" << ro_pl_r << std::endl;
            
            if(ro_pl_r > 0)
            {
                miu_pl_r = miu_pl_r* std::max(1.0/3, 1-(2*ro_pl_r - 1)* (2*ro_pl_r - 1)* (2*ro_pl_r - 1));
                v_lm_pl_r = 2;
            }
            else
            {
                miu_pl_r = miu_pl_r* v_lm_pl_r; 
                if(miu_pl_r > miu_plr_max)
                    miu_pl_r = miu_plr_max;
                v_lm_pl_r = 2* v_lm_pl_r;
            }

            ro_pl_m = (Last_J_error.dot(Last_J_error) - J_error.dot(J_error) )
            / (dlm* (miu_pl_m* dlm - Last_J_Lm.dot(Last_J_error)) );
            std::cout << "ro_pl_m: \n" << ro_pl_m << std::endl;
            
            if(ro_pl_m > 0)
            {
                miu_pl_m = miu_pl_m* std::max(1.0/3, 1-(2*ro_pl_m - 1)* (2*ro_pl_m - 1)* (2*ro_pl_m - 1));
                
                v_lm_pl_m = 2;
            }
            else
            {
                miu_pl_m = miu_pl_m* v_lm_pl_m; 
                if(miu_pl_m > miu_plm_max)
                    miu_pl_m = miu_plm_max;
                v_lm_pl_m = 2* v_lm_pl_m;
            }

            ro_pl_r_2 = (Last_J_error.dot(Last_J_error) - J_error.dot(J_error) )
            / (dlr_2.transpose()*(miu_pl_r_2* dlr_2 - Last_J_Lr_2.transpose()* Last_J_error) );
            std::cout << "ro_pl_r_2: \n" << ro_pl_r_2 << std::endl;
            
            if(ro_pl_r_2 > 0)
            {
                miu_pl_r_2 = miu_pl_r_2* std::max(1.0/3, 1-(2*ro_pl_r_2- 1)* (2*ro_pl_r_2 - 1)* (2*ro_pl_r_2 - 1));
                v_lm_pl_r_2 = 2;
            }
            else
            {
                miu_pl_r_2 = miu_pl_r_2* v_lm_pl_r_2; 
                if(miu_pl_r_2> miu_plr_max_2)
                    miu_pl_r_2 = miu_plr_max_2;
                v_lm_pl_r_2 = 2* v_lm_pl_r_2;
            }

            ro_pl_m_2 = (Last_J_error.dot(Last_J_error) - J_error.dot(J_error) )
            / (dlm_2* (miu_pl_m_2* dlm_2 - Last_J_Lm_2.dot(Last_J_error)) );
            std::cout << "ro_pl_m_2: \n" << ro_pl_m_2 << std::endl;
            
            if(ro_pl_m_2 > 0)
            {
                miu_pl_m_2 = miu_pl_m_2* std::max(1.0/3, 1-(2*ro_pl_m_2 - 1)* (2*ro_pl_m_2 - 1)* (2*ro_pl_m_2 - 1));
                
                v_lm_pl_m_2 = 2;
            }
            else
            {
                miu_pl_m_2 = miu_pl_m_2* v_lm_pl_m_2; 
                if(miu_pl_m_2 > miu_plm_max_2)
                    miu_pl_m_2 = miu_plm_max_2;
                v_lm_pl_m_2 = 2* v_lm_pl_m_2;
            }

            ro_r = (Last_J_error.dot(Last_J_error) - J_error.dot(J_error) )
            / (dR.transpose()*(miu_r* dR - Last_J_R.transpose()* Last_J_error) );
            std::cout << "ro_r: \n" << ro_r << std::endl;
            
            if(ro_r > 0)
            {
                miu_r = miu_r* std::max(1.0/3, 1-(2*ro_r - 1)* (2*ro_r - 1)* (2*ro_r - 1));
                v_lm_r = 2;
            }
            else
            {
                miu_r = miu_r* v_lm_r; 
                if(miu_r > miu_r_max)
                    miu_r = miu_r_max;

                v_lm_r = 2* v_lm_r;
            }

            ro_p = (Last_J_error.dot(Last_J_error) - J_error.dot(J_error) )
            / (dP.transpose()*(miu_p* dP - Last_J_P.transpose()* Last_J_error) );
            std::cout << "ro_p: \n" << ro_p << std::endl;
            
            if(ro_p > 0)
            {
                miu_p = miu_p* std::max(1.0/3, 1-(2*ro_p - 1)* (2*ro_p - 1)* (2*ro_p - 1));
                v_lm_p = 2;
            }
            else
            {
                miu_p = miu_p* v_lm_p; 
                if(miu_p > miu_p_max)
                    miu_p = miu_p_max;
                v_lm_p = 2* v_lm_p;
            }
        }
        std::cout << "miu_td: " << miu_td << " , v_lm_td: " << v_lm_td << std::endl;
        std::cout << "miu_pl_m: " << miu_pl_m << " , v_lm_pl_m: " << v_lm_pl_m << std::endl;
        std::cout << "miu_pl_r: " << miu_pl_r << " , v_lm_pl_r " << v_lm_pl_r << std::endl;
        std::cout << "miu_p: " << miu_p << " , v_lm_p: " << v_lm_p << std::endl;
        std::cout << "miu_r: " << miu_r << " , v_lm_r: " << v_lm_r << std::endl;

        if(enable_td_opt)
        {
            dtt = - (J_td.transpose()* J_td + miu_td* gtsam::I_1x1).inverse()* J_td.transpose()* J_error;
            dt = dt + dtt;
            delta_td = dtt;
            std::cout << "Dtt: " << dtt << " , DT: " << dt << std::endl;
        }
        else
        {
            delta_td = 0;
        }
                
        if(enable_pl_opt)
        {
            dlr = -(J_Lr.transpose()* J_Lr + miu_pl_r* gtsam::I_3x3).inverse()* J_Lr.transpose()* J_error;
            std::cout << "Dlr: \n" << dlr << std::endl;

            gtsam::Rot3 new_R = gtsam::Rot3::Expmap(dlr)* gtsam::Rot3::Expmap(new_Lw.head(3));
            new_Lw.head(3) = gtsam::Rot3::Logmap(new_R);
            
            dlm = -(J_Lm.transpose()* J_Lm + miu_pl_m* gtsam::I_1x1).inverse()* J_Lm.transpose()* J_error;
            std::cout << "Dlm: \n" << dlm << std::endl;
            new_Lw(3) = new_Lw(3) + dlm;
            std::cout << "new_Lw : \n" << new_Lw  << std::endl;

            dlr_2 = -(J_Lr_2 .transpose()* J_Lr_2  + miu_pl_r_2 * gtsam::I_3x3).inverse()* J_Lr_2.transpose()* J_error;
            std::cout << "Dlr_2 : \n" << dlr_2  << std::endl;

            new_R = gtsam::Rot3::Expmap(dlr_2)* gtsam::Rot3::Expmap(new_Lw_2.head(3));
            new_Lw_2.head(3) = gtsam::Rot3::Logmap(new_R);
            
            dlm_2 = -(J_Lm_2.transpose()* J_Lm_2 + miu_pl_m_2* gtsam::I_1x1).inverse()* J_Lm_2.transpose()* J_error;
            std::cout << "Dlm_2 : \n" << dlm_2 << std::endl;
            new_Lw_2(3) = new_Lw_2(3) + dlm_2 ;
            std::cout << "new_Lw_2 : \n" << new_Lw_2  << std::endl;

            // dlr = -(J_Lr.transpose()* J_Lr + 0* gtsam::I_3x3).inverse()* J_Lr.transpose()* J_error;
            // std::cout << "Dlr: \n" << dlr << std::endl;

            // gtsam::Rot3 new_R = gtsam::Rot3::Expmap(0.2* dlr)* gtsam::Rot3::Expmap(new_Lw.head(3));
            // new_Lw.head(3) = gtsam::Rot3::Logmap(new_R);
            
            // dlm = -(J_Lm.transpose()* J_Lm + 0* gtsam::I_1x1).inverse()* J_Lm.transpose()* J_error;
            // std::cout << "Dlm: \n" << dlm << std::endl;
            // new_Lw(3) = new_Lw(3) + 0.2* dlm;
            // std::cout << "new_Lw : \n" << new_Lw  << std::endl;

            // dlr_2 = -(J_Lr_2 .transpose()* J_Lr_2  + 0 * gtsam::I_3x3).inverse()* J_Lr_2.transpose()* J_error;
            // std::cout << "Dlr_2 : \n" << dlr_2  << std::endl;

            // new_R = gtsam::Rot3::Expmap(0.2* dlr_2)* gtsam::Rot3::Expmap(new_Lw_2 .head(3));
            // new_Lw_2.head(3) = gtsam::Rot3::Logmap(new_R);
            
            // dlm_2 = -(J_Lm_2.transpose()* J_Lm_2 + 0* gtsam::I_1x1).inverse()* J_Lm_2.transpose()* J_error;
            // std::cout << "Dlm_2 : \n" << dlm_2 << std::endl;
            // new_Lw_2(3) = new_Lw_2(3) + 0.2* dlm_2 ;
            // std::cout << "new_Lw_2 : \n" << new_Lw_2  << std::endl;
        }
            
        rotv = gtsam::Rot3::Logmap(eTc.rotation());


        Last_J_error  = J_error;
        Last_J_td     = J_td;
        Last_J_Lm     = J_Lm;
        Last_J_Lr     = J_Lr;
        Last_J_P      = J_P;
        Last_J_R      = J_R;
        Last_J_Lr_2   = J_Lr_2;
        Last_J_Lm_2   = J_Lm_2;

        
        if(enable_ext_opt)
        {
            // dR = - (J_R.transpose()* J_R + miu_r* gtsam::I_3x3).inverse()* J_R.transpose()* J_error;
            dP = - (J_P.transpose()* J_P + 0* gtsam::I_3x3).inverse()* J_P.transpose()* J_error;

            dR = - (J_R.transpose()* J_R + 0* gtsam::I_3x3).inverse()* J_R.transpose()* J_error;

            dR << dR(0), dR(1), dR(2);
            dP << dP(0), dP(1), dP(2);

            // dP = - (J_P.transpose()* J_P + 0* gtsam::I_3x3).inverse()* J_P.transpose()* J_error;
            gtsam::Point3 new_eP = eTc.translation()  + kp* dP;
            gtsam::Rot3 new_eR(gtsam::Rot3::Expmap(kr* dR).matrix()* eTc.rotation().matrix());
            eTc = gtsam::Pose3(new_eR, new_eP);
            std::cout << "dR: \n" << dR << std::endl;
            std::cout << "dP: \n" << dP << std::endl;
        }

        // std::cout << "MEAS_SIZE: " << meas_num << std::endl;
        // std::cout << "RMS: " << std::sqrt(distance_sum / meas_num) << std::endl;

        // for(uint idx = 0; idx < base_P_corners.size()-1; idx++)
        // {
        //     double dis = (base_P_corners[idx+1] - base_P_corners[idx]).norm();
        //     // std::cout << "dis: " << dis << std::endl;
        // }

        // float kdt = 0.1; //0.1
        // double k2 = 0.1; //0.1;
        // double km = 0.1; //0.05;

        // float dtt = - (J_td.transpose()* J_td).inverse()* J_td.transpose()* J_error;
        // dt = dt + kt* dtt;
        // delta_td = kt* dtt;
        // // delta_td = 0;

        // std::cout << "Dtt: \n" << dtt << " , DT: \n" << dt << std::endl;
        // // std::cout << J_lw << std::endl;
        // gtsam::Vector3 dR = - (J_R.transpose()* J_R).inverse()* J_R.transpose()* J_error;
        // gtsam::Vector3 dP = - (J_P.transpose()* J_P).inverse()* J_P.transpose()* J_error;
        
        // // if(std::abs(dtt) < 0.0005)
        // //     break;
        
        // float miu = 0;
        // float ro;

        // // if(iter_num > 0)
        // // {
        // //     ro = 0.5*(Last_J_error.transpose()* Last_J_error - J_error.transpose()* J_error)/
        // //     0.5*(Last_J_error.transpose()* Last_J_error - 
        // //         (Last_J_error + Last_J_lw* Last_delta_Lw).transpose()* (Last_J_error + Last_J_lw* Last_delta_Lw) );
        // //     std::cout << "ro: \n" << ro << std::endl;
        // // }
        // gtsam::Vector4 dlw;
        // if(enable_pl)
        // {
        //     dlw = -(J_lw.transpose()* J_lw).inverse()* J_lw.transpose()* J_error;
        //     std::cout << "Dlw: \n" << dlw << std::endl;

        //     gtsam::Rot3 new_R = gtsam::Rot3::Expmap(k2* dlw.head(3))* gtsam::Rot3::Expmap(new_Lw.head(3));

        //     new_Lw(3) = new_Lw(3) + km* dlw(3);
        //     new_Lw.head(3) = gtsam::Rot3::Logmap(new_R);
        // }
        
        
        // std::cout << "new_Lw: \n" << new_Lw << std::endl;
        // std::cout << "dR: \n" << dR << std::endl;
        // std::cout << "dP: \n" << dP << std::endl;

        // gtsam::Vector3 rotv = gtsam::Rot3::Logmap(eTc.rotation());

        // opt << dtt << " " << dt << " " << new_Lw(0) << " " << new_Lw(1) << " " << new_Lw(2) << " " << new_Lw(3) << " " << std::sqrt(distance_sum / meas_num) << " " << 
        // rotv[0] << " " << rotv[1] << " " << rotv[2] << " " << eTc.translation().x() << " " << eTc.translation().y() << " " << eTc.translation().z() << std::endl;

        // Last_J_error  = J_error;
        // Last_J_lw     = J_lw;
        // Last_J_td     = J_td;
        // Last_delta_Lw = dlw;

        // float e_k = 0.5;
        // float e_p = 0.5;
        // gtsam::Matrix3 I3 = gtsam::I_3x3;
        // I3(1,1) = 0;
        
        // if(enable_ext_opt)
        // {
        //     gtsam::Point3 new_eP = eTc.translation()  + e_p* gtsam::Point3(0, dP(1), dP(2));
        //     gtsam::Rot3 new_eR(gtsam::Rot3::Expmap(e_k* gtsam::Vector3(dR) ).matrix()* eTc.rotation().matrix());
        //     eTc = gtsam::Pose3(new_eR, new_eP);
        // }
        opt << dtt << " " << dt << " " << new_Lw(0) << " " << new_Lw(1) << " " << new_Lw(2) << " " << new_Lw(3) << " " << 
            new_Lw_2(0) << " " << new_Lw_2(1) << " " << new_Lw_2(2) << " " << new_Lw_2(3) << " " << std::sqrt(distance_sum / meas_num) << " " << 
            rotv[0] << " " << rotv[1] << " " << rotv[2] << " " 
            << eTc.translation().x() << " " << eTc.translation().y() << " " << eTc.translation().z() << std::endl;
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
 
#include <random>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "gtsam_wrapper.h"
#include "hand_eye_calibration.h"
#include "laser_camera_cal.h"
#include "td_factor.h"


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
#define P_L_E_Z    (P_S_Z - LVS_DEPTH  + 10 )

#define P_L_S_X_2    (P_S_X)
#define P_L_S_Y_2    (P_S_Y - 5)
#define P_L_S_Z_2    (P_S_Z - LVS_DEPTH)

#define P_L_E_X_2    (P_S_X + WELD_LENGTH)
#define P_L_E_Y_2    (P_S_Y - 20)
#define P_L_E_Z_2    (P_S_Z - LVS_DEPTH  + 10 )

#define T_MAX      (WELD_LENGTH/ VEL_X) // ms

#define T_CYCLE_CAM 27.6 // ms
#define T_CYCLE_EGM 4.0 //ms

#define T_D         12.0 //ms
#define EXT_X       10.0
#define EXT_Y       20.0
#define EXT_Z       30.0
// ext rot = I3

gtsam::Rot3 generate_rot_end(int index);
gtsam::Pose3 generate_r_end(float t, gtsam::Rot3 &rot);
gtsam::Vector3 generate_lvs(gtsam::Pose3& end, gtsam::Vector6 &line, gtsam::Vector6 &line2, gtsam::Pose3 &ext, gtsam::Vector3 &p_abc, gtsam::Vector3& lvs_2);
gtsam::Vector2 project(gtsam::Vector3 &c_p, gtsam::Matrix3 &cam);

gtsam::Vector3 p_abc(0,0,0);
gtsam::Pose3 ext;
gtsam::Vector6 line, line2;

int main()
{
    const double mean = 0.0;
    const double stddev = 0.02;

    const double std_td = 0.25;

    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    std::normal_distribution<double> noise(0.0, std_td);

    std::ofstream egm_output;
    egm_output.open("egm.txt", std::ios_base::trunc);

    std::ofstream cam_output;
    cam_output.open("cam.txt", std::ios_base::trunc);

    std::cout << "EGM INFO: \n";
    float base_time = 0;
    for(int idx = 0; idx < 14; idx++)
    {
        base_time = idx* (T_MAX + 1000); 
        for(float t = 0 ; t < T_MAX; t = t+ T_CYCLE_EGM)
        {
            gtsam::Rot3  rot = generate_rot_end(idx);
            gtsam::Pose3 egm = generate_r_end(t, rot);
            gtsam::Vector6 egm_ = gtsam::Pose3::Logmap(egm);
            gtsam::Vector3 eular_xyz = rot.xyz();
            gtsam::Vector3 t_end = egm.translation();
            

            float rev_t = t + noise(generator);
            std::cout << rev_t + base_time << " " << eular_xyz(0) << " " << eular_xyz(1) << " " << eular_xyz(2) << " " << t_end(0) << " " << t_end(1) << " " << t_end(2) << std::endl;
            egm_output << rev_t + base_time << " " << egm_(0) << " " << egm_(1) << " " << egm_(2) << " " << egm_(3) << " " << egm_(4) << " " << egm_(5) << std::endl;
        } 
    }
    

    gtsam::Pose3 ext = gtsam::Pose3(gtsam::Rot3(gtsam::I_3x3), gtsam::Vector3(EXT_X, EXT_Y, EXT_Z) );
    line << P_L_S_X, P_L_S_Y, P_L_S_Z, P_L_E_X, P_L_E_Y, P_L_E_Z;
    line2 << P_L_S_X_2, P_L_S_Y_2, P_L_S_Z_2, P_L_E_X_2, P_L_E_Y_2, P_L_E_Z_2;

    base_time = 0;
    gtsam::Vector3 lvs_2;
    for(int idx = 0; idx < 14; idx++)
    {
        base_time = idx* (T_MAX + 1000); 
        std::cout << "CAM INFO: \n";
        for(float t = 0; t < T_MAX; t = t + T_CYCLE_CAM)
        {
            gtsam::Rot3  rot = generate_rot_end(idx);
            gtsam::Pose3 egm = generate_r_end(t, rot);
            gtsam::Vector3 lvs_coordiante = generate_lvs(egm, line, line2, ext, p_abc, lvs_2);
            lvs_coordiante = lvs_coordiante + gtsam::Vector3(dist(generator), dist(generator), dist(generator));
            lvs_2 = lvs_2 + gtsam::Vector3(dist(generator), dist(generator), dist(generator));

            float rev_t = t + T_D + noise(generator);
            std::cout
                << rev_t + base_time << " " << lvs_coordiante(0) << " " << lvs_coordiante(1) << " " << lvs_coordiante(2) << std::endl;
            cam_output << rev_t+base_time << " " << lvs_coordiante(0) << " " << lvs_coordiante(1) << " " << lvs_coordiante(2) 
            << " " << lvs_2(0) << " " << lvs_2(1) << " " << lvs_2(2) << std::endl;
        } 
    }
    return 0;
}

#define D2R(a) (a/180.0*M_PI)

gtsam::Rot3 generate_rot_end(int index)
{
    std::vector<gtsam::Vector3> inr;
    inr.push_back(gtsam::Vector3(0,       0,       0) );
    inr.push_back(gtsam::Vector3(D2R(5),  0,       0) );
    inr.push_back(gtsam::Vector3(0,       D2R(5),  0) );
    inr.push_back(gtsam::Vector3(D2R(-5), 0,       0) );
    inr.push_back(gtsam::Vector3(0,       D2R(-5), 0) );
    inr.push_back(gtsam::Vector3(D2R(5) , D2R(5),  D2R(5)) );
    inr.push_back(gtsam::Vector3(D2R(-5), D2R(-5), D2R(-5)) );
    inr.push_back(gtsam::Vector3(D2R(-5), D2R(5),  D2R(-5)) );
    inr.push_back(gtsam::Vector3(D2R(5) , D2R(-5), D2R(5)) );

    inr.push_back(gtsam::Vector3(0,       D2R(-5), 0) );
    inr.push_back(gtsam::Vector3(D2R(3) , D2R(4),  D2R(5)) );
    inr.push_back(gtsam::Vector3(D2R(-5), D2R(-2), D2R(-5)) );
    inr.push_back(gtsam::Vector3(D2R(-6), D2R(-2), D2R(-5)) );
    inr.push_back(gtsam::Vector3(D2R(3) , D2R(4),  D2R(1)) );

    gtsam::Rot3 rot;
    gtsam::Vector3 inri = inr[index];

    rot = gtsam::Rot3::Rz(inri.z())* gtsam::Rot3::Ry(inri.y())* gtsam::Rot3::Rx(inri.x());
    
    return rot;
}

gtsam::Pose3 generate_r_end(float t, gtsam::Rot3 &rot)
{
    float H = 20.0; // mm
    float _dy  = H* sin(t/1000 *1.5* M_PI);
    float _dx = VEL_X* t;
    float _dz = 0.5* H* sin(t/1000 *1.5* M_PI);
    
    gtsam::Vector3 position(P_S_X + _dx, P_S_Y + _dy, P_S_Z + _dz);

    return gtsam::Pose3(rot, position);
}

gtsam::Vector3 generate_lvs(gtsam::Pose3& end, gtsam::Vector6 &line, gtsam::Vector6 &line2, gtsam::Pose3 &ext, gtsam::Vector3 &p_abc, gtsam::Vector3& lvs_2)
{
    gtsam::Pose3 b_cam = end* ext;
    gtsam::Pose3 c_cam_b = b_cam.inverse();

    gtsam::Vector3 c_line_a = c_cam_b.rotation().matrix()* line.head(3) + c_cam_b.translation();
    gtsam::Vector3 c_line_b = c_cam_b.rotation().matrix()* line.tail(3) + c_cam_b.translation();
    std::cout << c_line_a << " " << c_line_b << std::endl;
    float it =  - c_line_a(0) / (c_line_b(0) - c_line_a(0) );
    gtsam::Vector3 lvs_1 = it* (c_line_b - c_line_a) + c_line_a;

    c_line_a = c_cam_b.rotation().matrix()* line2.head(3) + c_cam_b.translation();
    c_line_b = c_cam_b.rotation().matrix()* line2.tail(3) + c_cam_b.translation();
    std::cout << c_line_a << " " << c_line_b << std::endl;
    it =  - c_line_a(0) / (c_line_b(0) - c_line_a(0) );
    lvs_2 = it* (c_line_b - c_line_a) + c_line_a;

    return lvs_1;
}

gtsam::Vector2 project(gtsam::Vector3 &c_p, gtsam::Matrix3 &cam)
{

}
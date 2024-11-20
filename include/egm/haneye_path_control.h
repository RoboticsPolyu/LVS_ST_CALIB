#include <iostream>
#include <gtsam_wrapper.h>


class HandeyePathPlaner
{
public:
    struct path_point
    {
        float x, y, z, eular_x, eular_y, eular_z;
        float qw, qx, qy, qz;

        void operator()(const path_point & my_struct)
        {
            x = my_struct.x;
            y = my_struct.y;
            z = my_struct.z;
            eular_x = my_struct.eular_x;
            eular_y = my_struct.eular_y;
            eular_z = my_struct.eular_z;
            qw = my_struct.qw;
            qx = my_struct.qx;
            qy = my_struct.qy;
            qz = my_struct.qz;
        }
    };

    HandeyePathPlaner()
    {
        planed_point_.eular_x = 0;
        planed_point_.eular_y = 0;
        planed_point_.eular_z = 0;
        planed_point_.x = 0;
        planed_point_.y = 0;
        planed_point_.z = 0;
        planed_point_.qw = 0.0;
        planed_point_.qx = 0.0;
        planed_point_.qy = 0.0;
        planed_point_.qz = 0.0;
        delta_y_ = 0.0;
        delta_z_ = 0.0;
        ori_flag_ = true;
    }

    void InitFirstPoint(float init_x, float init_y, float init_z, float eular_x, float eular_y, float eular_z, float linear_speed, float radius)
    {
        init_path_point_.x = init_x;
        init_path_point_.y = init_y;
        init_path_point_.z = init_z;
        init_path_point_.eular_x = eular_x;
        init_path_point_.eular_y = eular_y; 
        init_path_point_.eular_z = eular_z;  
        timestamp_ = 0.0;
        last_timestamp_ = 0.0;
        linear_speed_ = linear_speed;
        radius_ = radius;
        duration_ = 0.004;
        camera_stable_time_ = 1.0; // stop camera to a fix position 
        camera_stable_sumt_ = 0;
        onestep_time_ = 1.0;
        onesteo_cur_time_ = 0.0;
    };

    bool PlanNext(float dur, path_point& planed_path_point);

    void PlanTest(float dur, path_point& planed_path_point);

private:
    path_point init_path_point_;
    path_point planed_point_;

    float timestamp_;
    float last_timestamp_;

    float linear_speed_;

    float radius_;
    float duration_;

    float camera_stable_time_;
    float camera_stable_sumt_;
    float onestep_time_;
    float onesteo_cur_time_;

    float delta_y_;
    float delta_z_;
    bool ori_flag_;
    
};

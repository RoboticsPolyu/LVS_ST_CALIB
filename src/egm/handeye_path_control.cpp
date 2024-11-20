#include "egm/haneye_path_control.h"
#include <math.h>

bool HandeyePathPlaner::PlanNext(float dur, path_point& planed_path_point)
{
}

void HandeyePathPlaner::PlanTest(float dur, path_point& planed_path_point)
{
    timestamp_ = timestamp_ + dur;
    float linear_duration = radius_/ linear_speed_;
    float angle_y = 0.5;

    if(timestamp_ <= linear_duration)
    {
        planed_path_point.x = init_path_point_.x;
        planed_path_point.y = init_path_point_.y + linear_speed_* timestamp_;
        planed_path_point.z = init_path_point_.z;
    }
    else
    {
        float step_angle = 0.36 / 4;
        onesteo_cur_time_ += dur;
        if(onesteo_cur_time_ >= onestep_time_ && onesteo_cur_time_ < onestep_time_ + camera_stable_time_)
        {
            camera_stable_sumt_  += dur;
        }
        else if(onesteo_cur_time_ >= onestep_time_ + camera_stable_time_)
        {
            onesteo_cur_time_ = 0;
        }

        float x_p = radius_* std::sin(step_angle / 180.0* M_PI*(timestamp_- linear_duration - camera_stable_sumt_)/duration_);
        float y_p = radius_* std::cos(step_angle / 180.0* M_PI*(timestamp_- linear_duration - camera_stable_sumt_)/duration_);
        
        planed_path_point.x = init_path_point_.x + x_p;
        planed_path_point.y = init_path_point_.y + y_p;
        planed_path_point.z = init_path_point_.z;

        planed_path_point.eular_x = init_path_point_.eular_x;
        
        if(delta_y_ < 30.0)
        {
           delta_y_ = (timestamp_- linear_duration - camera_stable_sumt_)* angle_y;  
        }
        if(delta_z_ > 180.0)
        {
            ori_flag_ = false;
        }
        if(delta_z_ < -180.0)
        {
            ori_flag_  = true;
        }
        if(ori_flag_)
        {
            if(onesteo_cur_time_ >=0 && onesteo_cur_time_ < onestep_time_)
            {
                delta_z_ += step_angle;
            }
        }
        else
        {
            if(onesteo_cur_time_ >=0 && onesteo_cur_time_ < onestep_time_)
            {
                delta_z_ -= step_angle;
            }
        }

        planed_path_point.eular_y = init_path_point_.eular_y + delta_y_; //+ step_angle*(timestamp_- linear_duration - camera_stable_sumt_)/duration_;
        planed_path_point.eular_z = init_path_point_.eular_z + delta_z_; // + step_angle*(timestamp_- linear_duration - camera_stable_sumt_)/duration_;
        
        // std::cout << init_path_point_.eular_y + step_angle*(timestamp_- linear_duration - camera_stable_sumt_)/duration_ << std::endl;

        // gtsam::Vector3 xyz = gtsam::Vector3(init_path_point_.eular_x, init_path_point_.eular_y + 5, 
        //     init_path_point_.eular_z + step_angle*(timestamp_- linear_duration - camera_stable_sumt_)/duration_);
        // gtsam::Rot3 rot3 = gtsam::Rot3::RzRyRx(xyz);

        // gtsam::Quaternion quat(rot3.toQuaternion());
        // planed_path_point.qw = quat.w();
        // planed_path_point.qx = quat.x();
        // planed_path_point.qy = quat.y();
        // planed_path_point.qz = quat.z();
    }

    last_timestamp_ = timestamp_;
}


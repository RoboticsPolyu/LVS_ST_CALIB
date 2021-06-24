#include <ros/ros.h>
#include <abb_egm_wrapper.h>

class abb_egm_wrapper_app
{
    public:
        abb_egm_wrapper_app(int argc, char** argv)
        {
            ros::init(argc, argv, "abb_egm_wrapper_test");
            ros::NodeHandle node_handle;
        }

        void setup()
        {
            first_callback_ = true;

            abb_robot::egm_base_wrapper::abb_egm_wrapper_config config;
            config.port = 6512;
            egm_controler_ptr_ = std::make_shared<abb_robot::egm_controler_wrapper>(config);
            egm_controler_ptr_->set_egm_callback(std::bind(&abb_egm_wrapper_app::abb_egm_callback, this, std::placeholders::_1, std::placeholders::_2));

            ros::spin();
        }

    private:
        void abb_egm_callback(int seq, abb_robot::egm_controler_wrapper::CartesianPose pose)
        {
            double time = seq/((double) 250);
            std::cout << " --------------------------------- " << std::endl;
            std::cout << "callbak seq: " << seq << std::endl;

            if(first_callback_)
            {
                initial_pose_.CopyFrom(pose);
                first_callback_ = false;
            }
            else
            {
                // Compute references for position (along X-axis), and orientation (around Y-axis).
                double position_reference = initial_pose_.position().x() + position_amplitude_*(1.0 + std::sin(2.0*M_PI*frequency_*time - 0.5*M_PI));
                double orientation_reference = initial_pose_.euler().y() + orientation_amplitude_*(1.0 + std::sin(2.0*M_PI*frequency_*time - 0.5*M_PI));

                std::cout <<  "next pos_x " << position_reference << " [mm] | " << "y orientation = " << orientation_reference << " [deg]" << std::endl;

                next_pose_.mutable_position()->set_x(position_reference);
                next_pose_.mutable_euler()->set_y(orientation_reference);

                egm_controler_ptr_->set_next_pose(next_pose_);
            }
        }

        std::shared_ptr<abb_robot::egm_controler_wrapper> egm_controler_ptr_;
        abb_robot::egm_controler_wrapper::CartesianPose initial_pose_;
        abb_robot::egm_controler_wrapper::CartesianPose next_pose_;

        double position_amplitude_ = 100.0;
        double orientation_amplitude_ = -10.0;
        double frequency_ = 0.25;
        bool first_callback_;
        
};



int main(int argc, char** argv)
{
    abb_egm_wrapper_app abb_egm_app(argc, argv);
    abb_egm_app.setup();

    return 0;
}
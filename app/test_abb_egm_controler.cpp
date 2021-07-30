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

            abb_robot::EgmBaseWrapper::AbbEgmWrapperConfig config;
            config.port = 6512;
            egm_controler_ptr_ = std::make_shared<abb_robot::EgmControlerWrapper>(config);
            egm_controler_ptr_->SetEgmCallback(std::bind(&abb_egm_wrapper_app::abb_egm_callback, this, std::placeholders::_1));

            ros::spin();
        }

    private:
        void abb_egm_callback(abb_robot::EgmControlerWrapper::CtrlPoint ctrl_point)
        {
            double time = ctrl_point.sequence /((double) 250);
            std::cout << " --------------------------------- " << std::endl;
            std::cout << "callbak seq: " <<  ctrl_point.sequence << std::endl;

            if(first_callback_)
            {
                initail_control_point_(ctrl_point);
                first_callback_ = false;
            }

                // Compute references for position (along X-axis), and orientation (around Y-axis).
            double position_reference = initail_control_point_.value[0] + position_amplitude_*(1.0 + std::sin(2.0*M_PI*frequency_*time - 0.5*M_PI));
            double orientation_reference = initail_control_point_.value[5] + orientation_amplitude_*(1.0 + std::sin(2.0*M_PI*frequency_*time - 0.5*M_PI));

            std::cout <<  "next pos_x " << position_reference << " [mm] | " << "y orientation = " << orientation_reference << " [deg]" << std::endl;

            next_control_point_.value[0] = position_reference;
            next_control_point_.value[5] = orientation_reference;

            egm_controler_ptr_->SetNextCtrlPoint(next_control_point_);

        }

        std::shared_ptr<abb_robot::EgmControlerWrapper> egm_controler_ptr_;
        abb_robot::EgmControlerWrapper::CtrlPoint initail_control_point_;
        abb_robot::EgmControlerWrapper::CtrlPoint next_control_point_;

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
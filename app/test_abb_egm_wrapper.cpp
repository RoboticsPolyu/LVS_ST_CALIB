#include <ros/ros.h>
#include <abb_egm_wrapper.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "abb_egm_wrapper_test");
    ros::NodeHandle node_handle;

    abb_robot::egm_base_wrapper::abb_egm_wrapper_config config;
    config.port = 6512;
    abb_robot::egm_controler_wrapper egm_wrapper_instance(config);

    if(egm_wrapper_instance.egm_start_communication())
    {
        std::cout << "egm communication running";
    }

    return 0;
}
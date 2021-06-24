#include <iostream>
#include <abb_egm_wrapper.h>
#include <abb_exception.h>

namespace abb_robot
{
    egm_base_wrapper::egm_base_wrapper()
    {
    }

    egm_controler_wrapper::egm_controler_wrapper(egm_controler_wrapper::abb_egm_wrapper_config& config)
    {
        abb_egm_config_.port = config.port;

        egm_interface_ = std::make_shared<abb::egm::EGMControllerInterface>(io_service_, abb_egm_config_.port);

        if(!egm_interface_->isInitialized())
        {   
            std::string error = "EGM interface failed to initialize (e.g. due to port already bound)";
            std::cout << error << std::endl;
            throw AbbException(error);
        }

        // Spin up a thread to run the io_service.
        thread_group_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));
        egm_start_communication();
        abb_egm_controler_thread_ = std::shared_ptr<std::thread >(new std::thread(std::bind(&egm_controler_wrapper::start, this)));

    }

    egm_controler_wrapper::~egm_controler_wrapper()
    {
        io_service_.stop();
        thread_group_.join_all();
    }

    void egm_controler_wrapper::start()
    {
        int sequence_number;
        CartesianPose initial_pose;
        CartesianPose callback_pose;

        Joints initial_joints;
        Joints callback_joints;

        while(true)
        {
            // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
            if(egm_interface_->waitForMessage(500))
            {
                // Read the message received from the EGM client.
                egm_interface_->read(&input_);
                sequence_number = input_.header().sequence_number();
                callback_pose = input_.feedback().robot().cartesian().pose();

                egm_callback_hander_(sequence_number, callback_pose);

                if(sequence_number == 0)
                {
                    // Reset all references, if it is the first message.
                    output_.Clear();
                    initial_pose.CopyFrom(callback_pose);
                    std::cout << "initial pose x : " << callback_pose.position().x() << std::endl;
                    std::cout << "initial pose y : " << callback_pose.position().y() << std::endl;

                    pose_next_.CopyFrom(initial_pose);
                    output_.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(initial_pose);
                }
                else
                {
                    mutex_pose_.lock();
                    output_.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(pose_next_);
                    mutex_pose_.unlock();

                    // if(sequence_number%egm_rate == 0)
                    // {
                    //     std::cout << "References: " <<
                    //                 "X position = " << pose_next_.position().x() << " [mm] | " <<
                    //                 "Y position = " << pose_next_.position().y() << " [mm] | " << 
                    //                 "Y position = " << pose_next_.position().z() << " [mm] | " << std::endl;
                    // }
                }

                // Write references back to the EGM client.
                egm_interface_->write(output_);
            }
        }

    }

    bool egm_controler_wrapper::egm_start_communication()
    {
        std::cout << "1: Wait for an EGM communication session to start..." << std::endl;
        bool wait = true;
        
        while(wait)
        {
            if(egm_interface_->isConnected())
            {
                if(egm_interface_->getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
                {
                    std::cout << "RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program." << std::endl;
                }
                else
                {
                    wait = egm_interface_->getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
                }
            }

            sleep(0.5);
        }
        return true;
    }

    void egm_controler_wrapper::set_next_pose(CartesianPose& next_pose)
    {
        mutex_pose_.lock();
        pose_next_.CopyFrom(next_pose);
        mutex_pose_.unlock();
    }

    egm_trajectory_wrapper::egm_trajectory_wrapper(abb_egm_wrapper_config& config)
    {
    }

    void egm_trajectory_wrapper::add_egm_trajectory(TrajectoryGoal& trajectory)
    {
        egm_interface_->addTrajectory(trajectory);
    }

    void egm_trajectory_wrapper::set_point(TrajectoryGoal& trajectory, bool reach, float64_t duration, 
        float64_t value1, float64_t value2, float64_t value3, float64_t value4, float64_t value5, float64_t value6)
    {
        PointGoal* point_goal;
        point_goal = trajectory.add_points();
        point_goal->set_reach(reach);
        point_goal->set_duration(duration);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value1);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value2);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value3);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value4);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value5);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value6);
    }

}
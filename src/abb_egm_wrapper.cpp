#include <iostream>
#include <abb_egm_wrapper.h>
#include <abb_exception.h>

#include <ros/ros.h>

namespace abb_robot
{
    EgmBaseWrapper::EgmBaseWrapper()
    {
    }

    EgmControlerWrapper::EgmControlerWrapper(EgmControlerWrapper::AbbEgmWrapperConfig& config)
    {
        abb_egm_config_.port = config.port;
        abb_egm_config_.egm_mode = config.egm_mode;

        abb::egm::BaseConfiguration configuration;
        if(abb_egm_config_.egm_mode == EgmMode_t::JOINT_VEL_CONTROLER || abb_egm_config_.egm_mode == EgmMode_t::POSE_VEL_CONTROLER)
        {
            configuration.use_velocity_outputs = true;
        }

        egm_interface_ = std::make_shared<abb::egm::EGMControllerInterface>(io_service_, abb_egm_config_.port, configuration);

        if(!egm_interface_->isInitialized())
        {   
            std::string error = "EGM interface failed to initialize (e.g. due to port already bound)";
            std::cout << error << std::endl;
            throw AbbException(error);
        }

        // Spin up a thread to run the io_service.
        thread_group_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));
                std::cout << "Wait for an EGM communication session to Start..." << std::endl;
        
        bool wait = true;
        while(ros::ok() && wait)
        {
            if(egm_interface_->isConnected())
            {
                if(egm_interface_->getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
                {
                    std::cout << "RAPID execution state is UNDEFINED (might happen first time after controller Start/reStart). Try to reStart the RAPID program." << std::endl;
                }
                else
                {
                    wait = egm_interface_->getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
                }
            }

            sleep(0.5);
        }

        std::cout << "start egm controler thread " << std::endl;
        abb_egm_controler_thread_ = std::shared_ptr<std::thread >(new std::thread(std::bind(&EgmControlerWrapper::Start, this)));
    }
    
    EgmControlerWrapper::~EgmControlerWrapper()
    {
        io_service_.stop();
        thread_group_.join_all();
    }

    bool EgmBaseWrapper::PositionCloseby(abb::egm::wrapper::Cartesian p1, abb::egm::wrapper::Cartesian p2)
    {
        double threshold = 1.0; // [mm]

        return (std::abs(p1.x() - p2.x()) < threshold &&
                std::abs(p1.y() - p2.y()) < threshold &&
                std::abs(p1.z() - p2.z()) < threshold);
    }

    // Function to check if two joint messages are close to each other.
    bool EgmBaseWrapper::JointsCloseby(abb::egm::wrapper::Joints j1, abb::egm::wrapper::Joints j2)
    {
        bool result = j1.values_size() > 0 && j2.values_size() > 0;
        double threshold = 0.01; // [degrees]

        for (int i = 0; i < j1.values_size() && i < j2.values_size() && result; ++i)
        {
            result = std::abs(j1.values(i) - j2.values(i)) < threshold;
        }

        return result;
    }

    void EgmControlerWrapper::Start()
    {
        uint32_t sequence_number;
        CtrlPoint callback_ctrl_point;

        while(ros::ok())
        {
            // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
            if(egm_interface_->waitForMessage(500))
            {
                // Read the message received from the EGM client.
                egm_interface_->read(&input_);
                sequence_number = input_.header().sequence_number();
                callback_ctrl_point.sequence = sequence_number;

                if(abb_egm_config_.egm_mode == EgmMode_t::POSE_CONTROLER)
                {
                    CartesianPose callback_pose = input_.feedback().robot().cartesian().pose();
                    Convertor(callback_pose, callback_ctrl_point);
                }
                else if(abb_egm_config_.egm_mode == EgmMode_t::JOINT_CONTROLER)
                {
                    
                }
                else
                {
                    std::cout << "No such egm mode: " << abb_egm_config_.egm_mode << std::endl;
                    return;
                }
                
                std::cout << "initial value1 : " << callback_ctrl_point.value[0] << std::endl;
                std::cout << "initial value2 : " << callback_ctrl_point.value[1] << std::endl;
                egm_callback_hander_(callback_ctrl_point);

                mutex_ctrl_point_.lock();
                if(abb_egm_config_.egm_mode == EgmMode_t::POSE_CONTROLER)
                {
                    Convertor(next_ctrl_point_, output_.mutable_robot()->mutable_cartesian()->mutable_pose());
                }
                else if(abb_egm_config_.egm_mode == EgmMode_t::JOINT_CONTROLER)
                {
                    
                }
                else
                {
                    std::cout << "No such egm mode: " << abb_egm_config_.egm_mode << std::endl;
                    return;
                }
                mutex_ctrl_point_.unlock();
                
                // Write references back to the EGM client.
                egm_interface_->write(output_);
            }
        }
    }

    void EgmControlerWrapper::Convertor(const CartesianPose& pose, CtrlPoint& ctrl_point)
    {
        ctrl_point.value[0] = pose.position().x();
        ctrl_point.value[1] = pose.position().y();
        ctrl_point.value[2] = pose.position().z();
        ctrl_point.value[3] = pose.euler().x();
        ctrl_point.value[4] = pose.euler().y();
        ctrl_point.value[5] = pose.euler().x();
    }
    
    void EgmControlerWrapper::Convertor(const CtrlPoint& ctrl_point, CartesianPose* pose)
    {
        pose->mutable_position()->set_x(ctrl_point.value[0]);
        pose->mutable_position()->set_y(ctrl_point.value[1]);
        pose->mutable_position()->set_z(ctrl_point.value[2]);
        pose->mutable_euler()->set_x(ctrl_point.value[3]);
        pose->mutable_euler()->set_y(ctrl_point.value[4]);
        pose->mutable_euler()->set_z(ctrl_point.value[5]);
    }

    void EgmControlerWrapper::SetNextCtrlPoint(const CtrlPoint& next_ctrl_point)
    {
        mutex_ctrl_point_.lock();
        next_ctrl_point_(next_ctrl_point);
        mutex_ctrl_point_.unlock();
    }

    /* 
     * Egm Trajectory Wrapper
     */
    EgmTrajectoryWrapper::EgmTrajectoryWrapper(AbbEgmWrapperConfig& config)
    {
        abb_egm_config_.port = config.port;
        abb_egm_config_.egm_mode = config.egm_mode;

        abb::egm::BaseConfiguration configuration;
        if(abb_egm_config_.egm_mode == EgmMode_t::JOINT_VEL_CONTROLER || abb_egm_config_.egm_mode == EgmMode_t::POSE_VEL_CONTROLER)
        {
            configuration.use_velocity_outputs = true;
        }

        egm_interface_ = std::make_shared<abb::egm::EGMTrajectoryInterface>(io_service_, abb_egm_config_.port, configuration);

        if(!egm_interface_->isInitialized())
        {   
            std::string error = "EGM interface failed to initialize (e.g. due to port already bound)";
            std::cout << error << std::endl;
            throw AbbException(error);
        }

        // Spin up a thread to run the io_service.
        thread_group_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));

        bool wait = true;
        ROS_INFO("1: Wait for an EGM communication session to start...");
        while(ros::ok() && wait)
        {
            if(egm_interface_->isConnected())
            {
                if(egm_interface_->getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
                {
                    ROS_WARN("RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
                }
                else
                {
                    wait = egm_interface_->getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
                }
            }

            ros::Duration(0.5).sleep();
        }
    }

    void EgmTrajectoryWrapper::AddEgmTrajectoryExecute(TrajectoryGoal& trajectory)
    {
        egm_interface_->addTrajectory(trajectory);
    }

    void EgmTrajectoryWrapper::AddEgmTrajectoryExecute(const std::vector<CtrlPoint>& trajectory)
    {
        TrajectoryGoal _trajectory;
        for(auto iter = trajectory.begin(); iter != trajectory.end(); iter++)
        {
            SetPoint(_trajectory, iter->duration, iter->value[0], iter->value[1], iter->value[2], iter->value[3], iter->value[4], iter->value[5]);
        }
        egm_interface_->addTrajectory(_trajectory);

        ROS_INFO("ExecuteTrajectory(): Wait for the trajectory execution to finish...");
        abb::egm::wrapper::trajectory::ExecutionProgress execution_progress;
        bool wait = true;
        while(ros::ok() && wait)
        {
            ros::Duration(0.5).sleep();

            if(egm_interface_->retrieveExecutionProgress(&execution_progress))
            {
                wait = execution_progress.goal_active();
            }
        }
    }

    void EgmTrajectoryWrapper::AddEgmStaticGoalExecute(const std::vector<CtrlPoint>& static_points)
    {
        bool wait = true;
        while (ros::ok() && wait)
        {
            if (egm_interface_->retrieveExecutionProgress(&execution_progress_))
            {
                wait = !(execution_progress_.state() == abb::egm::wrapper::trajectory::ExecutionProgress_State_NORMAL);
            }

            ros::Duration(0.5).sleep();
        }
        abb::egm::wrapper::trajectory::StaticPositionGoal static_goal_1;
        
        ROS_INFO("Start static goal execution");
        egm_interface_->startStaticGoal();
        for(auto iter = static_points.begin(); iter != static_points.end(); iter++)
        {
            if(ros::ok())
            {
                static_goal_1.mutable_robot()->mutable_cartesian()->mutable_position()->set_x(iter->value[0]);
                static_goal_1.mutable_robot()->mutable_cartesian()->mutable_position()->set_y(iter->value[1]);
                static_goal_1.mutable_robot()->mutable_cartesian()->mutable_position()->set_z(iter->value[2]);

                static_goal_1.mutable_robot()->mutable_cartesian()->mutable_euler()->set_x(iter->value[3]);
                static_goal_1.mutable_robot()->mutable_cartesian()->mutable_euler()->set_y(iter->value[4]);
                static_goal_1.mutable_robot()->mutable_cartesian()->mutable_euler()->set_z(iter->value[5]);

                ROS_INFO("1.4: Set static goal");  
                egm_interface_->setStaticGoal(static_goal_1);
            }
        }

        wait = true;
        while (ros::ok() && wait)
        {
            if (egm_interface_->retrieveExecutionProgress(&execution_progress_))
            {
                wait = !PositionCloseby(static_goal_1.robot().cartesian().position(),
                    execution_progress_.inputs().feedback().robot().cartesian().pose().position());
            }
            ros::Duration(0.5).sleep();
        }

        if(ros::ok())
        {
            ROS_INFO("Finish static goal execution");
            egm_interface_->finishStaticGoal(true);
        }
    }

    void EgmTrajectoryWrapper::SetPoint(TrajectoryGoal& trajectory, float64_t duration, float64_t value1, float64_t value2, float64_t value3, 
        float64_t value4, float64_t value5, float64_t value6)
    {
        PointGoal* point_goal;
        point_goal = trajectory.add_points();
        point_goal->set_reach(true);
        point_goal->set_duration(duration);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value1);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value2);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value3);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value4);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value5);
        point_goal->mutable_robot()->mutable_joints()->mutable_position()->add_values(value6);
    }

}
#include <iostream>
#include <abb_egm_wrapper.h>
#include <abb_exception.h>

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
        EgmStartCommunication();
        abb_egm_controler_thread_ = std::shared_ptr<std::thread >(new std::thread(std::bind(&EgmControlerWrapper::Start, this)));

    }

    EgmControlerWrapper::~EgmControlerWrapper()
    {
        io_service_.stop();
        thread_group_.join_all();
    }

    void EgmControlerWrapper::Start()
    {
        uint32_t sequence_number;
        CtrlPoint callback_ctrl_point;

        while(true)
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

                mutex_pose_.lock();
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
                mutex_pose_.unlock();
                
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

    bool EgmControlerWrapper::EgmStartCommunication()
    {
        std::cout << "1: Wait for an EGM communication session to Start..." << std::endl;
        bool wait = true;
        
        while(wait)
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
        return true;
    }

    void EgmControlerWrapper::SetNextCtrlPoint(const CtrlPoint& next_ctrl_point)
    {
        mutex_pose_.lock();
        next_ctrl_point_(next_ctrl_point);
        mutex_pose_.unlock();
    }

    EgmTrajectoryWrapper::EgmTrajectoryWrapper(AbbEgmWrapperConfig& config)
    {
    }

    void EgmTrajectoryWrapper::AddEgmTrajectory(TrajectoryGoal& trajectory)
    {
        egm_interface_->addTrajectory(trajectory);
    }

    void EgmTrajectoryWrapper::SetPoint(TrajectoryGoal& trajectory, bool reach, float64_t duration, 
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
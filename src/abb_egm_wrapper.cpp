#include <iostream>
#include <abb_egm_wrapper.h>
#include <abb_exception.h>

namespace abb
{
    egm_wrapper::egm_wrapper(egm_wrapper::abb_egm_wrapper_config config)
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

    }

    bool egm_wrapper::egm_start_communication()
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

    egm_wrapper::~egm_wrapper()
    {
        io_service_.stop();
        thread_group_.join_all();
    }
}
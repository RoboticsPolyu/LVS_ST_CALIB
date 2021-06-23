#ifndef ABB_EGM_WRAPPER_H_
#define ABB_EGM_WRAPPER_H_

#include <abb_libegm/egm_controller_interface.h>

namespace abb
{
    class egm_wrapper
    {
        public:
            struct abb_egm_wrapper_config
            {
                /* data */
                unsigned short port;
            };
            
            egm_wrapper() = delete;
            
            egm_wrapper(abb_egm_wrapper_config config);

            bool egm_start_communication();

            ~egm_wrapper();

        private:
            std::shared_ptr<abb::egm::EGMControllerInterface> egm_interface_;
            boost::asio::io_service io_service_;
            boost::thread_group thread_group_;

            abb::egm::wrapper::Input input_;
            abb::egm::wrapper::CartesianPose initial_pose_;
            abb::egm::wrapper::Output output_;

            abb_egm_wrapper_config abb_egm_config_;

            const int egm_rate = 250.0;
    };
}

#endif
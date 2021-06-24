#ifndef ABB_EGM_WRAPPER_H_
#define ABB_EGM_WRAPPER_H_

#include <thread>
#include <memory>
#include <functional>
#include <vector>

#include <abb_libegm/egm_controller_interface.h>
#include <abb_libegm/egm_trajectory_interface.h>

namespace abb_robot
{
    class egm_base_wrapper
    {
        public:
            using float64_t = double;
            using TrajectoryGoal = abb::egm::wrapper::trajectory::TrajectoryGoal;
            using PointGoal = abb::egm::wrapper::trajectory::PointGoal;
            using CartesianPose = abb::egm::wrapper::CartesianPose;
            enum egm_mode_t: uint8_t
            {
                TRAJECTORY = 0,
                STATIC_JOINT_GOAL,
                STATIC_POSE_GOAL,
                JOINT_CONTROLER,
                POSE_CONTROLER,
                JOINT_VEL_CONTROLER,
                POSE_VEL_CONTROLER,
            };

            struct abb_egm_wrapper_config
            {
                /* data */
                unsigned short port;
                egm_mode_t egm_mode;
            };
            egm_base_wrapper();


        private:
    };

    class egm_controler_wrapper : public egm_base_wrapper
    {
        public:
            typedef std::function<void(int, CartesianPose&)> egm_callback;

            egm_controler_wrapper() = delete;
            egm_controler_wrapper(abb_egm_wrapper_config& config);

            bool egm_start_communication();
            ~egm_controler_wrapper();
            void start();
            
            void set_egm_callback(egm_callback hander)
            {
                egm_callback_hander_ = hander;
            }

            void add_next_pose(CartesianPose& next_pose);

        private:
            std::shared_ptr<abb::egm::EGMControllerInterface> egm_interface_;
            boost::asio::io_service io_service_;
            boost::thread_group thread_group_;

            abb::egm::wrapper::Input input_;
            abb::egm::wrapper::Output output_;
            abb_egm_wrapper_config abb_egm_config_;

            std::shared_ptr<std::thread> abb_egm_controler_thread_;
            egm_callback egm_callback_hander_;
            CartesianPose pose_next_;
            std::mutex mutex_pose_;

            const int egm_rate = 250.0;
    };

    class egm_trajectory_wrapper: public egm_base_wrapper
    {
        public:
            egm_trajectory_wrapper(abb_egm_wrapper_config& config);
            void add_egm_trajectory(TrajectoryGoal& trajectory);

        private:
            void set_point(TrajectoryGoal& trajectory, bool reach, float64_t duration, 
                float64_t value1, float64_t value2, float64_t value3, float64_t value4, float64_t value5, float64_t value6);

            std::vector<TrajectoryGoal> trajectory_goals_;

            std::shared_ptr<abb::egm::EGMTrajectoryInterface> egm_interface_;
    };

}

#endif
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
    class EgmBaseWrapper
    {
        public:
            using float64_t = double;
            using TrajectoryGoal = abb::egm::wrapper::trajectory::TrajectoryGoal;
            using PointGoal = abb::egm::wrapper::trajectory::PointGoal;
            using CartesianPose = abb::egm::wrapper::CartesianPose;
            using Joints = abb::egm::wrapper::Joints;

            typedef struct CtrlPoint
            {
                uint32_t sequence;
                float64_t value[6];
                void operator()(const CtrlPoint& ctrl_point)
                {   
                    sequence = ctrl_point.sequence;
                    for(uint32_t i = 0; i < 6; i++)
                    {
                        value[i] = ctrl_point.value[i];
                    }
                }
            }CtrlPoint;

            enum EgmMode_t: uint8_t
            {
                JOINT_TRAJECTORY = 0,
                POSE_TRAJECTORY,
                POSE_STATIC_JOINT_GOAL,
                STATIC_POSE_GOAL,
                JOINT_CONTROLER,
                POSE_CONTROLER,
                JOINT_VEL_CONTROLER,
                POSE_VEL_CONTROLER,
            };

            struct AbbEgmWrapperConfig
            {
                /* data */
                unsigned short port;
                EgmMode_t egm_mode;
            };
            EgmBaseWrapper();


        private:
    };

    class EgmControlerWrapper : public EgmBaseWrapper
    {
        public:
            typedef std::function<void(CtrlPoint&)> egm_callback;

            EgmControlerWrapper() = delete;
            EgmControlerWrapper(AbbEgmWrapperConfig& config);
            ~EgmControlerWrapper();
            
            /**
             * set egm callback function
             */
            void SetEgmCallback(egm_callback hander)
            {
                egm_callback_hander_ = hander;
            }

            /**
             * set target control point
             */
            void SetNextCtrlPoint(const CtrlPoint& next_ctrl_point);

        private: 
            bool EgmStartCommunication();

            void Start();

            void Convertor(const CartesianPose& pose, CtrlPoint& ctrl_point);
            
            void Convertor(const CtrlPoint& ctrl_point, CartesianPose* pose);

            std::shared_ptr<abb::egm::EGMControllerInterface> egm_interface_;
            boost::asio::io_service io_service_;
            boost::thread_group thread_group_;

            abb::egm::wrapper::Input input_;
            abb::egm::wrapper::Output output_;
            AbbEgmWrapperConfig abb_egm_config_;

            std::shared_ptr<std::thread> abb_egm_controler_thread_;
            egm_callback egm_callback_hander_;
            CtrlPoint next_ctrl_point_;

            std::mutex mutex_pose_;

            const int egm_rate = 250.0;
    };

    class EgmTrajectoryWrapper: public EgmBaseWrapper
    {
        public:
            EgmTrajectoryWrapper(AbbEgmWrapperConfig& config);
            void AddEgmTrajectory(TrajectoryGoal& trajectory);

        private:
            void SetPoint(TrajectoryGoal& trajectory, bool reach, float64_t duration, 
                float64_t value1, float64_t value2, float64_t value3, float64_t value4, float64_t value5, float64_t value6);

            std::vector<TrajectoryGoal> trajectory_goals_;

            std::shared_ptr<abb::egm::EGMTrajectoryInterface> egm_interface_;
    };

}

#endif
#include "abb_egm_wrapper.h"

class abb_egm_wrapper_app
{
    public:
        void Setup()
        {
            abb_robot::EgmBaseWrapper::AbbEgmWrapperConfig config;
            config.port = 6512;
            egm_trajectory_ptr_ = std::make_shared<abb_robot::EgmTrajectoryWrapper>(config);
        }

        bool TestTrajectory()
        {

        }
        
    private:
        std::shared_ptr<abb_robot::EgmTrajectoryWrapper> egm_trajectory_ptr_;
};

int main(void)
{
    
    return 0;
}
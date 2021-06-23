#include <abb_egm_wrapper.h>

int main(void)
{
    abb::egm_wrapper::abb_egm_wrapper_config config;
    config.port = 6512;
    abb::egm_wrapper egm_wrapper_instance(config);

    if(egm_wrapper_instance.egm_start_communication())
    {
        std::cout << "egm communication running";
    }
    
    return 0;
}
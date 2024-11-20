#ifndef ABB_EXCEPTION_H_
#define ABB_EXCEPTION_H_

#include <stdexcept>

namespace abb_robot
{

class AbbException: public std::runtime_error
{
    public:
    AbbException(const char* msg):
        std::runtime_error(msg)
    {
    }
    AbbException(const std::string& msg):
        std::runtime_error(msg)
    {
    }
};
}

#endif
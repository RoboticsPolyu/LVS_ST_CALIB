#ifndef __TD_FACTOR_H__
#define __TD_FACTOR_H__

#include <gtsam_wrapper.h>
#include <iostream>

namespace calibration
{

class PluckerLine
{
public:
    PluckerLine(const gtsam::Rot3& rot, const float m)
        : rot_(rot)
        , m_(m)
        //, rou_(rou)
        {}

    void Update(const gtsam::Vector4& Lw)
    {
        rot_ = gtsam::Rot3::Expmap(Lw.head(3));
        // rou_ = Lw(3);
        m_ = Lw(3);
    }

    float Distance(gtsam::Vector3& point, gtsam::Matrix13& D_p, gtsam::Matrix14& D_l);

    gtsam::Vector3 Distance(gtsam::Vector3& point, gtsam::Matrix3& D_p, gtsam::Matrix34& D_l);

    void plus();

    /*
    * convert mini representation to R6 vector
    */
    gtsam::Vector6 ToL();

private:
    gtsam::Rot3 rot_;
    float m_;
    // float rou_;

};

};

#endif // __TD_FACTOR_H__
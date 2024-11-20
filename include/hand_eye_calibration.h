#ifndef __HAND_EYE_CALIBRATION_H__
#define __HAND_EYE_CALIBRATION_H__

#include "gtsam_wrapper.h"
#include "types.h"
#include <vector>

using gtsam::symbol_shorthand::R;
using gtsam::symbol_shorthand::T;

namespace calibration
{

// Handeye calibration Rotation factor
class GTSAM_EXPORT RotFactor: public gtsam::NoiseModelFactor1<gtsam::Rot3>
{
    public:
        typedef gtsam::NoiseModelFactor1<gtsam::Rot3>   Base;
        typedef gtsam::noiseModel::Diagonal::shared_ptr SharedDiagonal;

        RotFactor(const gtsam::Key rot_key,
                  const gtsam::Rot3& rot_r1_r2,
                  const gtsam::Rot3& rot_c1_c2,
                  const SharedDiagonal& sigma)
                    : Base(sigma, rot_key), rot_r1_r2_(rot_r1_r2), rot_c1_c2_(rot_c1_c2)
        {
        }

        virtual ~RotFactor()
        {
        }

        gtsam::Vector evaluateError(const gtsam::Rot3& rot_r_c,
            boost::optional<gtsam::Matrix&> H_rot = boost::none) const
        {
            gtsam::Rot3 rotr1r2 = rot_r1_r2_;
            gtsam::Rot3 rotc1c2 = rot_c1_c2_;
            gtsam::Matrix33 H_e_part1, H_e_part2;
            
            // error = Log(r1*r) - Log(r*r2)
            gtsam::Vector error =  gtsam::Rot3::Logmap(rotr1r2* rot_r_c, H_e_part1) - gtsam::Rot3::Logmap(rot_r_c* rotc1c2, H_e_part2);
            //std::cout << "error: " << error << std::endl;
            
            // gtsam::Vector error = rot_r_c* gtsam::Rot3::Logmap(rotr1r2) - gtsam::Rot3::Logmap(rotc1c2);

            if(H_rot)
            {
                // Jacbian
                *H_rot = H_e_part1* gtsam::Matrix33::Identity() - H_e_part2* rotc1c2.inverse().matrix();
                // *H_rot = -rot_r_c.matrix()* gtsam::skewSymmetric(gtsam::Rot3::Logmap(rotr1r2));

                // std::cout << "H_rot: " << *H_rot << std::endl;
            }

            return error;
        }

    private:
        gtsam::Rot3 rot_r1_r2_;
        gtsam::Rot3 rot_c1_c2_;
};

// Handeye translation factor
class TlFactor: public gtsam::NoiseModelFactor1<gtsam::Vector3>
{
    public:
    typedef gtsam::NoiseModelFactor1<gtsam::Vector3>   Base;
    typedef gtsam::noiseModel::Diagonal::shared_ptr SharedDiagonal;

    TlFactor(const gtsam::Key tl_key,
            const gtsam::Rot3& rot_r_c,
            const gtsam::Rot3& RB,
            const gtsam::Vector3& tB,
            const gtsam::Vector3& tA,
            const SharedDiagonal& sigma)
                : Base(sigma, tl_key), rot_r_c_(rot_r_c), rot_r1_r2_(RB), t_c1_c2_(tB), t_r1_r2_(tA)
    {
    }

    virtual ~TlFactor()
    {
    }

    gtsam::Vector evaluateError(const gtsam::Vector3& tl_r_c,
        boost::optional<gtsam::Matrix&> H = boost::none) const
    {
        gtsam::Vector3 err = (rot_r1_r2_.matrix() - gtsam::Matrix33::Identity()) * tl_r_c 
            - rot_r_c_.matrix() * t_r1_r2_ + t_c1_c2_;
        // std::cout << "error: \n" << err << std::endl;
        // std::cout << "tl_r_c: \n" << tl_r_c << std::endl;
        if(H)
        {
            *H = (rot_r1_r2_.matrix() - gtsam::I_3x3);
        }
        return err;
    }

private:
        const gtsam::Rot3 rot_r_c_;
        const gtsam::Rot3 rot_r1_r2_;

        const gtsam::Vector3 t_c1_c2_;
        const gtsam::Vector3 t_r1_r2_;
        
};

class HandeyeFactor: public gtsam::NoiseModelFactor2<gtsam::Rot3, gtsam::Vector3>
{
    public:
    typedef gtsam::NoiseModelFactor2<gtsam::Rot3, gtsam::Vector3>   Base;
    typedef gtsam::noiseModel::Diagonal::shared_ptr SharedDiagonal;

    HandeyeFactor(const gtsam::Key rot_key,
            const gtsam::Key tl_key,
            const gtsam::Rot3& RB,
            const gtsam::Vector3& tB,
            const gtsam::Vector3& tA,
            const SharedDiagonal& sigma)
                : Base(sigma, rot_key, tl_key), rot_r1_r2_(RB), t_c1_c2_(tB), t_r1_r2_(tA)
    {
    }

    virtual ~HandeyeFactor()
    {
    }

    gtsam::Vector evaluateError(const gtsam::Rot3& rot_r_c, const gtsam::Vector3& tl_r_c,
        boost::optional<gtsam::Matrix&> H_rot = boost::none,
        boost::optional<gtsam::Matrix&> H_tl = boost::none) const
    {
        gtsam::Vector3 err = (rot_r1_r2_.matrix() - gtsam::Matrix33::Identity()) * tl_r_c 
            - rot_r_c.matrix() * t_r1_r2_ + t_c1_c2_;
        // std::cout << "error: \n" << err << std::endl;
        // std::cout << "tl_r_c: \n" << tl_r_c << std::endl;
        if(H_tl)
        {
            *H_tl = (rot_r1_r2_.matrix() - gtsam::I_3x3);
        }

        if(H_rot)
        {
            *H_rot = gtsam::skewSymmetric(rot_r_c* t_r1_r2_);
        }
        return err;
    }

private:
        const gtsam::Rot3 rot_r1_r2_;

        const gtsam::Vector3 t_c1_c2_;
        const gtsam::Vector3 t_r1_r2_;
        
};

// handeye calibration 
class HandEyeCalib
{
    public:
        HandEyeCalib(std::vector<gtsam::Pose3>& robot, std::vector<gtsam::Pose3> &sensor):
            robot_(robot),
            sensor_(sensor)
        {
        }

        bool Calibrate(gtsam::Pose3& pose_r_c);

        bool CalibrationTranslation(gtsam::Vector3& t_r_c);

        // calibration rot rQc by svd method
        bool CalibrationRotationSVD(gtsam::Vector3& rot_r_c);        
        // calibrate rot rqc by gtsam 
        bool CalibrateRotationGtsam(gtsam::Vector3& rot_r_c);
        
        // calibrate translation by LS method
        bool CalibrateTranslationLS(gtsam::Vector3& rot_r_c, gtsam::Vector3& translation);
        // calibrate translation by gtsam
        bool CalibrateTranslationGtsam(gtsam::Vector3& rot_r_c, gtsam::Vector3& translation);

        bool CalibrateGtsam(gtsam::Vector3& rot_r_c, gtsam::Vector3& translation);
    private:
        std::vector<gtsam::Pose3> robot_;
        std::vector<gtsam::Pose3> sensor_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif
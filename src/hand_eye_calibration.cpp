#include "hand_eye_calibration.h"
#include <opencv2/opencv.hpp>

namespace calibration
{   
    bool HandEyeCalib::Calibrate(gtsam::Pose3& pose_l_r)
    {
        gtsam::Vector3 rot;
        gtsam::Vector3 t;
        if(!CalibrationRotationSVD(rot))
        {
            return false;
        }
        if(!CalibrationTranslation(t))
        {
            return false;
        }
        pose_l_r = gtsam::Pose3(gtsam::Rot3::Expmap(rot), t);
        return true;
    }
     
    bool HandEyeCalib::CalibrationRotationSVD(gtsam::Vector3& rot_r_c)
    {
        std::vector<gtsam::Rot3> rot_rr, rot_cc;
        for(uint i = 0; i< robot_.size() - 1; i++)
        {
            gtsam::Rot3 r_1 = robot_[0].rotation();
            gtsam::Rot3 r_2 = robot_[i+1].rotation();
            gtsam::Rot3 rot_r1_r2 = r_1.between(r_2);
            rot_rr.push_back(rot_r1_r2);
            gtsam::Rot3 r_c1_p = sensor_[0].rotation();
            gtsam::Rot3 r_c2_p = sensor_[i+1].rotation();
            gtsam::Rot3 rot_c1_c2 = r_c1_p* r_c2_p.inverse();
            rot_cc.push_back(rot_c1_c2);
        }

        Eigen::MatrixXd h_matrixs(rot_rr.size()*4, 4);
        h_matrixs.setZero();

        for(int i = 0; i < rot_rr.size(); i++)
        {
            Quaternion rq = rot_rr[i].toQuaternion();
            Quaternion cq = rot_cc[i].toQuaternion();

            Eigen::Matrix4d quat_left, quat_right;
            Eigen::Vector3d q = rq.vec();
            float64_t w = rq.w();
            quat_left.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() + gtsam::skewSymmetric(q);
            quat_left.block<3, 1>(0, 3) = q;
            quat_left.block<1, 3>(3, 0) = -q.transpose();
            quat_left(3, 3) = w;

            q = cq.vec();
            w = cq.w();
            quat_right.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() - gtsam::skewSymmetric(q);
            quat_right.block<3, 1>(0, 3) = q;
            quat_right.block<1, 3>(3, 0) = -q.transpose();
            quat_right(3, 3) = w;

            h_matrixs.block<4, 4>(i * 4, 0) = quat_left - quat_right;
        }
       
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(h_matrixs, Eigen::ComputeFullU | Eigen::ComputeFullV);
        std::cout << svd.matrixV() << std::endl;
        
        Eigen::Matrix<float64_t, 4, 1> x = svd.matrixV().col(3); // xyzw ordor
        Eigen::Quaterniond estimated_R(x);
        // Eigen::Matrix3d rot_matrix = estimated_R.toRotationMatrix();
        Eigen::Vector4d ric_cov = svd.singularValues();
        std::cout << "ric_cov: " << ric_cov << std::endl;
        rot_r_c = (gtsam::Vector3)gtsam::Rot3::Logmap(gtsam::Rot3(estimated_R.toRotationMatrix()));

        std::cout << rot_r_c << std::endl;
        gtsam::Rot3 rot3_rc(estimated_R.toRotationMatrix());

        for(int i=0; i < rot_rr.size(); i++)
        {
            gtsam::Rot3 rot_r_t = robot_[i].rotation()* gtsam::Rot3(estimated_R.toRotationMatrix())* sensor_[i].rotation();
            gtsam::Vector3 rot_eular = rot_r_t.matrix().eulerAngles(0, 1, 2)/M_PI*180;
            std::cout << "Target Euler: " << rot_eular(0) << " " << rot_eular(1) << " " << rot_eular(2) << " degree." << std::endl;
            // std::cout << "rot_rt: " << gtsam::Rot3::Logmap() << std::endl;
            // std::cout << "rot _rc rotcc: " << gtsam::Rot3::Logmap(rot3_rc* rot_cc[i]) << std::endl;
            // std::cout << "error : " << gtsam::Rot3::Logmap((rot_rr[i]* rot3_rc).inverse()* (rot3_rc* rot_cc[i])) << std::endl;
        }
        return true;
    }

    double cal_stdev(std::vector<double>& resultSet)
    {
        double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
        double mean =  sum / resultSet.size(); //均值
    
        double accum  = 0.0;
        std::for_each (std::begin(resultSet), std::end(resultSet), [&](const double d) {
            accum  += (d-mean)*(d-mean);
        });
        
        double stdev = sqrt(accum/(resultSet.size()-1)); //方差
        return stdev;
    }
	
    bool HandEyeCalib::CalibrateRotationGtsam(gtsam::Vector3& rot_r_c)
    {
        gtsam::LevenbergMarquardtParams parameters;
        parameters.absoluteErrorTol = 1e-8;
        parameters.relativeErrorTol = 1e-8;
        parameters.maxIterations = 500;
        parameters.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
        parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_value;

        // auto rot_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
        auto rot_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.001, 0.001, 0.001));

        std::vector<gtsam::Rot3> rot_rr, rot_cc;
        for(uint i = 0; i< robot_.size() - 1; i++)
        {
            gtsam::Rot3 r_1 = robot_[0].rotation();
            gtsam::Rot3 r_2 = robot_[i+1].rotation();
            gtsam::Rot3 rot_r1_r2 = r_1.between(r_2);
            rot_rr.push_back(rot_r1_r2);
            // std::cout << "rot rr: " << rot_r1_r2.matrix() << std::endl;
            gtsam::Rot3 r_c_p1 = sensor_[0].rotation();
            gtsam::Rot3 r_c_p2 = sensor_[i+1].rotation();
            gtsam::Rot3 rot_c1_c2 = r_c_p1* r_c_p2.inverse();
            rot_cc.push_back(rot_c1_c2);
            // std::cout << "rot cc: " << rot_c1_c2.matrix() << std::endl;
            RotFactor rot_factor(R(0), rot_r1_r2, rot_c1_c2, rot_noise);
            graph.add(rot_factor);
        }

        for(uint i = 0; i< robot_.size() - 1; i++)
        {
            gtsam::Rot3 r_1 = robot_[i].rotation();
            gtsam::Rot3 r_2 = robot_[i+1].rotation();
            gtsam::Rot3 rot_r1_r2 = r_1.between(r_2);
            rot_rr.push_back(rot_r1_r2);
            // std::cout << "rot rr: " << rot_r1_r2.matrix() << std::endl;
            gtsam::Rot3 r_c_p1 = sensor_[i].rotation();
            gtsam::Rot3 r_c_p2 = sensor_[i+1].rotation();
            gtsam::Rot3 rot_c1_c2 = r_c_p1* r_c_p2.inverse();
            rot_cc.push_back(rot_c1_c2);
            // std::cout << "rot cc: " << rot_c1_c2.matrix() << std::endl;
            RotFactor rot_factor(R(0), rot_r1_r2, rot_c1_c2, rot_noise);
            graph.add(rot_factor);
        }

        initial_value.insert(R(0), gtsam::Rot3::Expmap(rot_r_c));
        
        std::cout << "###################### init optimizer ######################" << std::endl;
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

        std::cout << "###################### begin optimize ######################" << std::endl;
        gtsam::Values result = optimizer.optimize();

        gtsam::Rot3 optbRo = result.at<Rot3>(R(0));
        std::cout << "Nonliear optbRo: " << gtsam::Rot3::Logmap(optbRo) << std::endl;
        rot_r_c = gtsam::Rot3::Logmap(optbRo);
        gtsam::Marginals marginals(graph, result);

        std::cout << "rot cov: " << marginals.marginalCovariance(R(0)) << std::endl;

        for(int i=0; i < robot_.size(); i++)
        {
            gtsam::Rot3 rot_r_t = robot_[i].rotation()* gtsam::Rot3(optbRo)* sensor_[i].rotation();
            gtsam::Vector3 rot_eular = rot_r_t.matrix().eulerAngles(0, 1, 2)/M_PI*180;
            std::cout << "Target Euler: " << rot_eular(0) << " " << rot_eular(1) << " " << rot_eular(2) << " degree." << std::endl;
            // std::cout << "rot_rt: " << gtsam::Rot3::Logmap() << std::endl;
            // std::cout << "rot _rc rotcc: " << gtsam::Rot3::Logmap(rot3_rc* rot_cc[i]) << std::endl;
            // std::cout << "error : " << gtsam::Rot3::Logmap((rot_rr[i]* rot3_rc).inverse()* (rot3_rc* rot_cc[i])) << std::endl;
        }
        return true;
    }

    bool HandEyeCalib::CalibrateTranslationLS(gtsam::Vector3& rot_r_c, gtsam::Vector3& translation)
    {
        Eigen::MatrixXd h_matrixs((robot_.size()-1)*2*3, 3);
        h_matrixs.setZero();
        Eigen::VectorXd Z_vector = Eigen::VectorXd::Zero((robot_.size()-1)*2*3);

        gtsam::Rot3 RX = gtsam::Rot3::Expmap(rot_r_c);

        for(uint i = 0; i< robot_.size() - 1; i++)
        {
            gtsam::Pose3 B1 = robot_[0];
            gtsam::Pose3 B2 = robot_[i+1];
            gtsam::Pose3 Bi = B1.inverse()* B2;
            gtsam::Vector3 tB = Bi.translation();
            gtsam::Rot3  RB = Bi.rotation();
            // std::cout << "tB: " << tB << " tB norm: " << tB.norm() << std::endl;
            gtsam::Pose3 A1 = sensor_[0];
            gtsam::Pose3 A2 = sensor_[i+1];
            gtsam::Pose3 Ai = A1* A2.inverse();
            gtsam::Vector3 tA = Ai.translation();
            gtsam::Rot3 RA = Ai.rotation();
            // std::cout << "tA: " << tA << " tA norm: " << tA.norm() << std::endl;
            // (RB - I3)* X = rot_r_c* tA - tB
            gtsam::Matrix33 XI = RB.matrix() - gtsam::I_3x3;

            h_matrixs.block<3, 3>(i * 3, 0) = XI;
            Eigen::Vector3d tt = RX.matrix()*tA - tB;
            Z_vector(i*3) = tt(0);
            Z_vector(i*3+ 1) = tt(1);
            Z_vector(i*3+ 2) = tt(2);
        }
        uint32_t i_base = (robot_.size()-1)*3-1;

        for(uint i = 0; i< robot_.size() - 1; i++)
        {
            gtsam::Pose3 B1 = robot_[i];
            gtsam::Pose3 B2 = robot_[i+1];
            gtsam::Pose3 Bi = B1.inverse()* B2;
            gtsam::Vector3 tB = Bi.translation();
            gtsam::Rot3  RB = Bi.rotation();
            // std::cout << "tB: " << tB << " tB norm: " << tB.norm() << std::endl;
            gtsam::Pose3 A1 = sensor_[i];
            gtsam::Pose3 A2 = sensor_[i+1];
            gtsam::Pose3 Ai = A1* A2.inverse();
            gtsam::Vector3 tA = Ai.translation();
            gtsam::Rot3 RA = Ai.rotation();
            // std::cout << "tA: " << tA << " tA norm: " << tA.norm() << std::endl;
            // (RB - I3)* X = rot_r_c* tA - tB
            gtsam::Matrix33 XI = RB.matrix() - gtsam::I_3x3;

            h_matrixs.block<3, 3>(i * 3 + i_base, 0) = XI;
            Eigen::Vector3d tt = RX.matrix()*tA - tB;
            Z_vector(i*3 + i_base) = tt(0);
            Z_vector(i*3+ 1 + i_base) = tt(1);
            Z_vector(i*3+ 2 + i_base) = tt(2);
        }

        Eigen::Vector3d abc_coeff = (h_matrixs.transpose()* h_matrixs).inverse()* h_matrixs.transpose()* Z_vector;
        // std::cout << "************************************** LS TRANSLATION of HE ***************************************" << std::endl;
        // std::cout << "h_matrix:" << h_matrixs << std::endl;
        // std::cout << "error: " << h_matrixs* abc_coeff - Z_vector<< std::endl;
        // std::cout << "Z_vector:" << Z_vector << std::endl;

        // std::cout << "L2S translation: " << abc_coeff << std::endl;
        translation = abc_coeff;

        gtsam::Pose3 rTc(RX, translation);
        std::cout << "rTc: " << gtsam::Pose3::Logmap(rTc) << std::endl;

        std::vector<double> x_mean, y_mean, z_mean;

        for(int i=0; i < robot_.size(); i++)
        {
            gtsam::Pose3 bTp = robot_[i]* rTc* sensor_[i];
            gtsam::Vector3 bPp = bTp.translation();
            x_mean.push_back(bPp[0]);
            y_mean.push_back(bPp[1]);
            z_mean.push_back(bPp[2]);
            std::cout << "bTP: " << bPp << std::endl;
        }

        std::cout << "stdev: " << std::endl;
        std::cout << cal_stdev(x_mean) << " " << cal_stdev(y_mean) << " " << cal_stdev(z_mean) << std::endl;
        return true;
    }

    bool HandEyeCalib::CalibrateTranslationGtsam(gtsam::Vector3& rot_r_c, gtsam::Vector3& translation)
    {
        gtsam::LevenbergMarquardtParams parameters;
        parameters.absoluteErrorTol = 1e-8;
        parameters.relativeErrorTol = 1e-8;
        parameters.maxIterations = 500;
        parameters.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
        parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_value;

        // auto rot_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
        auto rot_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.2));

        std::vector<gtsam::Rot3> rot_rr, rot_cc;

        for(uint i = 0; i< robot_.size() - 1; i++)
        {
            gtsam::Pose3 B1 = robot_[0];
            gtsam::Pose3 B2 = robot_[i+1];
            gtsam::Pose3 Bi = B1.inverse()* B2;
            gtsam::Vector3 tB = Bi.translation(); // t_r1_r2
            gtsam::Rot3  RB = Bi.rotation(); // rot_r1_r2
            // std::cout << "tB: " << tB << " tB norm: " << tB.norm() << std::endl;
            gtsam::Pose3 A1 = sensor_[0];
            gtsam::Pose3 A2 = sensor_[i+1];
            gtsam::Pose3 Ai = A1* A2.inverse();
            gtsam::Vector3 tA = Ai.translation(); //t_c1_c2
            gtsam::Rot3 RA = Ai.rotation(); // rot_c1_c2
            // std::cout << "tA: " << tA << " tA norm: " << tA.norm() << std::endl;
            // (RB - I3)* X = rot_r_c* tA - tB

            TlFactor tl_factor(T(0),gtsam::Rot3::Expmap(rot_r_c), RB, tB, tA, rot_noise);
            graph.add(tl_factor);

        }
        uint32_t i_base = (robot_.size()-1)*3-1;
        
        for(uint i = 0; i< robot_.size() - 1; i++)
        {
            gtsam::Pose3 B1 = robot_[i];
            gtsam::Pose3 B2 = robot_[i+1];
            gtsam::Pose3 Bi = B1.inverse()* B2;
            gtsam::Vector3 tB = Bi.translation();
            gtsam::Rot3  RB = Bi.rotation();
            // std::cout << "tB: " << tB << " tB norm: " << tB.norm() << std::endl;
            gtsam::Pose3 A1 = sensor_[i];
            gtsam::Pose3 A2 = sensor_[i+1];
            gtsam::Pose3 Ai = A1* A2.inverse();
            gtsam::Vector3 tA = Ai.translation();
            gtsam::Rot3 RA = Ai.rotation();
            // std::cout << "tA: " << tA << " tA norm: " << tA.norm() << std::endl;
            // (RB - I3)* X = rot_r_c* tA - tB

            TlFactor tl_factor(T(0),gtsam::Rot3::Expmap(rot_r_c), RB, tB, tA, rot_noise);
            graph.add(tl_factor);

        }

        gtsam::Vector3 tl_initial_value(0,0,0);
        initial_value.insert(T(0), translation);
        
        std::cout << "###################### init optimizer ######################" << std::endl;
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

        std::cout << "###################### begin optimize ######################" << std::endl;
        gtsam::Values result = optimizer.optimize();

        gtsam::Vector3 optbTo = result.at<Vector3>(T(0));
        std::cout << "Nonliear Vector3: " << optbTo << std::endl;

        return true;
    }
    
    bool HandEyeCalib::CalibrateGtsam(gtsam::Vector3& rot_r_c, gtsam::Vector3& translation)
    {
        gtsam::LevenbergMarquardtParams parameters;
        parameters.absoluteErrorTol = 1e-8;
        parameters.relativeErrorTol = 1e-8;
        parameters.maxIterations = 500;
        parameters.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
        parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_value;

        // auto rot_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
        auto rot_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.2));

        std::vector<gtsam::Rot3> rot_rr, rot_cc;

        for(uint i = 0; i< robot_.size() - 1; i++)
        {
            gtsam::Pose3 B1 = robot_[0];
            gtsam::Pose3 B2 = robot_[i+1];
            gtsam::Pose3 Bi = B1.inverse()* B2;
            gtsam::Vector3 tB = Bi.translation(); // t_r1_r2
            gtsam::Rot3  RB = Bi.rotation(); // rot_r1_r2
            // std::cout << "tB: " << tB << " tB norm: " << tB.norm() << std::endl;
            gtsam::Pose3 A1 = sensor_[0];
            gtsam::Pose3 A2 = sensor_[i+1];
            gtsam::Pose3 Ai = A1* A2.inverse();
            gtsam::Vector3 tA = Ai.translation(); //t_c1_c2
            gtsam::Rot3 RA = Ai.rotation(); // rot_c1_c2
            // std::cout << "tA: " << tA << " tA norm: " << tA.norm() << std::endl;
            // (RB - I3)* X = rot_r_c* tA - tB

            HandeyeFactor he_factor(R(0), T(0), RB, tB, tA, rot_noise);
            graph.add(he_factor);

        }
        uint32_t i_base = (robot_.size()-1)*3-1;
        
        for(uint i = 0; i< robot_.size() - 1; i++)
        {
            gtsam::Pose3 B1 = robot_[i];
            gtsam::Pose3 B2 = robot_[i+1];
            gtsam::Pose3 Bi = B1.inverse()* B2;
            gtsam::Vector3 tB = Bi.translation();
            gtsam::Rot3  RB = Bi.rotation();
            // std::cout << "tB: " << tB << " tB norm: " << tB.norm() << std::endl;
            gtsam::Pose3 A1 = sensor_[i];
            gtsam::Pose3 A2 = sensor_[i+1];
            gtsam::Pose3 Ai = A1* A2.inverse();
            gtsam::Vector3 tA = Ai.translation();
            gtsam::Rot3 RA = Ai.rotation();
            // std::cout << "tA: " << tA << " tA norm: " << tA.norm() << std::endl;
            // (RB - I3)* X = rot_r_c* tA - tB

            HandeyeFactor he_factor(R(0), T(0), RB, tB, tA, rot_noise);
            graph.add(he_factor);

        }

        gtsam::Vector3 tl_initial_value(-100,0,-30);
        initial_value.insert(T(0), translation);
        initial_value.insert(R(0), gtsam::Rot3::Expmap(rot_r_c));

        std::cout << "###################### init optimizer ######################" << std::endl;
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

        std::cout << "###################### begin optimize ######################" << std::endl;
        gtsam::Values result = optimizer.optimize();
        optimizer.print();

        gtsam::Vector3 optbTo = result.at<Vector3>(T(0));
        gtsam::Rot3 optrot = result.at<gtsam::Rot3>(R(0));
        std::cout << "Nonliear Vector3: " << optbTo << std::endl;
        std::cout << "Nonliear Rot3: " << optrot << std::endl;

        
        gtsam::Marginals marginals(graph, result);

        std::cout << "rot cov: " << marginals.marginalCovariance(R(0)) << std::endl;
        std::cout << "t cov: " << marginals.marginalCovariance(T(0)) << std::endl;
        
        gtsam::Pose3 rTc(optrot, optbTo);
        std::cout << "rTc: " << gtsam::Pose3::Logmap(rTc) << std::endl;
        return true;
    }

    bool HandEyeCalib::CalibrationTranslation(gtsam::Vector3& t)
    {
        return true;
    }
} // namespace calibration  

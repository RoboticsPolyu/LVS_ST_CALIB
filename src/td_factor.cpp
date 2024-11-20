#include "td_factor.h"

namespace calibration
{
    using namespace std;
    
    void PluckerLine::plus()
    {

    }

    float PluckerLine::Distance(gtsam::Vector3& point, gtsam::Matrix13& D_p, gtsam::Matrix14& D_l)
    {
        float error;

        // // std::cout << "point_: " << std::endl;
        // std::cout << point_.matrix() << std::endl;
        
        // gtsam::Vector6 Lw = ToL();
        // std::cout << "Lw: " << Lw << std::endl;

        gtsam::Vector3 err;
        
        // float w1 = cos(rou_);
        // float w2 = sin(rou_);

        gtsam::Matrix33 U = rot_.matrix();
        // gtsam::Vector3 u1 = U.col(0);
        // gtsam::Vector3 u2 = U.col(1);
        gtsam::Vector3 ld = U.col(0);
        gtsam::Vector3 lm = U.col(1);
        float m = m_;
        // gtsam::Vector3 n = w1* u1;
        // gtsam::Vector3 v = w2* u2;

        // err = n - point.cross(v); // ???
        // err = err / v.norm(); // Vd
        // float k = 1/ v.norm();
        // error = error*k;
        // 

        err = point.cross(ld) - m* lm;

        // gtsam::Matrix36 point_block;
        // point_block.block(0, 0, 3, 3) = - gtsam::Matrix33::Identity();
        // point_block.block(0, 3, 3, 3) = gtsam::skewSymmetric(point);
        
        // gtsam::Matrix6 D_nv = gtsam::Matrix6::Zero();
        // D_nv.block(0, 0, 3, 3) = gtsam::Matrix3::Identity()/ v.norm();;
        // D_nv.block(3, 3, 3, 3) = (gtsam::Matrix3::Identity() - v/ v.norm()* v.transpose()/ v.norm())/ v.norm();
        // D_nv.block(0, 3, 3, 3) = - n* v.transpose() /(v.norm()* v.norm()* v.norm());

        // gtsam::Matrix64 D_d_l = gtsam::Matrix64::Zero();
        // D_d_l.block(0, 0, 3, 3) = - gtsam::skewSymmetric(w1* u1);
        // D_d_l.block(3, 0, 3, 3) = - gtsam::skewSymmetric(w2* u2);
        // D_d_l.block(0, 3, 3, 1) = - w2* u1;
        // D_d_l.block(3, 3, 3, 1) =   w1* u2;      
        
        gtsam::Matrix34 D_v_l;
        D_v_l.block(0, 0, 3, 3) = - gtsam::skewSymmetric(point)* gtsam::skewSymmetric(rot_* gtsam::Vector3(1,0,0)) + m* gtsam::skewSymmetric(rot_* gtsam::Vector3(0,1,0));
        D_v_l.block(0, 3, 3, 1) = - gtsam::Vector3(rot_* gtsam::Vector3(0,1,0));
        
        D_p = err.transpose()/ err.norm()* gtsam::skewSymmetric(-ld)/ ld.norm(); 
        // D_l = err.transpose()/ err.norm()* point_block* D_nv* D_d_l;
        D_l = err.transpose()/ err.norm()* D_v_l;

        error = err.norm();
        
        // std::cout << "----------- err ----------------- \n" << err << std::endl;  
        // std::cout << "n: \n" << n << " ,v: \n" << v << std::endl;
        // std::cout <<  "point: \n" << point << " ,v.norm: \n" << v.norm() << std::endl; 
        // std::cout << "D_v_l: \n" << D_v_l << std::endl;
        // std::cout << "err: \n" << err << std::endl;
        // std::cout << "D_l: \n" << D_l.matrix() << std::endl;
        // std::cout << "D_p: \n" << D_p.matrix() << std::endl;

        // std::cout << "point_block: \n" << point_block << std::endl;
        // std::cout << "D_nv: \n" << D_nv << std::endl;
        // std::cout << "D_d_l: \n" << D_d_l << std::endl;

        return error;
    }

    gtsam::Vector3 PluckerLine::Distance(gtsam::Vector3& point, gtsam::Matrix3& D_p, gtsam::Matrix34& D_l)
    {

        // // std::cout << "point_: " << std::endl;
        // std::cout << point_.matrix() << std::endl;
        
        // gtsam::Vector6 Lw = ToL();
        // std::cout << "Lw: " << Lw << std::endl;

        gtsam::Vector3 err;
        
        // float w1 = cos(rou_);
        // float w2 = sin(rou_);

        gtsam::Matrix33 U = rot_.matrix();
        // gtsam::Vector3 u1 = U.col(0);
        // gtsam::Vector3 u2 = U.col(1);
        gtsam::Vector3 ld = U.col(0);
        gtsam::Vector3 lm = U.col(1);
        float m = m_;
        // gtsam::Vector3 n = w1* u1;
        // gtsam::Vector3 v = w2* u2;

        // err = n - point.cross(v); // ???
        // err = err / v.norm(); // Vd
        // float k = 1/ v.norm();
        // error = error*k;
        // 

        err = point.cross(ld) - m* lm;

        // gtsam::Matrix36 point_block;
        // point_block.block(0, 0, 3, 3) = - gtsam::Matrix33::Identity();
        // point_block.block(0, 3, 3, 3) = gtsam::skewSymmetric(point);
        
        // gtsam::Matrix6 D_nv = gtsam::Matrix6::Zero();
        // D_nv.block(0, 0, 3, 3) = gtsam::Matrix3::Identity()/ v.norm();;
        // D_nv.block(3, 3, 3, 3) = (gtsam::Matrix3::Identity() - v/ v.norm()* v.transpose()/ v.norm())/ v.norm();
        // D_nv.block(0, 3, 3, 3) = - n* v.transpose() /(v.norm()* v.norm()* v.norm());

        // gtsam::Matrix64 D_d_l = gtsam::Matrix64::Zero();
        // D_d_l.block(0, 0, 3, 3) = - gtsam::skewSymmetric(w1* u1);
        // D_d_l.block(3, 0, 3, 3) = - gtsam::skewSymmetric(w2* u2);
        // D_d_l.block(0, 3, 3, 1) = - w2* u1;
        // D_d_l.block(3, 3, 3, 1) =   w1* u2;      
        
        gtsam::Matrix34 D_v_l;
        D_v_l.block(0, 0, 3, 3) = - gtsam::skewSymmetric(point)* gtsam::skewSymmetric(rot_* gtsam::Vector3(1,0,0)) + m* gtsam::skewSymmetric(rot_* gtsam::Vector3(0,1,0));
        D_v_l.block(0, 3, 3, 1) = - gtsam::Vector3(rot_* gtsam::Vector3(0,1,0));
        
        D_p = gtsam::skewSymmetric(-ld)/ ld.norm(); 
        // D_l = err.transpose()/ err.norm()* point_block* D_nv* D_d_l;
        D_l = D_v_l;

        
        // std::cout << "----------- err ----------------- \n" << err << std::endl;  
        // std::cout << "n: \n" << n << " ,v: \n" << v << std::endl;
        // std::cout <<  "point: \n" << point << " ,v.norm: \n" << v.norm() << std::endl; 
        // std::cout << "D_v_l: \n" << D_v_l << std::endl;
        // std::cout << "err: \n" << err << std::endl;
        // std::cout << "D_l: \n" << D_l.matrix() << std::endl;
        // std::cout << "D_p: \n" << D_p.matrix() << std::endl;

        // std::cout << "point_block: \n" << point_block << std::endl;
        // std::cout << "D_nv: \n" << D_nv << std::endl;
        // std::cout << "D_d_l: \n" << D_d_l << std::endl;

        return err;
    }
    gtsam::Vector6 PluckerLine::ToL()
    {
        // float w1 = cos(rou_);
        // float w2 = sin(rou_);

        // gtsam::Matrix33 U = rot_.matrix();
        // // std::cout << "U: " << U << std::endl;

        // gtsam::Vector3 u1 = U.col(0);
        // gtsam::Vector3 u2 = U.col(1);
        // // std::cout << U.matrix() << u1 << u2 << std::endl;
        // gtsam::Vector6 Lw;
        // Lw.head(3) = w1* u1;
        // Lw.tail(3) = w2* u2;

        // // std::cout << "Lw: " << Lw << std::endl;
        // return Lw;
    }

};
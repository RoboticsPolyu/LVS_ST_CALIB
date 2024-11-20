#include "td_factor.h"

int main()
{
    gtsam::Vector3 p1(4,0,0); 
    gtsam::Vector3 v(0.5*1.414/2, -0.5*1.414/2, 0); // unit vector v
    gtsam::Vector3 n = p1.cross(v); // moment n

    std::cout << "rel n:\n " << n << std::endl;
    std::cout << "rel v:\n " << v << std::endl;

    gtsam::Matrix33 rot3;
    rot3.col(0) = n/ n.norm();
    rot3.col(1) = v/ v.norm();
    rot3.col(2) = n.cross(v) / (n.cross(v).norm());

    gtsam::Vector2 rnv(n.norm(), v.norm());
    gtsam::Vector2 w12 = rnv/ rnv.norm();

    // std::cout << "w12: " << w12 << std::endl;
    float rou = 0;

    if(w12(1) > 0)
        rou = acos(w12(0));
    else
        rou = -acos(w12(0));
    
    //gtsam::Vector3 w1(0.1, 0.2, 0.3);
    // gtsam::SO3 wso3 = gtsam::SO3::Expmap(w1);
    gtsam::Rot3 wso3(rot3);

    calibration::PluckerLine pl(wso3, rou);

    gtsam::Vector3 p(0,0,0);
    gtsam::Matrix13 D1;
    gtsam::Matrix14 D2;
    //std::cout << "p*v - n" << p.cross(v) + n;
    pl.Distance(p, D1, D2);

    p = gtsam::Vector3(1,1,0);
    //std::cout << "p*v + n" << p.cross(v) + n;

    pl.Distance(p, D1, D2);

    p = gtsam::Vector3(2,2,0);
    //std::cout << "p*v + n" << p.cross(v) + n;
    pl.Distance(p, D1, D2);

    return 0;
}
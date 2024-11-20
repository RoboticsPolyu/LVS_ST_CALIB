#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "hand_eye_calibration.h"
#include "gtsam_wrapper.h"
#include "laser_camera_cal.h"
 
using namespace std;
using namespace Eigen;

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

void cal_stdev(std::vector<double>& resultSet, double& mean, double& stdev)
{
    double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
    mean =  sum / resultSet.size(); //均值

    double accum  = 0.0;
    std::for_each (std::begin(resultSet), std::end(resultSet), [&](const double d) {
        accum  += (d - mean)*(d-mean);
    });
    
    stdev = sqrt(accum/(resultSet.size()-1)); //方差
}

int main(int argc, char **argv) 
{
    YAML::Node config = YAML::LoadFile("../config/config.yaml");

    std::string trajectory_file = config["trajectory_file"].as<std::string>();
    std::string egm_file        = config["egm_path"].as<std::string>();
  
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;

    ifstream fin(trajectory_file);
    if(!fin) 
    {
      cout << "cannot find trajectory file at " << trajectory_file << endl;
      return 1;
    }

    int index= 0;
    while (!fin.eof()) 
    {
      double tx, ty, tz; // time, qx, qy, qz, qw;
      double theta_x, theta_y, theta_z;
      fin >> tx >> ty >> tz >> theta_x >> theta_y >> theta_z;

      std::cout << index << " " << tx << " " << ty << " " << tz << std::endl;
      index++;
      Eigen::Matrix3d matlab_rot;

      Quaterniond q = gtsam::Quaternion(gtsam::Rot3::Expmap(gtsam::Vector3(theta_x, theta_y, theta_z)).matrix());

      // Quaterniond q = Eigen::AngleAxisd(theta_z/180.0* M_PI, ::Eigen::Vector3d::UnitZ()) *Eigen::AngleAxisd(theta_y/180.0* M_PI, ::Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(theta_x/180.0* M_PI, ::Eigen::Vector3d::UnitX());
      // Quaterniond q = Eigen::AngleAxisd(theta_z, ::Eigen::Vector3d::UnitZ()) *Eigen::AngleAxisd(theta_y, ::Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(theta_x, ::Eigen::Vector3d::UnitX());
  
      // 用Eigen::Isometry表示欧氏变换矩阵
      Eigen::Isometry3d Twr = Eigen::Isometry3d::Identity( );  // 三维变换矩阵
      Twr.rotate(q); // Quaterniond(qw, qx, qy, qz) );  // 旋转部分赋值
      Twr.pretranslate(Vector3d(tx, ty, tz));// 设置平移向量

      std::cout << Twr.inverse().translation() << std::endl;
      poses.push_back(Twr);
      
    }
    std::cout << "pose size: " << poses.size() << std::endl;

    std::ifstream file;
    file.open(egm_file);
    if(!file.is_open())
    {
        std::cout << "Not open Egm file: " << std::endl;
        return 0;
    }

    std::string str;
    std::istringstream iss;
    double timestamp, x, y, z, ex, ey, ez, qw, qx, qy, qz;
    std::vector<gtsam::Pose3> sensors, robots;

    while(std::getline(file, str))
    {
      iss.clear();
      iss.str(str);
      iss >> timestamp >> x >> y >> z >> ex >> ey >> ez >> qw >> qx >> qy >> qz;
      Quaterniond q(qw, qx, qy, qz);
      gtsam::Pose3 robot(gtsam::Rot3(q.matrix()), Eigen::Vector3d(x, y, z));
      robots.push_back(robot);
    }
    file.close();

    for(uint idx = 0; idx < poses.size() - 1; idx++)
    {
      gtsam::Pose3 sensor(gtsam::Rot3(poses[idx].rotation()), poses[idx].translation());
      sensors.push_back(sensor);
    }

    std::cout << "sensors's size: " << sensors.size() << "robot size: " << robots.size() << std::endl;
                                    
    assert(robots.size() == sensors.size());
    gtsam::Vector6 extrinsic;

    float dx = -1, dy = 2, dz = -3;
    extrinsic <<-0.571813596271, 0.582504536057, -1.51798630798, -72.049116392 +dx, -74.4696765119+dy, -87.0965539537+dz;
    // extrinsic << -0.562436596, 0.616238938914, -1.48823465114, -70.3869737546, -76.0375654058, -87.4195790369;
    extrinsic << -0.573411, 0.573156, -1.48322, -73.9481, -75.7107, -93.6198;
    extrinsic << -0.5598042514, 0.582083563342, -1.46377790677, -73.979784963, -75.4260364391, -93.6139079339;
    extrinsic << -0.576000, 0.566958, -1.48899, -72.6375, -78.8219, -97.0048;

    gtsam::Pose3 eTc = gtsam::Pose3::Expmap(extrinsic);
    std::vector<double> xs,ys,zs;
    for(uint idx = 0; idx < robots.size(); idx++)
    {
      gtsam::Pose3 bTp = robots[idx]* eTc * sensors[idx];
      std::cout << "bTp translation: " << bTp.translation() << std::endl;
      xs.push_back(bTp.translation()[0]);
      ys.push_back(bTp.translation()[1]);
      zs.push_back(bTp.translation()[2]);

    }    
    
    std::cout << "stdev: " << std::endl;
    double mean,stdev;
    cal_stdev(xs, mean, stdev);
    std::cout << mean << " " << stdev << " " ;
    cal_stdev(ys, mean, stdev);
    std::cout << mean << " " << stdev << " " ;
    cal_stdev(zs, mean, stdev);
    std::cout << mean << " " << stdev << std::endl;

    gtsam::Vector3 rot(0, M_PI/4, 0), t(0, 0, 0);
    calibration::HandEyeCalib handeye_calib(robots, sensors);
    handeye_calib.CalibrationRotationSVD(rot);
    // handeye_calib.CalibrateRotationGtsam(rot);
    handeye_calib.CalibrateTranslationLS(rot, t);
    // handeye_calib.CalibrateTranslationGtsam(rot, t);
    handeye_calib.CalibrateGtsam(rot, t);
      

    // draw trajectory in pangolin
    // DrawTrajectory(poses);
    return 0;
}
 
 
/*******************************************************************************************/
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
  // create pangolin window and plot the trajectory
  // 1创建名称为“Main”的GUI窗口，1024×768
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  // 2启动深度测试
  glEnable(GL_DEPTH_TEST);
  //该功能会使得pangolin只会绘制朝向镜头的那一面像素点，避免容易混淆的透视关系出现，因此在任何3D可视化中都应该开启该功能
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
  // 3创建一个观察相机视图
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000), //摄像机的内参矩阵ProjectionMatrix Pangolin会自动根据内参矩阵完成对应的透视变换
    pangolin::ModelViewLookAt(poses[0].translation().x(), poses[0].translation().y(), poses[0].translation().z(), 
      poses[0].translation().x() + 0.1, poses[0].translation().y(), poses[0].translation().z(), 0.0, -1.0, 0.0) //摄像机初始时刻所处的位置 摄像机的视点位置（即摄像机的光轴朝向哪一个点）以及摄像机的本身哪一轴朝上
  );
  
 
   //4创建交互视图用于显示上一步摄像机所“拍摄”到的内容
  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)//前四个参数依次表示视图在视窗中的范围（下、上、左、右），可以采用相对坐标（0~1）以及绝对坐标（使用Attach对象）。 //
    .SetHandler(new pangolin::Handler3D(s_cam));
  //5 开始画图
  while (pangolin::ShouldQuit() == false) {
    // 5-1 清空颜色和深度缓存 否则视窗内会保留上一帧的图形
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  
 
   // 0 画出轨迹连线
    for (size_t i = 0; i < poses.size(); i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    //1-1设置每个位置相机视角三维轴   自己写计算过程
    
    for (size_t i = 0; i < poses.size(); i++) 
    {
      // 计算变换后的点
      Vector3d Ow = poses[i].translation();//获取平移矩阵
      Vector3d Xw = poses[i] * (0.01 * Vector3d(1, 0, 0));//单位坐标1的线段变换到指定位置
      Vector3d Yw = poses[i] * (0.01 * Vector3d(0, 1, 0));
      Vector3d Zw = poses[i] * (0.01 * Vector3d(0, 0, 1));
       
      //开始画图
      glBegin(GL_LINES);
      glLineWidth(2);
      glColor3f(1.0, 0.0, 0.0);//定义颜色
      glVertex3d(Ow[0], Ow[1], Ow[2]);   glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);   glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);   glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
 
  
    //1-2划出相机位置 使用库中的矩阵计算
    for (size_t i = 0; i < poses.size(); i++) 
    {     
          //1创建矩阵
      glPushMatrix();
          
          //2矩阵赋值，后续所有操作都乘上这个系数
      Eigen::Isometry3d  R =poses[i];
      std::vector<GLdouble> Twc = {
                              R(0, 0), R(1,0), R(2, 0), 0.,
                      R(0, 1), R(1, 1), R(2, 1), 0.,
                      R(0, 2), R(1, 2), R(2, 2), 0.,
                      R(0, 3), R(1, 3), R(2, 3), 1.};
      glMultMatrixd(Twc.data());//pangolin后续绘制中的所有坐标均需要乘以这个矩阵
  
      
  
      //具体画框
      glBegin(GL_LINES);//4-1
          glLineWidth(2); //4-2
      glColor3f(1.0, 0.0, 0.0);//4-3定义颜色
      glVertex3f(0,0,0);      glVertex3f(2,0,0);
          glColor3f(0.0, 1.0, 0.0);//4-3定义颜色
      glVertex3f(0,0,0);      glVertex3f(0,2,0);
          glColor3f(0.0, 0.0, 1.0);//4-3定义颜色
      glVertex3f(0,0,0);      glVertex3f(0,0,2);
      glEnd();
  
          //3更新矩阵
      glPopMatrix();
 
    }
 
    
 
    //2划出相机位置 
    for (size_t i = 0; i < poses.size(); i++) 
    { 
          //1创建矩阵
      glPushMatrix();
          
          //2矩阵赋值，后续所有操作都乘上这个系数
      Eigen::Isometry3d  R =poses[i];
      std::vector<GLdouble> Twc = {
                      R(0, 0), R(1,0), R(2, 0), 0.,
                      R(0, 1), R(1, 1), R(2, 1), 0.,
                      R(0, 2), R(1, 2), R(2, 2), 0.,
                      R(0, 3), R(1, 3), R(2, 3), 1.};
      glMultMatrixd(Twc.data());//pangolin后续绘制中的所有坐标均需要乘以这个矩阵
  
      //3绘制相机轮廓线
      const float w = 2;
      const float h = w * 0.75;
      const float z = w * 0.6;
  
      glLineWidth(2); //4-1
      glBegin(GL_LINES);//4-2
      glColor3f(0.0f,1.0f,1.0f);//4-3
      glVertex3f(0,0,0);      glVertex3f(w,h,z);
      glVertex3f(0,0,0);      glVertex3f(w,-h,z);
      glVertex3f(0,0,0);      glVertex3f(-w,-h,z);
      glVertex3f(0,0,0);      glVertex3f(-w,h,z);
      glVertex3f(w,h,z);      glVertex3f(w,-h,z);
      glVertex3f(-w,h,z);     glVertex3f(-w,-h,z);
      glVertex3f(-w,h,z);     glVertex3f(w,h,z);
      glVertex3f(-w,-h,z);    glVertex3f(w,-h,z);
          //5
      glEnd();
          //3更新矩阵
      glPopMatrix();
 
    }
 
    // 运行帧循环以推进窗口事件
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }
} // 清空颜色和深度缓存
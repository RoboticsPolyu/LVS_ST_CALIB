#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>

#include "gtsam_wrapper.h"
#include "hand_eye_calibration.h"

// 本例演示了如何画出一个预先存储的轨迹
 
using namespace std;
using namespace Eigen;

string matlab_tra_file = "/home/yangpeiwen/Documents/ypw/wcq_lab/build/egm.txt";

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);


int main(int argc, char **argv) 
{
   
  //1定义容器-保存轨迹数据
  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
  //2打开文件，读取轨迹数据
  ifstream fin(matlab_tra_file);
  if (!fin) {
    cout << "cannot find trajectory file at " << matlab_tra_file << endl;
    return 1;
  }

    double tx_center, ty_center, tz_center;

  int index= 0;
  //3将文件数据赋值给容器
  while (!fin.eof()) {
    double timestamp, x, y, z, e1, e2, e3, e4, e5, e6, timestamp_f; // timestamp_f: ms
    fin >>  timestamp >> e1 >> e2 >> e3 >> e4 >> e5 >> e6;
    gtsam::Vector6 egm_;
    egm_ << e1, e2, e3, e4, e5, e6;

    gtsam::Pose3 robot = gtsam::Pose3::Expmap(egm_);

    if(index == 0)
    {
        tx_center = robot.translation().x();
        ty_center = robot.translation().y();
        tz_center = robot.translation().z();
    }
    index++;
    if(index == 2500)
      break;

    gtsam::Vector6 extrinsic;
    extrinsic = gtsam::Pose3::Logmap( gtsam::Pose3(gtsam::Rot3::Expmap(gtsam::Vector3(0.0,0.0,0) ), gtsam::Vector3(10, 20, 30) ) );
  
    gtsam::Pose3 eTc_ = gtsam::Pose3::Expmap(extrinsic);

    gtsam::Pose3 bTc = robot* eTc_;

    // 用Eigen::Isometry表示欧氏变换矩阵
    Eigen::Isometry3d Twr = Eigen::Isometry3d::Identity( );  // 三维变换矩阵
    Twr.rotate(bTc.rotation().toQuaternion()); // Quaterniond(qw, qx, qy, qz) );  // 旋转部分赋值
    Twr.pretranslate(bTc.translation());// 设置平移向量

    poses.push_back(Twr);
    
  }
  std::cout << "pose size: " << poses.size() << std::endl;
  
  cout << "read total " << poses.size() << " pose entries" << endl;
  //4执行画图
  // draw trajectory in pangolin
  DrawTrajectory(poses);
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
  
 
//    // 0 画出轨迹连线
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
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));//单位坐标1的线段变换到指定位置
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
       
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
      glVertex3f(0,0,0);      glVertex3f(0.05,0,0);
          glColor3f(0.0, 1.0, 0.0);//4-3定义颜色
      glVertex3f(0,0,0);      glVertex3f(0,0.05,0);
          glColor3f(0.0, 0.0, 1.0);//4-3定义颜色
      glVertex3f(0,0,0);      glVertex3f(0,0,0.05);
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
      const float w = 0.02;
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
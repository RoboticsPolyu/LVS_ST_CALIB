#include <xyf_cal_lib.h>
//#include "light_cal.h"    

int main(void)
{
    calibration::cm_xyf::MyStruct cm_data;

    /* internal paramater calibration*/
    cm_data.fold_path = "/home/wiind/Desktop/calibration/test/one_image";
    cm_data.corner_rows = 8;
    cm_data.corner_cols = 6;
    cm_data.cornersize_rows = 35;
    cm_data.cornersize_cols = 35;
    int i = 0;
//for (size_t i = 0; i < 1; i++)
//{
  	calibration::cm_xyf cm_xyf_instance(cm_data);
	calibration::cm_xyf::output cameramat = cm_xyf_instance.getcmdata();

    vector<cv::Point3f> b_cbpoint;
    vector<vector<cv::Point2f>> one_image_point = cm_xyf_instance.GetImageCorners();
    vector<vector<cv::Point3f>> oneimage_cbpoint = cm_xyf_instance.GetBoardCorners();
    Eigen::Vector4f light_points;
    light_points << 0.967553,
                    0.252666, 
                    1372.42, 
                    822.211;
    std::vector<cv::Point2f> cross_pointsdata;
    cm_xyf_instance.ComputeImageCrossPoints(one_image_point[i], light_points , cross_pointsdata);

    std::vector<cv::Point2f> points_group;
    std::vector<cv::Point2f> useful_points;
    std::vector<Eigen::Vector3f> lt_3dpoint;
    cm_xyf_instance.SelectThreeNeigborPoints(one_image_point[i], oneimage_cbpoint[i], cross_pointsdata , points_group , useful_points , b_cbpoint);
    cm_xyf_instance.Get3dPoints(cameramat, oneimage_cbpoint, points_group, useful_points, lt_3dpoint, b_cbpoint);
    cout << "points_group" << points_group << endl;
    cout << "useful_points" << useful_points << endl;
    cout << "b_cbpoint" << b_cbpoint << endl;
    cout << "lt_3dpoint" << lt_3dpoint[0] << endl;
    //cout << "lt_3dpoint" << lt_3dpoint[i] << endl;
    /* */
    return 0;
//}
}
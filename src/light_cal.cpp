#include "light_cal.h"


namespace calibration
{
    lt_xyf::lt_output lt_xyf::getltdata(void)
    {  
    	//just for test
		// cm_xyf::MyStruct cm_data;
		// cm_xyf::output output_data;

		// cm_data.fold_path = "";
		// cm_data.corner_rows = 8;
		// cm_data.corner_cols = 6;
		// cm_data.corners_ize_cols = 35;
		// cm_data.corners_ize_rows = 35;

		// cm_xyf cm_instance(cm_data);
		// output_data = cm_instance.getcmdata();
		
		// //creat test points
		// Eigen::Vector3f a_1 (200, 200 ,0);
		// Eigen::Vector3f b_1 (250, 250 ,0);
		// Eigen::Vector3f c_1 (300, 300 ,0);
		// Eigen::Vector3f p_1 (280, 280 ,0);

		// Eigen::Vector3f a_2 (250, 150 ,0);
		// Eigen::Vector3f b_2 (300, 200 ,0);
		// Eigen::Vector3f c_2 (350, 250 ,0);
		// Eigen::Vector3f p_2 (320, 220 ,0);

		// Eigen::Vector3f B_1 (0, 70, 0);
		// Eigen::Vector3f B_2 (70, 35, 0);

		// //(u,v)to(X,Y)
		// float fx = output_data.cameraMatrix.at<float>(1, 1);
		// float fy = output_data.cameraMatrix.at<float>(2, 2);
		// float cx = output_data.cameraMatrix.at<float>(1, 3);
		// float cy = output_data.cameraMatrix.at<float>(2, 3);
		
		// Eigen::Matrix3f temp_mat;
		// temp_mat << 1/fx , 0 , -cx/fx ,
		// 			0 , 1/fy , -cy/fy ,
		// 			0 , 0 , 0 ;

		// //points in imaging coordinate
		// Eigen::Vector3f Xa_1 =  temp_mat*a_1;
		// Eigen::Vector3f Xb_1 =  temp_mat*b_1;
		// Eigen::Vector3f Xc_1 =  temp_mat*c_1;
		// Eigen::Vector3f Xp_1 =  temp_mat*p_1;

		// Eigen::Vector3f Xa_2 =  temp_mat*a_2;
		// Eigen::Vector3f Xb_2 =  temp_mat*b_2;
		// Eigen::Vector3f Xc_2 =  temp_mat*c_2;
		// Eigen::Vector3f Xp_2 =  temp_mat*p_2;

		// //length of lines
		// float len_chessborad = cm_data.corners_ize_cols;
		// float len_ap_1 = (Xa_1 - Xp_1).norm();
		// float len_bc_1 = (Xb_1 - Xc_1).norm();
		// float len_ac_1 = (Xa_1 - Xc_1).norm();
		// float len_bp_1 = (Xb_1 - Xp_1).norm();
		
		// float len_ap_2 = (Xa_2 - Xp_2).norm();
		// float len_bc_2 = (Xb_2 - Xc_2).norm();
		// float len_ac_2 = (Xa_2 - Xc_2).norm();
		// float len_bp_2 = (Xb_2 - Xp_2).norm();

		// //position of point p in chessboard coordinate
		// float len_Xbp_1 = (((2*len_ap_1*len_bc_1)/(len_ac_1*len_bp_1)-1)/1)*len_chessborad;
		// float len_Xbp_2 = (((2*len_ap_2*len_bc_2)/(len_ac_2*len_bp_2)-1)/1)*len_chessborad;
		// Eigen::Vector4f pw_1 (B_1[0], B_1[1]+len_Xbp_1, 0, 1);
		// Eigen::Vector4f pw_2 (B_2[0], B_2[1]+len_Xbp_1, 0, 1);
		
		// //create mat_cw
		// cv::Mat temp_mat_1;
		// cv::Mat temp_mat_2 = (cv::Mat_<double>(1, 4) << 0,0,0,1);
		// cv::hconcat(output_data.rvecsMat_, output_data.tvecsMat_, temp_mat_1);
		// cv::Mat M_cw_temp;
		// cv::vconcat(temp_mat_1, temp_mat_2, M_cw_temp);
		// Eigen::Matrix4f M_cw;
		// cv::cv2eigen(M_cw_temp, M_cw);

		// //position of point p in camera coordinate
		// Eigen::Vector4f temp_pc_1 = M_cw*pw_1;
		// Eigen::Vector4f temp_pc_2 = M_cw*pw_2;
		
		// Eigen::Vector3f pc_1 = temp_pc_1.head(3);
		// Eigen::Vector3f pc_2 = temp_pc_2.head(3);

		// //make it a straight line
		// //output point p in camera coordinate
		// lt_output ltdata_output;
        
        // ltdata_output.pc_1 = pc_1;
        // ltdata_output.pc_2 = pc_2;

        // return ltdata_output;
    }
}
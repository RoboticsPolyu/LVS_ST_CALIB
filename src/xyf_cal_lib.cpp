#include <xyf_cal_lib.h>

namespace calibration
{
    void cm_xyf::ComputeImageCrossPoints(vector<cv::Point2f> &one_image_point, Eigen::Vector4f light_points, std::vector<cv::Point2f>& cross_points)
    {
        int cols = cm_data_.corner_cols;
        int rows = cm_data_.corner_rows;
        
        std::cout << "cols: " << cols << " ,rows: " << rows << std::endl; 
        std::vector<cv::Point2f> group_points_;
        std::vector<cv::Vec4f> line;

        for (int c = 0; c < cols; c++)
        {
            for (int r = 0; r < rows; r++)
            {
                cv::Point2f temp_point = one_image_point[r+rows*c];
                std::cout << "temp_point" << temp_point << std::endl;
                group_points_.push_back(temp_point);
            }
            cv::Vec4f line_para;
            cv::fitLine(group_points_,line_para,cv::DIST_L2, 0, 1e-2, 1e-2);
            line.push_back(line_para);
            group_points_.clear();
        };
        //printf("corners",line);
        for (int c_1 = 0;c_1 < cols; c_1++)
        {
            cv::Vec4f line_data = line[c_1];
            float x_b = line_data[2];
            float y_b = line_data[3];
            float k_b = line_data[1]/line_data[0];
            float b_b = y_b - k_b * x_b;
//***get line linformation
//***caculate the cross point
            float x_l = light_points[2];
            float y_l = light_points[3];
            float k_l = light_points[1]/light_points[0];
            float b_l = y_l - k_l * x_l;           

            // TODO CHECK k_l - k_b == 0
            float x_c = (b_b - b_l)/(k_l - k_b);
            float y_c = x_c * k_b + b_b;
            cv::Point2f cross_point (x_c , y_c);
          
            cross_points.push_back(cross_point);
        }            
        //cout << "cross_points" << std::fixed << cross_points << endl;

    }

    void cm_xyf::SelectThreeNeigborPoints(vector<cv::Point2f> one_image_pxpoint, vector<cv::Point3f> one_image_cbpoint, std::vector<cv::Point2f>& cross_points, std::vector<cv::Point2f>& points_group, std::vector<cv::Point2f>& useful_crosspoints, vector<cv::Point3f>& b_cbpoint)
    {
        int cols = cm_data_.corner_cols;
        int rows = cm_data_.corner_rows;
        std::vector<cv::Point2f> cols_points_px;
        std::vector<cv::Point3f> cols_points_cb;       
        std::vector<cv::Vec4f> lines;
        //std::vector<cv::Point2f> useful_crosspoints;

        for (int c = 0; c < cols; c++)
        {
            for (int r = 0; r < rows; r++)
            {
                cv::Point2f temp_point_1 = one_image_pxpoint[r+rows*c];
                cv::Point3f temp_point_2 = one_image_cbpoint[r+rows*c];              
                cols_points_px.push_back(temp_point_1);//***select col points in (u,v)***//
                cols_points_cb.push_back(temp_point_2);//***select col points in (X,Y)***//             
            }
            
            cv::Point2f temp_crosspoint = cross_points[c];
            float test_x = temp_crosspoint.x;
            std::cout<< "cols_points_px[col].x" << cols_points_px[0] <<endl;
            std::cout<< "cols_points_cb[col].x" << cols_points_cb[0] <<endl;
           

            if(test_x < cols_points_px[cols-1].x && test_x > cols_points_px[1].x)
            {
                useful_crosspoints.push_back(temp_crosspoint);
            }
            
            for (int c = 2; c < rows; c++)
            {
            if (test_x < cols_points_px[c].x && test_x > cols_points_px[c - 1].x && test_x > cols_points_px[c - 2].x)
             {
              b_cbpoint.push_back(cols_points_cb[c-1]);
              //
              points_group.push_back(cols_points_px[c]);
              points_group.push_back(cols_points_px[c-1]);
              points_group.push_back(cols_points_px[c-2]);
             }
            
            }
            cols_points_cb.clear();
            cols_points_px.clear();
            
        };

    }

    void cm_xyf::Get3dPoints(struct output cm_output,std::vector<vector<cv::Point3f>> object_points_, std::vector<cv::Point2f>& points_group, std::vector<cv::Point2f>& useful_crosspoints, std::vector<Eigen::Vector3f>& cc_points, vector<cv::Point3f>& b_cbpoint)
    {
        int count = sizeof(useful_crosspoints);
        for (size_t i = 0; i < count; i++)
        {

        
        Eigen::Vector3f a_1 (points_group[i].x , points_group[i].y , 0);
	    Eigen::Vector3f b_1 (points_group[i+1].x , points_group[i+1].y , 0);
	    Eigen::Vector3f c_1 (points_group[i+2].x , points_group[i+2].y , 0);
	    Eigen::Vector3f p_1 (useful_crosspoints[i].x , useful_crosspoints[i].y , 0);
        Eigen::Vector3f B_1 (b_cbpoint[i].x, b_cbpoint[i].y, b_cbpoint[i].z);

        std:;cout << "cm_output.cam" << cm_output.cameraMatrix << endl;

        // float fx = cm_output.cameraMatrix.at<float>(1, 1);
		// float fy = cm_output.cameraMatrix.at<float>(2, 2);
		// float cx = cm_output.cameraMatrix.at<float>(1, 3);
		// float cy = cm_output.cameraMatrix.at<float>(2, 3);

        
        // std::cout << "fx" << cm_output.cameraMatrix.ptr<float>(0) << endl;
        
        Eigen::Matrix3f internal_matrix;
        cv::cv2eigen(cm_output.cameraMatrix, internal_matrix);

        Eigen::Matrix3f internal_inverse = internal_matrix.inverse();
        Eigen::Vector3f corner_1 = internal_inverse* a_1;

        //std::cout << " internal matrix eigen: " << internal_matrix ;
        //Eigen::Matrix3f temp_mat;
		// temp_mat << 1/fx , 0 , -cx/fx ,
		// 			0 , 1/fy , -cy/fy ,
		// 			0 , 0 , 0 ;
        // std::cout << "temp_mat" << temp_mat << endl;

        Eigen::Vector3f Xa_1 =  internal_inverse*a_1;
		Eigen::Vector3f Xb_1 =  internal_inverse*b_1;
		Eigen::Vector3f Xc_1 =  internal_inverse*c_1;
		Eigen::Vector3f Xp_1 =  internal_inverse*p_1;

        std::cout << "Xa_1" << Xa_1 <<endl;
        std::cout << "Xb_1" << Xb_1 <<endl;
        std::cout << "Xc_1" << Xc_1 <<endl;
        std::cout << "Xp_1" << Xp_1 <<endl;

        float len_chessborad = 35;
		float len_ap_1 = (Xa_1 - Xp_1).norm();
		float len_bc_1 = (Xb_1 - Xc_1).norm();
		float len_ac_1 = (Xa_1 - Xc_1).norm();
		float len_bp_1 = (Xb_1 - Xp_1).norm();

        std::cout << "len_xbp_1" << len_ap_1 <<endl;
        std::cout << "len_xbp_1" << len_bc_1 <<endl;
        std::cout << "len_xbp_1" << len_ac_1 <<endl;
        std::cout << "len_xbp_1" << len_bp_1 <<endl;

        
        float len_Xbp_1 = (((2*len_ap_1*len_bc_1)/(len_ac_1*len_bp_1)-1)/1)*len_chessborad;

        std::cout << "len_xbp_1" << len_Xbp_1 <<endl;

		Eigen::Vector4f pw_1 (B_1[0], B_1[1]+len_Xbp_1, 0, 1);

        std::cout << "pw_1" << pw_1 << endl;
   
        cv::Mat temp_mat_1;
		cv::Mat temp_mat_2 = (cv::Mat_<double>(1, 4) << 0,0,0,1);

        std::cout << "temp_mat_2" << temp_mat_2 << endl;

		cv::hconcat(cm_output.rvecsMat, cm_output.tvecsMat[0], temp_mat_1);
		cv::Mat M_cw_temp;
		cv::vconcat(temp_mat_1, temp_mat_2, M_cw_temp);
		Eigen::Matrix4f M_cw;
		cv::cv2eigen(M_cw_temp, M_cw);

		//position of point p in camera coordinate
		Eigen::Vector4f temp_pc_1 = M_cw*pw_1;

        std::cout << "temp_pc_1" << temp_pc_1 << endl;

		Eigen::Vector3f pc_1 = temp_pc_1.head(3);
        cc_points.push_back(pc_1);
        }
    }

    cm_xyf::output cm_xyf::getcmdata(void)
    {
        output cm_output;
        std::vector<cv::String> filenames; // notice here that we are using the Opencv's embedded "String" class

        cv::glob(cm_data_.fold_path, filenames); // new function that does the job ;-)
        std::vector<cv::Mat> src_img_vector;
        cv::Mat src_img;
        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* camera intrinsic mat */
        cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* camera distortion data k1,k2,p1,p2,k3 */
        cv::Mat map1; // = Mat(image.size(), CV_32FC1, Scalar(0));
        cv::Mat map2;// = Mat(image.size(), CV_32FC1, Scalar(0));
        cv::Mat cv_R = cv::Mat::eye(3, 3, CV_32F);
        cv::Size imagesize;

        if(filenames.size() == 0)
        {
            std::cout << "file names size: " << filenames.size() << std::endl;
            return cm_output;
        }

        for(size_t i = 0; i < filenames.size(); ++i)
        {
            std::cout<< "file name: " << filenames[i] << std::endl;
            src_img = cv::imread(filenames[i]);
            src_img_vector.push_back(src_img);

            if(!src_img.data)
                std::cerr << "Problem loading image!!!" << std::endl;

            // cv::imshow("temp",src);
            // cv::waitKey(0);
            // /* do whatever you want with your images here */
        
            imagesize = src_img.size();
            cv::Size corner_size = cv::Size(cm_data_.corner_rows, cm_data_.corner_cols);
            int patternfound = cv::findChessboardCorners(src_img, corner_size, corners_, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
            
            cout << "this pic size: " << corners_.size() << endl;
            // find4Quadcorners_ubpix(image, corners_, Size(5, 5));
            // corners_ubPix(image, corners_, Size(11, 11), Size(-1, -1),
            // TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));

            image_points_seq_.push_back(corners_);
            cv::drawChessboardCorners(src_img , corner_size , corners_ , patternfound);

            std::cout << "drawed_corner" << cm_data_.fold_path+"/drawed_corner/"+ std::to_string(i) << std::endl;
            cv::imshow("drawed corner" +std::to_string(i), src_img);

            cv::imwrite(cm_data_.fold_path+"/drawed_corner/"+std::to_string(i) + ".bmp", src_img);
            cv::Size square_size = cv::Size(cm_data_.cornersize_rows, cm_data_.cornersize_cols);

            vector<cv::Point3f> realPoint;
            for (int i = 0; i < cm_data_.corner_cols; i++) {
                for (int j = 0; j < cm_data_.corner_rows; j++) {
                    cv::Point3f tempPoint;
                    /*  */
                    tempPoint.x = i * square_size.width;
                    tempPoint.y = j * square_size.height;
                    tempPoint.z = 0;
                    realPoint.push_back(tempPoint);
                }
            }
            object_points_.push_back(realPoint);
            printf("#objectPoints %ld\n", sizeof(object_points_[0]));
            std::cout << object_points_[0] << endl;

            // printf("#image_points %ld\n", sizeof(image_points_seq_[0]));
            // cout << corners_ << endl;
        }

        //TODO: check save style
        //**save objectpointdata in .yml
        cv::FileStorage temp_file1("object_points.yml" ,cv::FileStorage::WRITE);
        temp_file1 << "image_points_seq_" << image_points_seq_;
        temp_file1 << "object_points" << object_points_;
        temp_file1.release();
        
        calibrateCamera(object_points_, image_points_seq_ , src_img.size() , cameraMatrix , distCoeffs , rvecsMat_ , tvecsMat_);

        cv::Mat Knew_ = cameraMatrix.clone();
        // Mat tmp = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imagesize, 1, imagesize, 0);

        initUndistortRectifyMap(cameraMatrix , distCoeffs , cv_R, Knew_, imagesize , CV_16SC2, map1 , map2);
        //cv::remap(image_original, imageCalibration , map1 , map2 , cv::INTER_LINEAR);
        
        int image_index = 0;
        //TODO: fix undistort image name based filename[i]
        for(auto it = src_img_vector.begin(); it != src_img_vector.end(); it++)
        {
            undistort(*it, image_undistort_ , cameraMatrix , distCoeffs);
            cv::imwrite(cm_data_.fold_path + "/undistort/" + std::to_string(image_index) + ".bmp", image_undistort_);
            cv::imshow("undistort" +std::to_string(image_index), image_undistort_);
            image_index++;
        }

        Rodrigues(rvecsMat_[0], res_mat_);

        cout << "tvecsMat_:\n" << tvecsMat_[0] << endl;
        cout << "rvecsMat_:\n" << rvecsMat_[0] << endl;
        cout << "#cameraMatrix:\n" << cameraMatrix << endl;
        cout << "#distCoeffs:\n" << distCoeffs << endl;
        cout << "#res_mat_:\n" << res_mat_ << endl;
        
        cm_output.tvecsMat = tvecsMat_;
        cm_output.rvecsMat = res_mat_;
        cm_output.cameraMatrix = cameraMatrix;
        cm_output.distCoeffs = distCoeffs;
        //cv::waitKey(0);
        return cm_output;
        
    }

}
#include <laser_camera_cal.h>

namespace calibration
{
    void LaserCameraCal::ComputeImageCrossPoints(vector<cv::Point2f> &one_image_point, Eigen::Vector4f light_points, std::vector<cv::Point2f>& cross_points)
    {
        int cols = cm_data_.corner_cols;
        int rows = cm_data_.corner_rows;
        
        std::vector<cv::Point2f> group_points_;
        std::vector<cv::Vec4f> line;

        for (int c = 0; c < cols; c++)
        {
            for (int r = 0; r < rows; r++)
            {
                cv::Point2f temp_point = one_image_point[r+rows*c];
                group_points_.push_back(temp_point);
            }
            cv::Vec4f line_para;
            cv::fitLine(group_points_, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
            line.push_back(line_para);
            group_points_.clear();
        };

        for (int c_1 = 0; c_1 < cols; c_1++)
        {
            cv::Vec4f line_data = line[c_1];
            float x_b = line_data[2];
            float y_b = line_data[3];
            float k_b = line_data[1]/line_data[0];
            float b_b = y_b - k_b * x_b;

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
    }

    void LaserCameraCal::SelectThreeNeigborPoints(vector<cv::Point2f> one_image_pxpoint, vector<cv::Point3f> one_image_cbpoint, 
        std::vector<cv::Point2f>& cross_points, std::vector<cv::Point2f>& points_group, std::vector<cv::Point2f>& useful_crosspoints, vector<cv::Point3f>& b_cbpoint)
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
           
            if(test_x < cols_points_px[cols-1].x && test_x > cols_points_px[1].x)
            {
                useful_crosspoints.push_back(temp_crosspoint);
            }
            
            for (int c = 2; c < rows; c++)
            {
                if (test_x < cols_points_px[c].x && test_x > cols_points_px[c - 1].x && test_x > cols_points_px[c - 2].x)
                {
                    b_cbpoint.push_back(cols_points_cb[c-1]);
                    points_group.push_back(cols_points_px[c]);
                    points_group.push_back(cols_points_px[c-1]);
                    points_group.push_back(cols_points_px[c-2]);
                }
            }

            cols_points_cb.clear();
            cols_points_px.clear();
        };

    }

    /**
     * index: image rot and translation's idx
     * cm_output: internal matrix, rot nad t matrix
     * 
     * b_cbpoint: calibration board point 
     */
    void LaserCameraCal::Get3dPoints(int index, struct output cm_output, std::vector<cv::Point2f>& points_group, 
        std::vector<cv::Point2f>& useful_crosspoints, std::vector<Eigen::Vector3f>& cc_points, vector<cv::Point3f>& b_cbpoint)
    {
        int count = useful_crosspoints.size();
        for (int i = 0; i < count; i++)
        {
            // Compute cross point's real position at board coordinate
            Eigen::Vector3f a_1(points_group[i].x , points_group[i].y , 0);
            Eigen::Vector3f b_1(points_group[i+1].x , points_group[i+1].y , 0);
            Eigen::Vector3f c_1(points_group[i+2].x , points_group[i+2].y , 0);
            
            std::cout << "a_1: " << a_1 << std::endl;
            std::cout << "b_1: " << b_1 << std::endl;
            std::cout << "c_1: " << c_1 << std::endl;

            Eigen::Vector3f p_1(useful_crosspoints[i].x , useful_crosspoints[i].y , 0);
            Eigen::Vector3f B_1(b_cbpoint[i].x, b_cbpoint[i].y, b_cbpoint[i].z);

            std::cout << "p_1: " << p_1 << std::endl;
            std::cout << "B_1: " << B_1 << std::endl;

            Eigen::Matrix3f internal_matrix;
            cv::cv2eigen(cm_output.cameraMatrix, internal_matrix);

            Eigen::Matrix3f internal_inverse = internal_matrix.inverse();
            Eigen::Vector3f corner_1 = internal_inverse* a_1;

            Eigen::Vector3f Xa_1 =  internal_inverse*a_1;
            Eigen::Vector3f Xb_1 =  internal_inverse*b_1;
            Eigen::Vector3f Xc_1 =  internal_inverse*c_1;
            Eigen::Vector3f Xp_1 =  internal_inverse*p_1;

            float len_chessborad = cm_data_.len_chessborad;

            float len_ap_1 = (Xa_1 - Xp_1).norm();
            float len_bc_1 = (Xb_1 - Xc_1).norm();
            float len_ac_1 = (Xa_1 - Xc_1).norm();
            float len_bp_1 = (Xb_1 - Xp_1).norm();
            
            float len_Xbp_1 = (((2.0*len_ap_1 * len_bc_1) / (len_ac_1 * len_bp_1) -1.0 ) /1.0 ) * len_chessborad;

            Eigen::Vector4f pw_1(B_1[0], B_1[1] + len_Xbp_1, 0, 1);

            std::cout << "len_Xbp_1: " << len_Xbp_1 << std::endl;
            std::cout << "pw_1: " << pw_1[0] << " " << pw_1[1] << std::endl;

            // Transform board point from board's coordinate to camera's coordinate
            cv::Mat temp_mat_1;
            cv::Mat temp_mat_2 = (cv::Mat_<double>(1, 4) << 0,0,0,1);

            cv::Mat res_mat;
            Rodrigues(cm_output.rvecsMat[index], res_mat);
            cv::hconcat(res_mat, cm_output.tvecsMat[index], temp_mat_1);
            cv::Mat M_cw_temp;
            cv::vconcat(temp_mat_1, temp_mat_2, M_cw_temp);
            Eigen::Matrix4f M_cw;
            cv::cv2eigen(M_cw_temp, M_cw);
            std::cout << "Debug !!!!!!!!!! M_cw !!!!!!!!!!!" << M_cw << std::endl;

            Eigen::Vector4f temp_pc_1 = M_cw*pw_1;

            Eigen::Vector3f pc_1 = temp_pc_1.head(3);
            cc_points.push_back(pc_1);
        }
    }

    LaserCameraCal::output LaserCameraCal::getcmdata(void)
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
            
            std::cout << "this image corner size: " << corners_.size() << endl;
            // cv::find4QuadCornerSubpix(src_img, corners_, cv::Size(5, 5));
            // cv::cornerSubPix(src_img, corners_, cv::Size(11, 11), cv::Size(-1, -1),
            // cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));

            image_corners_seq_.push_back(corners_);
            cv::drawChessboardCorners(src_img , corner_size , corners_ , patternfound);

            std::cout << "drawed_corner" << cm_data_.fold_path+"/drawed_corner/"+ std::to_string(i) << std::endl;
            cv::Mat src_img_dst;
            ResizeImage(src_img, src_img_dst);
            // cv::imshow("drawed corner" +std::to_string(i), src_img_dst);

            cv::imwrite(cm_data_.fold_path+"/drawed_corner/drawed_"+std::to_string(i) + ".bmp", src_img);
            cv::Size square_size = cv::Size(cm_data_.cornersize_rows, cm_data_.cornersize_cols);

            vector<cv::Point3f> realPoint;
            for (int i = 0; i < cm_data_.corner_cols; i++) {
                for (int j = 0; j < cm_data_.corner_rows; j++) {
                    cv::Point3f tempPoint;
                    tempPoint.x = i * square_size.width;
                    tempPoint.y = j * square_size.height;
                    tempPoint.z = 0;
                    realPoint.push_back(tempPoint);
                }
            }
            object_points_.push_back(realPoint);
            // printf("#objectPoints %ld\n", sizeof(object_points_[0]));
            // std::cout << object_points_[0] << endl;
            // printf("#image_points %ld\n", sizeof(image_corners_seq_[0]));
            // cout << corners_ << endl;
        }

        //TODO: check save style
        //**save objectpointdata in .yaml
        cv::FileStorage temp_file1("object_points.yaml" ,cv::FileStorage::WRITE);
        temp_file1 << "image_corners_seq_" << image_corners_seq_;
        temp_file1 << "object_points" << object_points_;
        temp_file1.release();
        
        cv::calibrateCamera(object_points_, image_corners_seq_ , src_img.size() , cameraMatrix , distCoeffs , rvecsMat_ , tvecsMat_);

        cv::Mat Knew_ = cameraMatrix.clone();
        // Mat tmp = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imagesize, 1, imagesize, 0);

        cv::initUndistortRectifyMap(cameraMatrix , distCoeffs , cv_R, Knew_, imagesize , CV_16SC2, map1 , map2);
        //cv::remap(image_original, imageCalibration , map1 , map2 , cv::INTER_LINEAR);
        
        int image_index = 0;
        //TODO: fix undistort image name based filename[i]
        for(auto it = src_img_vector.begin(); it != src_img_vector.end(); it++)
        {
            undistort(*it, image_undistort_ , cameraMatrix , distCoeffs);
            cv::Mat image_undistort_dst;
            ResizeImage(image_undistort_, image_undistort_dst);

            cv::imwrite(cm_data_.fold_path + "/undistort/undistort_" + std::to_string(image_index) + ".bmp", image_undistort_);
            // DetectLine(image_undistort_);

            image_undistorts_.push_back(image_undistort_);

            // cv::imshow("undistort" +std::to_string(image_index), image_undistort_dst);
            image_index++;
        }

        // Rodrigues(rvecsMat_[0], res_mat_);

        std::cout << "#cameraMatrix:\n" << cameraMatrix << endl;
        std::cout << "#distCoeffs:\n" << distCoeffs << endl;
        
        cm_output.tvecsMat = tvecsMat_;
        cm_output.rvecsMat = rvecsMat_;
        cm_output.cameraMatrix = cameraMatrix;
        cm_output.distCoeffs = distCoeffs;

        cv::cv2eigen(cameraMatrix, camera_matrix_);

        return cm_output;
    }

    void LaserCameraCal::ComputeLaserPoint(int idx, float u, float v, float& z_c, float& x_c, float& y_c)
    {
        // pattern board own normal vector
        Eigen::Vector3f pb_normal_identity(0, 0, 1);

        // compute pattern board normal vector at camera coordinate
        Eigen::Vector3f pb_normal_c;
        Eigen::Matrix3f rot;
        Eigen::Vector3f t; // px py pz
        cv::Mat res_mat;
        Rodrigues(rvecsMat_[idx], res_mat);
        cv::cv2eigen(res_mat, rot);
        cv::cv2eigen(tvecsMat_[idx], t);
        pb_normal_c = rot* pb_normal_identity + t; // ax ay az

        float ax = pb_normal_c(0);
        float ay = pb_normal_c(1);
        float az = pb_normal_c(2);

        // compute (u,v) ---> (x_c, y_c, z_c)
        float ap = pb_normal_c.dot(t);
        float u_u0_kx = (u - camera_matrix_(0,2))/ camera_matrix_(0,0);
        float v_v0_ky = (v - camera_matrix_(1,2))/ camera_matrix_(1,1);

        z_c = ap / (u_u0_kx* ax + v_v0_ky* ay + az);
        x_c = z_c* u_u0_kx;
        y_c = z_c* v_v0_ky;
    }

    void LaserCameraCal::DetectLine(cv::Mat imagesrc)
    {
        
        // line_finder.final(imagesrc);
        cv::Vec4f vec4f;
        LineFinder line_finder;

        line_finder.finish(imagesrc, vec4f);
        light_vecs_.push_back(vec4f);
        std::cout << "This image's light vec4f is : " << vec4f << std::endl;
        // cv::waitKey(0);
    }

    bool LaserCameraCal::DetectLine()
    {
        std::vector<cv::String> filenames; // notice here that we are using the Opencv's embedded "String" class
        cv::Mat src_img;

        cv::glob(cm_data_.line_image_path, filenames); // new function that does the job ;-)

        if(filenames.size() == 0)
        {
            std::cout << "file names size: " << filenames.size() << std::endl;
            return 1;
        }

        for(int i = 0; i < filenames.size(); ++i)
        {
            std::cout<< "Laser file name: " << filenames[i] << std::endl;
            src_img = cv::imread(filenames[i], IMREAD_GRAYSCALE);
            if(!src_img.data)
                std::cerr << "Problem loading image!!!" << std::endl;

            DetectLine(src_img);

        }
        cv::FileStorage temp_file1("detect_line.yaml" ,cv::FileStorage::WRITE);
        for(int i = 0; i < light_vecs_.size(); i++)
        {
            temp_file1 << "line_vec4f_" + std::to_string(i) << light_vecs_[i];
        }
        temp_file1.release();

        std::cout << "DetectLine last Line " << std::endl;
    }

    void LaserCameraCal::ResizeImage(cv::Mat& imagesrc, cv::Mat& imagedst)
	{
		double fScale = 0.2;//缩放系数
		//计算目标图像的大小
		cv::Size dsize = cv::Size(imagesrc.cols*fScale, imagesrc.rows*fScale);
		cv::resize(imagesrc, imagedst, dsize);
	}
}
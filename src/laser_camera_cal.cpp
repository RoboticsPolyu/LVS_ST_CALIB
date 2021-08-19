#include <laser_camera_cal.h>

namespace calibration
{
    bool LaserCameraCal::LoadParameter(const std::string& filename)
    {
        YAML::Node config = YAML::LoadFile(filename);
        int corner_rows = config["corner_rows"].as<int>();
        int corner_cols = config["corner_cols"].as<int>();
        float cornersize_rows = config["cornersize_rows"].as<float>();
        float cornersize_cols = config["cornersize_cols"].as<float>();
        std::string fold_path = config["fold_path"].as<std::string>();
        std::string line_image_path = config["line_image_path"].as<std::string>();
        std::string intrinsic_file = config["intrinsic_file"].as<std::string>();
        uint8_t camera_model = config["camera_model"].as<uint8_t>();

        parameter_.corner_rows = corner_rows;
        parameter_.corner_cols = corner_cols;
        parameter_.cornersize_cols = cornersize_cols;
        parameter_.cornersize_rows = cornersize_rows;
        parameter_.fold_path = fold_path;
        parameter_.line_image_path = line_image_path;
        parameter_.intrinsic_file = intrinsic_file;
        parameter_.camera_model = camera_model;

        parameter_.print();
        return true;
    }

    void LaserCameraCal::MultiImageCalibrate(void)
    {
        std::vector<cv::String> filenames;

        cv::glob(parameter_.fold_path, filenames); 
        std::vector<cv::Mat> src_imgs;
        cv::Mat src_img;
        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* camera intrinsic mat */
        cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* camera distortion data k1,k2,p1,p2,k3 */
        cv::Mat map1, map2;
        cv::Mat cv_R = cv::Mat::eye(3, 3, CV_32F);
        cv::Size image_size;
        cv::Mat image_undistort;
        vector<cv::Point2f> corners;
        
        if(filenames.size() == 0)
        {
            std::cout << "File names size: " << filenames.size() << std::endl;
            return;
        }

        for(size_t i = 0; i < filenames.size(); ++i)
        {
            std::cout<< "File name: " << filenames[i] << std::endl;
            src_img = cv::imread(filenames[i], IMREAD_GRAYSCALE);
            src_imgs.push_back(src_img);

            if(!src_img.data)
                std::cerr << "Problem loading image!!!" << std::endl;
        
            image_size = src_img.size();
            cv::Size corner_size = cv::Size(parameter_.corner_cols, parameter_.corner_rows);
            int patternfound = cv::findChessboardCorners(src_img, corner_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH);
            
            std::cout << "This image corner size: " << corners.size() << endl;

            float64_t tag_width_in_pixel = cv::norm(corners[0] - corners[1]);
            int32_t win_width = tag_width_in_pixel / 32.0 >= 2.0 ? (int32_t)(tag_width_in_pixel / 32.0) : 2;
            cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 40, 0.01);
            cv::Size win_size(win_width, win_width);
            cv::cornerSubPix(src_img, corners, win_size, cv::Size(-1, -1), criteria);

            // std::cout << "************************************************" <<std::endl;
            // std::cout << "corners: " << corners << std::endl;
            // std::cout << "************************************************" <<std::endl;

            image_corners_seq_.push_back(corners);
            cv::drawChessboardCorners(src_img , corner_size , corners , patternfound);
            std::cout << "Drawed_corner" << parameter_.fold_path+"/drawed_corner/"+ std::to_string(i) << std::endl;
            cv::imwrite(parameter_.fold_path+"/drawed_corner/drawed_"+std::to_string(i) + ".bmp", src_img);

            vector<cv::Point3f> realPoint;
            for (int i = 0; i < parameter_.corner_rows; i++) {
                for (int j = 0; j < parameter_.corner_cols; j++) {
                    cv::Point3f tempPoint;
                    tempPoint.x = i * parameter_.cornersize_rows;
                    tempPoint.y = j * parameter_.cornersize_cols;
                    tempPoint.z = 0;
                    realPoint.push_back(tempPoint);
                }
            }
            
            object_points_.push_back(realPoint);
        }

        image_size_ = image_corners_seq_.size();
        
        cv::calibrateCamera(object_points_, image_corners_seq_ , src_img.size() , cameraMatrix , distCoeffs , rvecsMat_ , tvecsMat_);

        cv::Mat Knew_ = cameraMatrix.clone();
        cv::initUndistortRectifyMap(cameraMatrix , distCoeffs , cv_R, Knew_, image_size , CV_16SC2, map1 , map2);
        
        int image_index = 0;
        for(auto it = src_imgs.begin(); it != src_imgs.end(); it++)
        {
            undistort(*it, image_undistort , cameraMatrix , distCoeffs);
            cv::imwrite(parameter_.fold_path + "/undistort/undistort_" + std::to_string(image_index) + ".bmp", image_undistort);
            image_index++;
        }

        std::cout << "#CameraMatrix:\n" << cameraMatrix << endl;
        std::cout << "#DistCoeffs:\n" << distCoeffs << endl;

        dist_coeffs_ = distCoeffs;

        cv::cv2eigen(cameraMatrix, camera_eigen_matrix_);

        std::vector<float> errors;
        float64_t rms = ComputeReprojectionErrors(object_points_, image_corners_seq_, rvecsMat_, tvecsMat_, cameraMatrix, distCoeffs, errors,false);
        std::cout << "#Calibration RMS: " << rms << std::endl;
        return;
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
        pb_normal_c = rot* pb_normal_identity;//  + t; // ax ay az

        float ax = pb_normal_c(0);
        float ay = pb_normal_c(1);
        float az = pb_normal_c(2);

        // compute (u,v) ---> (x_c, y_c, z_c)
        float ap = pb_normal_c.dot(t);
        float u_u0_kx = (u - camera_eigen_matrix_(0,2))/ camera_eigen_matrix_(0,0);
        float v_v0_ky = (v - camera_eigen_matrix_(1,2))/ camera_eigen_matrix_(1,1);

        z_c = ap / (u_u0_kx* ax + v_v0_ky* ay + az);
        x_c = z_c* u_u0_kx;
        y_c = z_c* v_v0_ky;
    }
    
    void LaserCameraCal::UndistortPoints(const std::vector<cv::Point2f>& in, std::vector<cv::Point2f> &out) const
    {
        cv::Mat camera_matrix;
        cv::eigen2cv(camera_eigen_matrix_, camera_matrix);

        if (parameter_.camera_model == CAMERA_MODEL::FISHEYE)
        {
            cv::fisheye::undistortPoints(in, out, camera_matrix, dist_coeffs_, cv::Matx33d::eye(), camera_matrix);
        }
        else
        {
            cv::undistortPoints(in, out, camera_matrix, dist_coeffs_, cv::Matx33d::eye(), camera_matrix);
        }
    }

    void LaserCameraCal::DetectLine(cv::Mat imagesrc)
    {
        
        // line_finder.final(imagesrc);
        cv::Vec4f vec4f;
        LineFinder line_finder;

        line_finder.finish(imagesrc, vec4f);
        light_vecs_.push_back(vec4f);
        std::cout << "This image's light vec4f is : " << vec4f << std::endl;
    }

    bool LaserCameraCal::DetectLine()
    {
        std::vector<cv::String> filenames; // notice here that we are using the Opencv's embedded "String" class
        cv::Mat src_img;
        cv::Mat image_undistort;

        cv::glob(parameter_.line_image_path, filenames); // new function that does the job ;-)

        if(filenames.size() == 0)
        {
            std::cout << "file names size: " << filenames.size() << std::endl;
            return 1;
        }

        for(uint32_t i = 0; i < filenames.size(); ++i)
        {
            std::cout<< "Laser file name: " << filenames[i] << std::endl;
            src_img = cv::imread(filenames[i], IMREAD_GRAYSCALE);
            if(!src_img.data)
                std::cerr << "Problem loading image!!!" << std::endl;
            cv::Mat camera_matrix;
            cv::eigen2cv(camera_eigen_matrix_, camera_matrix);
            undistort(src_img, image_undistort , camera_matrix , dist_coeffs_);

            DetectLine(image_undistort);
        }
        cv::FileStorage temp_file1("detect_line.yaml" ,cv::FileStorage::WRITE);
        for(uint32_t i = 0; i < light_vecs_.size(); i++)
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

    void LaserCameraCal::ComputeLaserPoint(float p_a, float p_b, float p_c, float u, float v, float x_c, float y_c, float z_c)
    {
        float u0 = camera_eigen_matrix_(0,2);
        float v0 = camera_eigen_matrix_(1,2);
        float kx = camera_eigen_matrix_(0,0);
        float ky = camera_eigen_matrix_(1,1);

        z_c = p_c* kx* ky /(kx* ky + p_a* ky* (u0- u) + p_b* kx* (v0 - v));
        x_c = z_c* (u - u0)/ kx;
        y_c = z_c* (v - v0)/ ky;
    }

    bool LaserCameraCal::LoadCameraMatrix(Eigen::Matrix3f& camera_matrix)
    {
        YAML::Node config = YAML::LoadFile(parameter_.intrinsic_file);
        float fx = config["fx"].as<float>();
        float fy = config["fy"].as<float>();
        float u0 = config["u0"].as<float>();
        float v0 = config["v0"].as<float>();
        
        camera_matrix(0, 0) = fx;
        camera_matrix(1, 1) = fy;
        camera_matrix(0, 2) = u0;
        camera_matrix(1, 2) = v0;

        std::cout << "*Load Camera Matrix: " << std::endl;
        std::cout << camera_matrix << std::endl;
    }

    bool LaserCameraCal::SaveCameraMatrix(const Eigen::Matrix3f& camera_matrix)
    {
    }
 
    double LaserCameraCal::ComputeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                                const vector<vector<Point2f> >& imagePoints,
                                                const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                                const Mat& cameraMatrix , const Mat& distCoeffs,
                                                vector<float>& perViewErrors, bool fisheye)
    {
        vector<Point2f> imagePoints2;
        size_t totalPoints = 0;
        double totalErr = 0, err;
        perViewErrors.resize(objectPoints.size());
        for(size_t i = 0; i < objectPoints.size(); ++i )
        {
            if (fisheye)
            {
                fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                distCoeffs);
            }
            else
            {
                projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
            }
            err = norm(imagePoints[i], imagePoints2, NORM_L2);
            size_t n = objectPoints[i].size();
            perViewErrors[i] = (float) std::sqrt(err*err/n);
            totalErr += err*err;
            totalPoints += n;
        }
        return std::sqrt(totalErr/totalPoints);
    }
} 
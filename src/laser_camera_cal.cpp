#include <laser_camera_cal.h>
#include "hand_eye_calibration.h"
#include <sstream>

namespace calibration
{
    bool LaserCameraCalib::LoadParameter(const std::string& filename)
    {
        YAML::Node config = YAML::LoadFile(filename);
        int corner_rows = config["corner_rows"].as<int>();
        int corner_cols = config["corner_cols"].as<int>();
        float cornersize_rows = config["cornersize_rows"].as<float>();
        float cornersize_cols = config["cornersize_cols"].as<float>();
        std::string fold_path = config["fold_path"].as<std::string>();
        std::string line_image_path = config["line_image_path"].as<std::string>();
        std::string egm_data_path = config["egm_path"].as<std::string>();
        uint8_t camera_model = config["camera_model"].as<uint8_t>();
        bool use_intrinsic = config["use_intrinsic"].as<bool>();

        parameter_.corner_rows = corner_rows;
        parameter_.corner_cols = corner_cols;
        parameter_.cornersize_cols = cornersize_cols;
        parameter_.cornersize_rows = cornersize_rows;
        parameter_.fold_path = fold_path;
        parameter_.line_image_path = line_image_path;
        parameter_.egm_path = egm_data_path;
        parameter_.camera_model = camera_model;
        parameter_.use_intrinsic = use_intrinsic;
        parameter_.print();

        float fx = config["fx"].as<float>();
        float fy = config["fy"].as<float>();
        float u0 = config["u0"].as<float>();
        float v0 = config["v0"].as<float>();
        
        camera_eigen_matrix_(0, 0) = fx;
        camera_eigen_matrix_(1, 1) = fy;
        camera_eigen_matrix_(0, 2) = u0;
        camera_eigen_matrix_(1, 2) = v0;
        camera_eigen_matrix_(2, 2) = 1;

        k1_ = config["k1"].as<float>();
        k2_ = config["k2"].as<float>();
        k3_ = config["k3"].as<float>();
        p1_ = config["p1"].as<float>();
        p2_ = config["p2"].as<float>();

        std::cout << "********************************************" << std::endl;
        std::cout << " -- Load Camera Matrix: " << std::endl;
        std::cout << camera_eigen_matrix_ << std::endl;
        std::cout << " -- Load Distortion Matrix: " << std::endl;
        std::cout <<  "k1: " << k1_ << std::endl;
        std::cout <<  "k2: " << k2_ << std::endl;
        std::cout <<  "k3: " << k3_ << std::endl;
        std::cout <<  "p1: " << p1_ << std::endl;
        std::cout <<  "p2: " << p2_ << std::endl;
        std::cout << "********************************************" << std::endl;

        return true;
    }

    void LaserCameraCalib::Calibrate(void)
    {
        std::ofstream pattern_pose_file;
        pattern_pose_file.open("pattern_pose_file.txt", std::ios_base::trunc);

        std::ofstream pattern_cam_file;
        pattern_cam_file.open("pattern_camera_file.txt", std::ios_base::trunc);

        std::vector<cv::String> filenames;

        cv::glob(parameter_.fold_path, filenames); 
        std::vector<cv::Mat> src_imgs;
        cv::Mat src_img;
        cv::Mat cv_K  = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* camera intrinsic mat */
        cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* camera distortion data k1,k2,p1,p2,k3 */
        cv::Mat map1, map2;
        cv::Mat cv_R = cv::Mat::eye(3, 3, CV_32F);
        cv::Size image_size;
        cv::Mat image_undistort;
        std::vector<cv::Point2f> corners;
        
        if(filenames.size() == 0)
        {  
            std::cout << "File names size: " << filenames.size() << std::endl;
            return;
        }

        for(size_t i = 0; i < filenames.size(); ++i)
        {
            std::cout << "-----------------------------------------" << std::endl;
            std::cout<< "File name: " << filenames[i] << std::endl;
            src_img = cv::imread(filenames[i], cv::IMREAD_GRAYSCALE);
            src_imgs.push_back(src_img);

            if(!src_img.data)
                std::cerr << "Problem loading image!!!" << std::endl;
        
            image_size = src_img.size();
            cv::Size corner_size = cv::Size(parameter_.corner_cols, parameter_.corner_rows);
            int patternfound = cv::findChessboardCorners(src_img, corner_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH);
            
            std::cout << "This image corner size: " << corners.size() << std::endl;

            float64_t tag_width_in_pixel = cv::norm(corners[0] - corners[1]);
            int32_t win_width = tag_width_in_pixel / 32.0 >= 2.0 ? (int32_t)(tag_width_in_pixel / 32.0) : 2;
            cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 40, 0.01);
            cv::Size win_size(win_width, win_width);
            cv::cornerSubPix(src_img, corners, win_size, cv::Size(-1, -1), criteria);

            // std::cout << "************************************************" <<std::endl;
            // std::cout << "corners: " << corners << std::endl;
            // std::cout << "************************************************" <<std::endl;

            image_corners_seq_.push_back(corners);
            cv::drawChessboardCorners(src_img, corner_size, corners, patternfound);
            std::cout << "Drawed_corner： " << parameter_.fold_path+"/drawed_corner/"+ std::to_string(i) << std::endl;
            cv::imwrite(parameter_.fold_path+"/drawed_corner/drawed_"+std::to_string(i) + ".bmp", src_img);

            std::vector<cv::Point3f> realPoint;
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
        image_corners_undistorted_.clear();

        if(!parameter_.use_intrinsic)
        {
            cv::calibrateCamera(object_points_, image_corners_seq_ , src_img.size() , cv_K , distCoeffs , rvecs_mat_ , tvecs_mat_, cv::CALIB_ZERO_TANGENT_DIST);
        }
        else
        {        
            cv::eigen2cv(camera_eigen_matrix_, cv_K);
            distCoeffs = (cv::Mat_<float>(1, 5) << k1_, k2_, p1_, p2_, k3_);

            for(size_t idx = 0; idx < filenames.size(); ++idx)
            {
                std::vector<cv::Point2f> undistort_points;
                std::vector<int32_t> inliers;
                cv::Mat cv_rvec(3, 1, CV_64F), cv_tvec(3, 1, CV_64F);
                cv::undistortPoints(image_corners_seq_[idx], undistort_points, cv_K, distCoeffs, cv::Matx33d::eye(), cv_K);
                cv::solvePnPRansac(object_points_[idx], undistort_points, cv_K, cv::noArray(), cv_rvec, cv_tvec, false, 100, 8, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
                image_corners_undistorted_.push_back(undistort_points);

                std::cout << cv_rvec << std::endl;
                std::cout << cv_tvec << std::endl;
                tvecs_mat_.push_back(cv_tvec);
                rvecs_mat_.push_back(cv_rvec);
            }
        }
        
        int image_index = 0;
        
        for(auto it = src_imgs.begin(); it != src_imgs.end(); it++)
        {
            cv::undistort(*it, image_undistort , cv_K , distCoeffs);
            cv::imwrite(parameter_.fold_path + "/undistort/undistort_" + std::to_string(image_index) + ".bmp", image_undistort);
            image_index++;
        }

        std::cout << "#CameraMatrix:\n" << cv_K << std::endl;
        std::cout << "#DistCoeffs:\n" << distCoeffs << std::endl;

        dist_coeffs_ = distCoeffs;

        cv::cv2eigen(cv_K, camera_eigen_matrix_);

        pattern_pose_file.precision(5);
        for(int idx = 0; idx < rvecs_mat_.size(); idx++)
        {
            cv::Mat res_mat;
            Eigen::Matrix3d rot;

            cv::Rodrigues(rvecs_mat_[idx], res_mat);
            Eigen::Vector3d t, tc; // px py pz
            cv::cv2eigen(res_mat, rot);

            Eigen::Vector3d eulerAngle = rot.eulerAngles(0, 1, 2);
            gtsam::Vector3 rot_rad = gtsam::Rot3::Logmap(gtsam::Rot3(rot));

            cv::cv2eigen(tvecs_mat_[idx], t);

            pattern_pose_file << t(0) << " " << t(1) << " " << t(2) << " " << rot_rad(0) << " " << rot_rad(1) << " " << rot_rad(2) << std::endl;
            
            rot_rad = gtsam::Rot3::Logmap(gtsam::Rot3(rot).inverse());
            eulerAngle = rot.inverse().eulerAngles(0, 1, 2);

            tc =  -rot.transpose()* t;
            pattern_cam_file << tc(0) << " " << tc(1) << " " << tc(2) << " " << eulerAngle(0) << " " << eulerAngle(1) << " " << eulerAngle(2) << std::endl;
        }
        std::vector<float> errors;
        float64_t rms = ComputeReprojectionErrors(object_points_, image_corners_seq_, rvecs_mat_, tvecs_mat_, cv_K, distCoeffs, errors, false);
        std::cout << "#Calibration RMS: " << rms << std::endl;
        return;
    }

    void LaserCameraCalib::ComputeLaserPoint(int idx, float u, float v, float& z_c, float& x_c, float& y_c)
    {
        // pattern board own normal vector
        Eigen::Vector3f pb_normal_identity(0, 0, 1);

        // compute pattern board normal vector at camera coordinate
        Eigen::Vector3f pb_normal_c;
        Eigen::Matrix3f rot;
        Eigen::Vector3f t; // px py pz
        cv::Mat res_mat;
        Rodrigues(rvecs_mat_[idx], res_mat);
        cv::cv2eigen(res_mat, rot);
        cv::cv2eigen(tvecs_mat_[idx], t);
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
    
    void LaserCameraCalib::UndistortPoints(const std::vector<cv::Point2f>& in, std::vector<cv::Point2f> &out) const
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

    void LaserCameraCalib::ResizeImage(cv::Mat& imagesrc, cv::Mat& imagedst)
	{
		double fScale = 0.4;//缩放系数
		//计算目标图像的大小
		cv::Size dsize = cv::Size(imagesrc.cols*fScale, imagesrc.rows*fScale);
		cv::resize(imagesrc, imagedst, dsize);
	}

    void LaserCameraCalib::LaserLineDetector()
    {
        std::vector<cv::String> filenames;
        cv::glob(parameter_.line_image_path, filenames); 
        cv::Mat src_img, src_img_undistortion;
        cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_32F);
        cv::eigen2cv(camera_eigen_matrix_, camera_matrix);

        if(filenames.size() == 0)
        {
            std::cout << "File names size: " << filenames.size() << std::endl;
            return;
        }

        straight_lines_.clear();
        for(size_t i = 0; i < filenames.size(); ++i)
        {
            std::cout<< "File name: " << filenames[i] << std::endl;
            src_img = cv::imread(filenames[i], cv::IMREAD_GRAYSCALE);

            if(!src_img.data)
                std::cerr << "Problem loading image!!!" << std::endl;
            
            cv::undistort(src_img, src_img_undistortion, camera_matrix, dist_coeffs_);
            
            StraightLine straight_line;
            if(LaserLineDetector(src_img_undistortion, straight_line))
            {
                straight_lines_.push_back(straight_line);
                // @TODO
                // if image can't extract line or corners, You should keep right corresponsible relation.
            }
            
        }
    }

    void drawLine(cv::Mat &img, std::vector<cv::Vec2f> lines,  double rows,   double cols, cv::Scalar scalar, int n)
    {
        cv::Point pt1, pt2;
        for (size_t i = 0; i < lines.size(); i++)
        {
            float rho = lines[i][0];  //直线距离坐标原点的距离
            float theta = lines[i][1];  //直线过坐标原点垂线与x轴夹角
            double a = cos(theta);  //夹角的余弦值
            double b = sin(theta);  //夹角的正弦值
            double x0 = a*rho, y0 = b*rho;  //直线与过坐标原点的垂线的交点
            double length = std::max(rows, cols);  //图像高宽的最大值
                                            //计算直线上的一点
            pt1.x = cvRound(x0 + length  * (-b));
            pt1.y = cvRound(y0 + length  * (a));
            //计算直线上另一点
            pt2.x = cvRound(x0 - length  * (-b));
            pt2.y = cvRound(y0 - length  * (a));
            //两点绘制一条直线
            line(img, pt1, pt2, scalar, n);
        }
    }

    void LaserCameraCalib::GetLaserLines(std::vector<StraightLine>& straight_lines)
    {
        straight_lines = straight_lines_;
    }

    bool LaserCameraCalib::LaserLineDetector(cv::Mat& src_image, StraightLine& straight_line)
    {
	    cv::Mat edge;  
	    cv::Mat image_dst;
        
        std::vector<cv::Vec4f> lines;
        // ResizeImage(src_image, src_image); ///!!!!!!!!!!!!!!!!!
        
        std::cout << "threshold";
        // Canny(src_image, edge, 230, 230, 3, false);
	    cv::threshold(src_image, edge, 230.0, 255, cv::THRESH_BINARY);  
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(edge, image_dst, cv::MORPH_CLOSE, element);
        std::vector<cv::Point> points;

        for(int i = 0; i < image_dst.cols; i++)
        {
            float sum_pix = 0, sum_cols_pix = 0;
            for(int j = 0; j < image_dst.rows; j++)
            {
                float pixel = image_dst.data[i + j* image_dst.cols];
                sum_pix += pixel;
                sum_cols_pix += pixel* j;
            }
            float ave_rows = 0;
            ave_rows = sum_cols_pix / sum_pix;
            
            cv::circle(image_dst, cv::Point(i, ave_rows), 2, cv::Scalar(155));
            points.push_back(cv::Point(i, ave_rows));
        }

        cv::Vec4f line;
        cv::fitLine(points, line, cv::DIST_L2, 0, 0.01,0.01);

        float laser_line_k = line(1) / line(0);
        float laser_line_b = line(3) - line(2)* laser_line_k;
        straight_line.k = laser_line_k;
        straight_line.b = laser_line_b;

        float u_0 = 0;
        float v_0 = laser_line_k* 0 + laser_line_b;
        float u_1 = 2000; 
        float v_1 = laser_line_k* 2000 + laser_line_b;

        std::cout << "Laser K: " << laser_line_k << " , Laser B: " << laser_line_b << std::endl;
        cv::line(image_dst, cv::Point(u_0, v_0), cv::Point(u_1, v_1), cv::Scalar(55), 2);
        ResizeImage(image_dst, image_dst);
        cv::imshow("img", image_dst);
        cv::waitKey(0);

        return true;
    }

    void LaserCameraCalib::ComputeLaserPoint(float p_a, float p_b, float p_c, float u, float v, float x_c, float y_c, float z_c)
    {
        float u0 = camera_eigen_matrix_(0,2);
        float v0 = camera_eigen_matrix_(1,2);
        float kx = camera_eigen_matrix_(0,0);
        float ky = camera_eigen_matrix_(1,1);

        z_c = p_c* kx* ky /(kx* ky + p_a* ky* (u0- u) + p_b* kx* (v0 - v));
        x_c = z_c* (u - u0)/ kx;
        y_c = z_c* (v - v0)/ ky;
    }

    bool LaserCameraCalib::SaveCameraMatrix(const Eigen::Matrix3f& camera_matrix)
    {
    }
    
    double LaserCameraCalib::ComputeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                                const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                                const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                                const cv::Mat& cv_K , const cv::Mat& distCoeffs,
                                                std::vector<float>& perViewErrors, bool fisheye)
    {
        std::vector<cv::Point2f> imagePoints2;
        size_t totalPoints = 0;
        double totalErr = 0, err;
        perViewErrors.resize(objectPoints.size());
        for(size_t i = 0; i < objectPoints.size(); ++i )
        {
            if (fisheye)
            {
                cv::fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cv_K,
                distCoeffs);
            }
            else
            {
                projectPoints(objectPoints[i], rvecs[i], tvecs[i], cv_K, distCoeffs, imagePoints2);
            }
            err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);
            size_t n = objectPoints[i].size();
            perViewErrors[i] = (float) std::sqrt(err*err/n);
            totalErr += err*err;
            totalPoints += n;
        }
        return std::sqrt(totalErr/totalPoints);
    }

    void LaserCameraCalib::Handeye()
    {
        std::ifstream file;
        file.open(parameter_.egm_path);
        if(!file.is_open())
        {
            std::cout << "Not open Egm file: " << parameter_.egm_path << std::endl;
            return;
        }
        std::string str;
        std::istringstream iss;
        double timestamp, x, y, z, ex, ey, ez;
        std::vector<gtsam::Pose3> sensors, robots;

        while(std::getline(file, str))
        {
            iss.clear();
            iss.str(str);
            iss >> timestamp >> x >> y >> z >> ex >> ey >> ez;
            gtsam::Rot3 r_rot = gtsam::Rot3::Expmap(Eigen::Vector3d(ex /180.0* M_PI, ey/180.0* M_PI, ez/180.0* M_PI));
            gtsam::Pose3 robot(r_rot, Eigen::Vector3d(x, y, z));
            robots.push_back(robot);
        }
        file.close();

        for(uint idx = 0; idx < tvecs_mat_.size(); idx++)
        {
            Eigen::Matrix3d rotation;
            Eigen::Vector3d translation;
            Eigen::Matrix3f rotation_f;
            Eigen::Vector3f translation_f;
            cv::Mat res_mat;
            cv::Rodrigues(rvecs_mat_[idx], res_mat);
            cv::cv2eigen(res_mat, rotation_f);
            rotation = rotation_f.cast<double>();
            cv::cv2eigen(tvecs_mat_[idx], translation_f);
            translation = translation_f.cast<double>();

            gtsam::Rot3 c_rot = gtsam::Rot3(rotation);
            gtsam::Pose3 sensor(c_rot, translation);
            
            sensors.push_back(sensor);
        }

        std::cout << "sensors's size: " << sensors.size() << "robot size: " << robots.size() << std::endl;

        assert(robots.size() == sensors.size());
        
        gtsam::Vector3 rot, t;
        HandEyeCalib handeye_calib(robots, sensors);
        handeye_calib.CalibrationRotationSVD(rot);
        handeye_calib.CalibrateRotationGtsam(rot);
        
        handeye_calib.CalibrateTranslationLS(rot, t);
        handeye_calib.CalibrateTranslationGtsam(rot, t);
        
        std::cout << "handeye angle: " << rot << std::endl;
    }

    void LaserCameraCalib::CalibrateWithCrossRatio()
    {
        if(straight_lines_.size() == 0)
        {
            std::cout << "Straight line is null" << std::endl;
        }

        std::vector<cv::Point3f> realPoint;
        for (int i = 0; i < parameter_.corner_rows; i++) {
            for (int j = 0; j < parameter_.corner_cols; j++) {
                cv::Point3f tempPoint;
                tempPoint.x = i * parameter_.cornersize_rows;
                tempPoint.y = j * parameter_.cornersize_cols;
                tempPoint.z = 0;
                realPoint.push_back(tempPoint);
            }
        }

        for(uint32_t idx_img = 0; idx_img < image_size_; idx_img++)
        {
            std::cout << "idx img: " << idx_img << std::endl;

            StraightLine line = straight_lines_[idx_img];
            float        k_line = line.k;
            float        b_line = line.b;

            std::vector<cv::Point2f> corners = image_corners_undistorted_[idx_img];
            
            uint32_t idx_rows_A = 0;
            uint32_t idx_rows_B = 2;
            uint32_t idx_rows_E = parameter_.corner_rows - 1;

            uint32_t idx_cols_A_start = idx_rows_A* parameter_.corner_cols;
            uint32_t idx_cols_B_start = idx_rows_B* parameter_.corner_cols;
            uint32_t idx_cols_E_start = idx_rows_E* parameter_.corner_cols;

            for(uint32_t idx_cols = 0; idx_cols < parameter_.corner_cols; idx_cols++)
            {
                cv::Point2f a = corners[idx_cols_A_start + idx_cols];
                cv::Point2f b = corners[idx_cols_B_start + idx_cols];
                cv::Point2f e = corners[idx_cols_E_start + idx_cols];
                cv::Point3f A = realPoint[idx_cols_A_start + idx_cols];
                cv::Point3f B = realPoint[idx_cols_B_start + idx_cols];
                cv::Point3f E = realPoint[idx_cols_E_start + idx_cols];

                std::vector<cv::Point2f> cols_corners;
                for(uint32_t idx_rows = 0; idx_rows < parameter_.corner_rows; idx_rows++)
                {
                    uint32_t idx_corner = idx_rows* parameter_.corner_cols + idx_cols;
                    cols_corners.push_back(corners[idx_corner]);
                }

                float au = a.x;
                float av = a.y;
                float bu = b.x;
                float bv = b.y;
                float eu = e.x;
                float ev = e.y;

                float Ax = A.x;
                float Ay = A.y;
                float Bx = B.x;
                float By = B.y;
                float Ex = E.x;
                float Ey = E.y;

                cv::Vec4f line;
                cv::fitLine(cols_corners, line, cv::DIST_L2, 0, 0.01,0.01);

                float cols_line_k = line(1) / line(0);
                float cols_line_b = line(3) - line(2)* cols_line_k;          

                float u_cross = (cols_line_b - b_line) / (k_line - cols_line_k);
                float v_cross = (b_line* cols_line_k - cols_line_b* k_line) / (cols_line_k - k_line);

                std::cout << "cols line k: " << cols_line_k << std::endl;
                std::cout << "cols line b: " << cols_line_b << std::endl;
                std::cout << "u     cross: " << u_cross << " , " << "v cross: " << v_cross << std::endl;
                std::cout << "cross ratio: " << au << " , " << bu << " , " << u_cross << " , " << eu << " , " << Ax << " , " << Bx << " , " << Ex << std::endl;
                std::cout << "             " << av << " , " << bv << " , " << v_cross << " , " << ev << " , " << Ay << " , " << By << " , " << Ey << std::endl;
                // float Cx = CrossRatio(au, bu, u_cross, eu, Ax, Bx, Cx);
                float Cy = CrossRatio(av, bv, v_cross, ev, Ax, Bx, Ex);
                float Cx = Ay;

                std::cout << "Cx: " << Cx << " , Cy: " << Cy << std::endl;
            }
            
            // std::cout << corners << std::endl;
        }

    }

    float LaserCameraCalib::CrossRatio(float a, float b, float c, float e, float A, float B, float E)
    {
        // AC/AE   ac/ae 
        // ----- = -----
        // BC/BE   bc/be
        float cross_ratio = (a - c)*(b - e) / (a - e) / (b - c);
        float r = cross_ratio*(A-E)/(B-E);
        float C = (A- r*B)/(1-r);
        return C;
    }

} 
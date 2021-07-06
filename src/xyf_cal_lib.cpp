#include <xyf_cal_lib.h>

namespace calibration
{
    cm_xyf::output cm_xyf::getcmdata(void)
    {
        output cm_output;
        std::vector<cv::String> filenames; // notice here that we are using the Opencv's embedded "String" class

        cv::glob(cm_data_.fold_path, filenames); // new function that does the job ;-)
        vector<vector<cv::Point3f>> object_points; /* chessboardcorner in real world(mm) */
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
            cv::Size square_size = cv::Size(cm_data_.corners_ize_rows, cm_data_.corners_ize_cols);

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
            object_points.push_back(realPoint);
            // printf("#objectPoints %ld\n", sizeof(object_points[0]));
            // cout << object_points[0] << endl;

            // printf("#image_points %ld\n", sizeof(image_points_seq_[0]));
            // cout << corners_ << endl;
        }

        //TODO: check save style
        //**save objectpointdata in .yml
        cv::FileStorage temp_file1("object_points.yml" ,cv::FileStorage::WRITE);
        temp_file1 << "image_points_seq_" << image_points_seq_;
        temp_file1 << "object_points" << object_points;
        temp_file1.release();
        
        calibrateCamera(object_points, image_points_seq_ , src_img.size() , cameraMatrix , distCoeffs , rvecsMat_ , tvecsMat_);

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

        cv::waitKey(0);
        return cm_output;
        
    }

}
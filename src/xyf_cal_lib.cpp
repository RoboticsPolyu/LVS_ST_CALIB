#include <xyf_cal_lib.h>

namespace calibration
{
    cm_xyf::output cm_xyf::getcmdata(void)
    {
        cv::Mat image_original = cv::imread(cm_data_.image_path, 0);
        cv::Size imagesize = image_original.size();
        cv::Size corner_size = cv::Size(cm_data_.corner_rows, cm_data_.corner_cols);
        int patternfound = cv::findChessboardCorners (image_original, corner_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        cout << corners.size() << endl;
        //find4QuadCornerSubpix(image, corners, Size(5, 5));
        //cornerSubPix(image, corners, Size(11, 11), Size(-1, -1),
        //TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
        image_points_seq.push_back(corners);

        drawChessboardCorners(image_original , corner_size , corners , patternfound);
        cv::imwrite(cm_data_.post_path_1, image_original);

        cv::Mat map1; // = Mat(image.size(), CV_32FC1, Scalar(0));
        cv::Mat map2;// = Mat(image.size(), CV_32FC1, Scalar(0));
        cv::Mat cv_R = cv::Mat::eye(3, 3, CV_32F);
        cv::Size square_size = cv::Size(cm_data_.cornersize_rows, cm_data_.cornersize_cols);
        vector<vector<cv::Point3f>> object_points; /* chessboardcorner in real world(mm) */
        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* camera intrinsic mat */
        cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* camera distortion data k1,k2,p1,p2,k3 */

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

        //**save objectpointdata in .yml
        cv::FileStorage temp_file1("object_points.yml" ,cv::FileStorage::WRITE);
        temp_file1 << "image_points_seq" << image_points_seq;
        temp_file1 << "object_points" << object_points;
        temp_file1.release();
    
        printf("#objectPoints�� %ld\n", sizeof(object_points[0]));
        cout << object_points[0] << endl;

        printf("#image_points�� %ld\n", sizeof(image_points_seq[0]));
        cout << corners << endl;


        calibrateCamera(object_points, image_points_seq , image_original.size() , cameraMatrix , distCoeffs , rvecsMat , tvecsMat);

        cv::Mat Knew_ = cameraMatrix.clone();
        // Mat tmp = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imagesize, 1, imagesize, 0);

        initUndistortRectifyMap(cameraMatrix , distCoeffs , cv_R, Knew_, imagesize , CV_16SC2, map1 , map2);
        //cv::remap(image_original, imageCalibration , map1 , map2 , cv::INTER_LINEAR);
        
        undistort(image_original, image_undistort , cameraMatrix , distCoeffs);
        cv::imwrite(cm_data_.post_path_2, image_undistort);
        Rodrigues(rvecsMat[0], res_mat);

        cout << "tvecsMat:\n" << tvecsMat[0] << endl;
        cout << "rvecsMat:\n" << rvecsMat[0] << endl;
        cout << "#cameraMatrix:\n" << cameraMatrix << endl;
        cout << "#distCoeffs:\n" << distCoeffs << endl;
        cout << "#res_mat:\n" << res_mat << endl;
        
        output cm_output;
        
        cm_output.tvecsMat = tvecsMat;
        cm_output.rvecsMat = res_mat;
        cm_output.cameraMatrix = cameraMatrix;
        cm_output.distCoeffs = distCoeffs;

        return cm_output;
        
    }

}
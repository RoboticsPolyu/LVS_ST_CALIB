#ifndef __LASER_CAMERA_CAL_H__
#define __LASER_CAMERA_CAL_H__

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

#include "types.h"
#include "gtsam_wrapper.h"

#define CV_TERMCRIT_ITER    1
#define CV_TERMCRIT_NUMBER  CV_TERMCRIT_ITER
#define CV_TERMCRIT_EPS     2

namespace calibration
{

enum CAMERA_MODEL: uint8_t
{
	FISHEYE = 0,
	PINHOLE,
};

//
class GTSAM_EXPORT CalFactor: public gtsam::NoiseModelFactor3<gtsam::Rot3, gtsam::Point3, gtsam::Vector9>
{
    public:
        typedef gtsam::NoiseModelFactor3<gtsam::Rot3, gtsam::Point3, gtsam::Vector9>  Base;
        typedef gtsam::noiseModel::Diagonal::shared_ptr SharedDiagonal;

	CalFactor(const gtsam::Key key_rot, const gtsam::Key key_t, const gtsam::Key key_cal,
		const::Point3& p, const gtsam::Point2& uv, const SharedDiagonal& sigma)
		: Base(sigma, key_rot, key_t, key_cal), p_(p), uv_(uv)
	{
	}

	gtsam::Vector evaluateError(const gtsam::Rot3& R, const gtsam::Point3& t, gtsam::Vector9& K,
            boost::optional<gtsam::Matrix&> HR = boost::none,
			boost::optional<gtsam::Matrix&> Ht = boost::none,
			boost::optional<gtsam::Matrix&> Hk = boost::none) const
	{
		gtsam::Cal3DS2 cal2ds2(K);
		gtsam::Matrix33 D_R, D_t;
		gtsam::Point3 xyz = R.rotate(p_, D_R, boost::none) + t;
		              D_t = gtsam::Matrix33::Identity();
		
		gtsam::Point2 pxy(xyz[0] / xyz[2], xyz[1] / xyz[2]);
		gtsam::Matrix29 Jac_K;
		gtsam::Matrix22 Jac_p;
		cal2ds2.calibrate(pxy, Jac_K, Jac_p);

	}

	virtual ~CalFactor()
	{
	}
	
	private:
		gtsam::Point2 uv_;
		gtsam::Point3 p_;
		gtsam::Cal3DS2 cal3cs2_;

};

// PnP Project Factor
class GTSAM_EXPORT PnPFactor: public gtsam::NoiseModelFactor2<gtsam::Rot3, gtsam::Vector3>
{
    public:
        typedef gtsam::NoiseModelFactor2<gtsam::Rot3, gtsam::Vector3>  Base;
        typedef gtsam::noiseModel::Diagonal::shared_ptr SharedDiagonal;

        PnPFactor(const gtsam::Key key_rot, const gtsam::Key key_t,
                  const gtsam::Vector3& plate_xyz,
                  const gtsam::Vector2& camera_uv,
				  const gtsam::Matrix33& K,
                  const SharedDiagonal& sigma)
                    : Base(sigma, key_rot, key_t), K_(K), plate_xyz_(plate_xyz), camera_uv_(camera_uv)
        {
        }

        virtual ~PnPFactor()
        {
        }

        gtsam::Vector evaluateError(const gtsam::Rot3& rot_plate_cam, const gtsam::Vector3& t_plate_cam,
            boost::optional<gtsam::Matrix&> H_rot = boost::none,
			boost::optional<gtsam::Matrix&> H_t = boost::none) const
        {
			gtsam::Vector error;
			gtsam::Matrix33 jac_xyz2_rot, jac_xyz2_t;
			gtsam::Vector3 xyz2 = rot_plate_cam.rotate(plate_xyz_, jac_xyz2_rot) + t_plate_cam;
			jac_xyz2_t  =gtsam::Matrix33::Identity();
			gtsam::Vector3 err3 = K_* xyz2;
			error = gtsam::Vector2(err3[0] /err3[2], err3[1] /err3[2]) - camera_uv_;

			gtsam::Matrix23 jac_err_xyz2;
			jac_err_xyz2 << 1/xyz2(2), 0, -xyz2(0)/ (xyz2(2)* xyz2(2)), 0, 1/xyz2(1), -xyz2(1)/(xyz2(2)* xyz2(2));
			jac_err_xyz2 = jac_err_xyz2* K_;

            if(H_rot)
            {
				*H_rot = jac_err_xyz2* jac_xyz2_rot;

            }

			if(H_t)
			{
				*H_t = jac_err_xyz2* jac_xyz2_t;
			}

            return error;
        }

    private:
		gtsam::Vector3 plate_xyz_;
		gtsam::Vector2 camera_uv_;
		gtsam::Matrix33 K_;
};


class LaserCameraCalib
{
	public:
	    struct Parameters
	    {
		    int     corner_rows;
		    int     corner_cols;
		    float   cornersize_rows;
		    float   cornersize_cols;
		    std::string  fold_path;
			std::string  line_image_path;
			std::string  egm_path;
			uint8_t camera_model;
			bool    use_intrinsic;

			void operator()(const Parameters & my_struct)
		    {
			   fold_path = my_struct.fold_path;
			   corner_cols = my_struct.corner_cols;
			   corner_rows = my_struct.corner_rows;
			   cornersize_cols = my_struct.cornersize_cols;
			   cornersize_rows = my_struct.cornersize_rows;
			   line_image_path = my_struct.line_image_path;
			   camera_model = my_struct.camera_model;
			   use_intrinsic = my_struct.use_intrinsic;
			   egm_path = my_struct.egm_path;
		    }

			void print()
			{
				std::cout << "**********************************************" << std::endl;
				std::cout << "Laser Camera Calibration Parameter: " << std::endl;
				std::cout << "corner_cols: " << corner_cols << std::endl;
				std::cout << "corner_rows: " << corner_rows << std::endl;
				std::cout << "cornersize_cols: " << cornersize_cols << std::endl;
				std::cout << "cornersize_rows: " << cornersize_rows << std::endl;
				std::cout << "fold_path: " << fold_path << std::endl;
				std::cout << "line_image_path: " << line_image_path << std::endl;
				std::cout << "camera_mode: " << (uint32_t)camera_model << std::endl;
				std::cout << "use intrinsic: " << (use_intrinsic ? "true" : "false") << std::endl;
				std::cout << "egm path: " << egm_path << std::endl;
				std::cout << "**********************************************" << std::endl;
			}
	    };

		struct StraightLine
		{
			float k;
			float b;
		};

	   	LaserCameraCalib()
		{
			dist_coeffs_ = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
			camera_eigen_matrix_.setZero();
		}
		
		LaserCameraCalib(Parameters parameter)
	    {
			parameter_(parameter);
	    }

		std::vector<std::vector<cv::Point2f>> GetImageCorners()
		{
			return image_corners_seq_;
		}

		std::vector<std::vector<cv::Point3f>> GetBoardCorners()
		{
			return object_points_;
		}

		const Parameters& GetParams() const
		{
			return parameter_;
		};

		bool LoadParameter(const std::string& filename);

		void ResizeImage(cv::Mat& imagesrc, cv::Mat& imagedst);

		void Calibrate(void);

		void ComputeLaserPoint(int idx, float u, float v, float& z_c, float& x_c, float& y_c);

		void UndistortPoints(const std::vector<cv::Point2f>& in, std::vector<cv::Point2f> &out) const;

		const int GetValidImageSize() const 
		{
			return image_size_;
		}

		void LaserLineDetector();
		
		void GetLaserLines(std::vector<StraightLine>& straight_lines);

		bool LaserLineDetector(cv::Mat& src_image, StraightLine& straght_line);
		
		/**
		 * p_a p_b p_c ; plane paramters
		 * p_a*x + p_b*y + p_c = z
		 *  u, v : laser point
		 *  x_c, y_c, z_c: 3d position at camera coordinate 
		 */
		void ComputeLaserPoint(float p_a, float p_b, float p_c, float u, float v, float x_c, float y_c, float z_c);
		
		float64_t ComputeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
												const std::vector<std::vector<cv::Point2f> >& imagePoints,
												const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
												const cv::Mat& cv_K , const cv::Mat& distCoeffs,
												std::vector<float>& perViewErrors, bool fisheye);

		void Handeye();
		
		/**
		 * Calibrate with Cross ratio invariant
		 */
		void CalibrateWithCrossRatio();
		
		float CrossRatio(float a, float b, float c, float e, float A, float B, float E);

   	private:
		bool SaveCameraMatrix(const Eigen::Matrix3f& camera_matrix);

	    struct Parameters parameter_;
		
		std::vector<std::vector<cv::Point2f>> image_corners_seq_;
		std::vector<std::vector<cv::Point2f>> image_corners_undistorted_;
		std::vector<std::vector<cv::Point3f>> object_points_;
	    
		// translation and rotation of pattern coordinate at camera coordnite
	    std::vector<cv::Mat> tvecs_mat_; // t_c_p
	    std::vector<cv::Mat> rvecs_mat_; // r_c_p

		std::vector<StraightLine> straight_lines_;
		
		Eigen::Matrix3f   camera_eigen_matrix_;
		cv::Mat           dist_coeffs_;
		
		float             k1_, k2_, p1_, p2_, k3_;            

		int image_size_;	
		
	public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif // __LASER_CAMERA_CAL_H__ 
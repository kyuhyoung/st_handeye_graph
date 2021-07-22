#include <regex>
#include <random>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include <visp/vpDebug.h>
#include <visp/vpPoint.h>
#include <visp/vpCalibration.h>
#include <visp/vpExponentialMap.h>

#include <st_handeye/st_handeye.hpp>

#ifndef OPENCV_BELOW_3
#include <st_handeye/epnp.h>
#endif  //  OPENCV_BELOW_3

namespace st_handeye
{

    cv::Mat eigen2cvmat( const Eigen::MatrixXd& matrix )
    {
        cv::Mat cv_mat( matrix.rows(), matrix.cols(), CV_64FC1 );

        for ( int i = 0; i < matrix.rows(); i++ )
        {
            for ( int j = 0; j < matrix.cols(); j++ )
            {
                cv_mat.at<double>( i, j ) = matrix( i, j );
            }
        }

        return cv_mat;
    }

    Eigen::MatrixXd cvmat2eigen( const cv::Mat& matrix )
    {
        Eigen::MatrixXd eigen_mat( matrix.rows, matrix.cols );

        for ( int i = 0; i < matrix.rows; i++ )
        {
            for ( int j = 0; j < matrix.cols; j++ )
            {
                eigen_mat( i, j ) = matrix.at<double>( i, j );
            }
        }

        return eigen_mat;
    }

    vpHomogeneousMatrix eigen2vpmat( const Eigen::MatrixXd& matrix )
    {
        vpHomogeneousMatrix vpmat;

        for ( int i = 0; i < 4; i++ )
        {
            for ( int j = 0; j < 4; j++ )
            {
                vpmat[i][j] = matrix( i, j );
            }
        }

        return vpmat;
    }

    Eigen::MatrixXd vpmat2eigen( const vpHomogeneousMatrix& matrix )
    {
        Eigen::MatrixXd eigen_mat( 4, 4 );

        for ( int i = 0; i < 4; i++ )
        {
            for ( int j = 0; j < 4; j++ )
            {
                eigen_mat( i, j ) = matrix[i][j];
            }
        }

        return eigen_mat;
    }

    std::vector<Eigen::Isometry3d> calc_object2eyes(
        const Eigen::Matrix3d& camera_matrix,
        const Eigen::MatrixXd& pattern_3d,
        const std::vector<Eigen::MatrixXd>& pattern_2ds
    )
    {
        std::vector<Eigen::Isometry3d> object2eyes( pattern_2ds.size() );
        cv::Mat cv_camera_matrix = eigen2cvmat( camera_matrix );
        cv::Mat cv_pattern_3d = eigen2cvmat( pattern_3d );

        for ( int i = 0; i < pattern_2ds.size(); i++ )
        {
            cv::Mat cv_pattern_2d = eigen2cvmat( pattern_2ds[i] );
            cv::Mat rvec = cv::Mat::zeros( 3, 1, CV_64FC1 ), tvec = cv::Mat::zeros( 3, 1, CV_64FC1 );
            cv::solvePnP( cv_pattern_3d.t(), cv_pattern_2d.t(), cv_camera_matrix, cv::Mat(), rvec, tvec );
            cv::Mat rotation;
            cv::Rodrigues( rvec, rotation );
            Eigen::Isometry3d object2eye = Eigen::Isometry3d::Identity();
            object2eye.translation() = cvmat2eigen( tvec );
            object2eye.linear() = cvmat2eigen( rotation );
            object2eyes[i] = object2eye;
        }

        return object2eyes;
    }

#ifndef OPENCV_BELOW_3
    std::vector<Eigen::Isometry3d> cMo_epnp( const Eigen::Matrix3d& camera_matrix, const Eigen::MatrixXd& pattern_3d, const std::vector<Eigen::MatrixXd>& pattern_2ds )
    {
        std::vector<Eigen::Isometry3d> li_cMo;
        epnp PnP;
        double fu = camera_matrix( 0, 0 ), fv = camera_matrix( 1, 1 ), uc = camera_matrix( 0, 2 ), vc = camera_matrix( 1, 2 );
        PnP.set_internal_parameters( uc, vc, fu, fv );
        int n_frm = pattern_2ds.size();

        for ( int iF = 0; iF < n_frm; iF++ )
        {
            int n_pt = /*pattern_2ds[iF].rows()*/ pattern_2ds[iF].cols();
            PnP.set_maximum_number_of_correspondences( n_pt );
            PnP.reset_correspondences();

            for ( int iP = 0; iP < n_pt; iP++ )
            {
                double Xw = pattern_3d( 0, iP ), Yw = pattern_3d( 1, iP ), Zw = pattern_3d( 2, iP );
                double u = pattern_2ds[iF]( 0, iP ), v = pattern_2ds[iF]( 1, iP );
                PnP.add_correspondence( Xw, Yw, Zw, u, v );
            }

            double R_est[3][3], t_est[3];
            std::cout << "AAAA" << std::endl; //exit(0);
            PnP.compute_pose( R_est, t_est );
            //Mat cMo = cv::Mat::eye(4, 4, CV_64F);
            Eigen::Isometry3d cMo;

            for ( int iR = 0; iR < 3; iR++ )
            {
                for ( int iC = 0; iC < 3; iC++ )
                {
                    cMo.matrix()( iR, iC ) = R_est[iR][iC];
                }

                cMo.matrix()( iR, 3 ) = t_est[iR];
            }

            cMo.matrix()( 3, 0 ) = 0.0;
            cMo.matrix()( 3, 1 ) = 0.0;
            cMo.matrix()( 3, 2 ) = 0.0;
            cMo.matrix()( 3, 3 ) = 1.0;
            li_cMo.push_back( cMo );
            //std::cout << "cMo.matrix() : " << std::endl << cMo.matrix() << std::endl;
            //std::cout << "BBBB" << std::endl; exit(0);
        }

        return li_cMo;
    }
#endif  //  OPENCV_BELOW_3


    /**
     * @brief estimates hane-eye transformation using Tsai's algorithm
     */
    bool spatial_calibration_visp (
        const Eigen::Matrix3d& camera_matrix,
        const Eigen::MatrixXd& pattern_3d,
        const std::vector<Eigen::Isometry3d>& world2hands,
        const std::vector<Eigen::MatrixXd>& pattern_2ds,
        Eigen::Isometry3d& hand2eye,
        Eigen::Isometry3d& object2world,
        OptimizationParams params
    )
    {
#ifdef OPENCV_BELOW_3
        std::vector<Eigen::Isometry3d> object2eyes = calc_object2eyes( camera_matrix, pattern_3d, pattern_2ds );
#else   //  OPENCV_BELOW_3
        std::vector<Eigen::Isometry3d> object2eyes = cMo_epnp( camera_matrix, pattern_3d, pattern_2ds );
#endif  //  OPENCV_BELOW_3
        std::vector<vpHomogeneousMatrix> vp_eye2objects;
        std::vector<vpHomogeneousMatrix> vp_world2hands;

        for ( int i = 0; i < world2hands.size(); i++ )
        {
            vp_eye2objects.push_back( eigen2vpmat( object2eyes[i].matrix() ) );
            vp_world2hands.push_back( eigen2vpmat( world2hands[i].inverse().matrix() ) );
        }

        vpHomogeneousMatrix vp_hand2eye = eigen2vpmat( hand2eye.matrix() );
        vpCalibration::calibrationTsai( vp_eye2objects, vp_world2hands, vp_hand2eye );
        Eigen::Matrix4d hand2eye_ = vpmat2eigen( vp_hand2eye ).inverse();
        hand2eye.translation() = hand2eye_.block<3, 1>( 0, 3 );
        hand2eye.linear() = hand2eye_.block<3, 3>( 0, 0 );
        object2world = world2hands[0].inverse() * hand2eye.inverse() * object2eyes[0];
        return true;
    }

    /**
     * @brief estimates hane-eye transformation using Dual quaternions-based method
     */
    bool spatial_calibration_dualquaternion (
        const Eigen::Matrix3d& camera_matrix,
        const Eigen::MatrixXd& pattern_3d,
        const std::vector<Eigen::Isometry3d>& world2hands,
        const std::vector<Eigen::MatrixXd>& pattern_2ds,
        Eigen::Isometry3d& hand2eye,
        Eigen::Isometry3d& object2world,
        OptimizationParams params
    )
    {
#ifdef OPENCV_BELOW_3
        std::vector<Eigen::Isometry3d> object2eyes = calc_object2eyes( camera_matrix, pattern_3d, pattern_2ds );
#else   //  OPENCV_BELOW_3
        std::vector<Eigen::Isometry3d> object2eyes = cMo_epnp( camera_matrix, pattern_3d, pattern_2ds );
#endif  //  OPENCV_BELOW_3
#if 1
        long pid = getpid();
        long rnd = random();
        std::string input_filename = ( boost::format( "/tmp/poses_%d_%d.yml" ) % pid % rnd ).str();
        std::string output_filename = ( boost::format( "/tmp/handeye_%d_%d.yml" ) % pid % rnd ).str();
        std::string nodename = ( boost::format( "handeye_calib_camodocal_%d_%d" ) % pid % rnd ).str();
        cv::FileStorage fs( input_filename, cv::FileStorage::WRITE );
        fs << "frameCount" << static_cast<int>( world2hands.size() );

        for ( int i = 0; i < world2hands.size(); i++ )
        {
            cv::Mat cv_world2hand = eigen2cvmat( world2hands[i].inverse().matrix() );
            cv::Mat cv_object2eye = eigen2cvmat( object2eyes[i].inverse().matrix() );
            fs << ( boost::format( "T1_%d" ) % i ).str() << cv_world2hand;
            fs << ( boost::format( "T2_%d" ) % i ).str() << cv_object2eye;
        }

        fs.release();
        std::stringstream sst;
        sst << "rosrun handeye_calib_camodocal handeye_calib_camodocal "
            << "__name:=" << nodename << " "
            << "_load_transforms_from_file:=true "
            << "_transform_pairs_load_filename:=" << input_filename << " "
            << "_output_calibrated_transform_filename:=" << output_filename;
        std::string command = sst.str();
        std::cout << "command: " << command << std::endl;

        if ( system( command.c_str() ) )
        {
            return false;
        }

        cv::FileStorage ifs( output_filename, cv::FileStorage::READ );
        cv::Mat cv_hand2eye;
        ifs["ArmTipToMarkerTagTransform"] >> cv_hand2eye;
        Eigen::Matrix4d h2e = cvmat2eigen( cv_hand2eye );
        hand2eye = Eigen::Isometry3d( h2e ).inverse();
        object2world = world2hands[0].inverse() * hand2eye.inverse() * object2eyes[0];
#endif  //  0
        return true;
    }

}

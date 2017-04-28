/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud.h"
#include "boost/thread.hpp"
#include "util/SophusUtil.h"
#include "DataStructures/types.h"
#include "util/rosPub.h"
#include "visensor_node/visensor_imu.h"
#include <tf/transform_broadcaster.h>

namespace cv {
	class Mat;
}


namespace lsd_slam
{

class SlamSystem;

struct LiveSLAMWrapper
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LiveSLAMWrapper(std::string packagePath, ros::NodeHandle& _nh, const CALIBRATION_PAR& calib_par);

	/** Destructor. */
	~LiveSLAMWrapper();
	
    void popAndSetGravity() ;

	/** Runs the main processing loop. Will never return. */
	void Loop();

    /** Runs the main processing loop. Will never return. */
    void BALoop();

    void pubCameraLink() ;

    void pubPointCloud(int num, ros::Time imageTimeStamp , Matrix3d R_vi_2_odometry) ;

//	/** Resets everything, starting the odometry from the beginning again. */
//	void resetAll();

	/** Writes the given time and pose to the outFile. */
    void logCameraPose();
	
	inline SlamSystem* getSlamSystem() {return monoOdometry;}
	
    std::list<ImageMeasurement> image0Buf;
    std::list<ImageMeasurement> image1Buf;
    std::list<ImageMeasurement>::iterator pImage0Iter;
    std::list<ImageMeasurement>::iterator pImage1Iter;
    boost::mutex image0_queue_mtx;
    boost::mutex image1_queue_mtx;

    std::list<visensor_node::visensor_imu> imuQueue;
    std::list<visensor_node::visensor_imu>::iterator currentIMU_iter;
    boost::mutex imu_queue_mtx;

	// initialization stuff
	bool isInitialized;
    Eigen::Vector3d gravity_b0 ;
    Eigen::Matrix3d R_vi_2_odometry ;
    Eigen::Matrix3d R_i_2_c ;
    Eigen::Vector3d T_i_2_c ;

	// monoOdometry
	SlamSystem* monoOdometry;

	std::string outFileName;
    std::ofstream outFile;
	
	float fx, fy, cx, cy;
    double sumDist ;
	int width, height;

	int imageSeqNumber;
    ros::NodeHandle nh ;
    ros::Time lastLoopClorsureTime ;

    ros::Time initialTime ;

    int cnt_info_smooth ;
    geometry_msgs::Vector3 to_pub_info ;

    int log_cameraPoseID = 0 ;
    cv::Mat lastestImg ;
};

}

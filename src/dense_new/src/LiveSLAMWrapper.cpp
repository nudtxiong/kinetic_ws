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

#include "LiveSLAMWrapper.h"
#include <vector>
#include <list>
#include <iostream>
#include "util/SophusUtil.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"
#include "IOWrapper/ImageDisplay.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/PointCloud2.h"



namespace lsd_slam
{


LiveSLAMWrapper::LiveSLAMWrapper(std::string packagePath, ros::NodeHandle& _nh, const CALIBRATION_PAR &calib_par)
{
    fx = calib_par.fx;
    fy = calib_par.fy;
    cx = calib_par.cx;
    cy = calib_par.cy;
    width = calib_par.width;
    height = calib_par.height;
    R_i_2_c = calib_par.R_i_2_c;
    T_i_2_c = calib_par.T_i_2_c;
    nh = _nh ;

    isInitialized = false;
    Sophus::Matrix3f K_sophus;
    K_sophus << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

    R_vi_2_odometry << 0, 0, 1, -1, 0, 0, 0, -1, 0 ;

    outFileName = packagePath+"estimated_poses.txt";
//    outFileName = packagePath+"angular_volcity.txt";
    outFile.open(outFileName);

	// make Odometry
    monoOdometry = new SlamSystem(width, height, K_sophus, _nh);

    log_cameraPoseID = 0 ;

	imageSeqNumber = 0;
    sumDist = 0 ;
    cnt_info_smooth = 0 ;
    to_pub_info.x = 0 ;
    to_pub_info.y = 0 ;
    to_pub_info.z = 0 ;
    image0Buf.clear();
    image1Buf.clear();
    imuQueue.clear();
}


LiveSLAMWrapper::~LiveSLAMWrapper()
{
	if(monoOdometry != 0)
		delete monoOdometry;
    if( outFile.is_open() )
	{
        outFile.flush();
        outFile.close();
	}
    image0Buf.clear();
    image1Buf.clear();
    imuQueue.clear();
}

void LiveSLAMWrapper::  popAndSetGravity()
{
    unsigned int image0BufSize ;
    unsigned int image1BufSize ;
    std::list<ImageMeasurement>::reverse_iterator reverse_iterImage ;
    ros::Time tImage ;
    ros::Rate r(100) ;

    gravity_b0.setZero() ;
    while ( nh.ok() )
    {
        ros::spinOnce() ;
        image0_queue_mtx.lock();
        image1_queue_mtx.lock();
        imu_queue_mtx.lock();
        image0BufSize = image0Buf.size();
        image1BufSize = image1Buf.size();
        if ( imuQueue.size() < 120
             || image0BufSize < 2 || image1BufSize < 2
              ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        reverse_iterImage = image0Buf.rbegin() ;
        tImage = reverse_iterImage->t ;
        reverse_iterImage = image1Buf.rbegin() ;
        if ( reverse_iterImage->t < tImage ){
            tImage = reverse_iterImage->t ;
        }
        pImage0Iter = image0Buf.begin();
        pImage1Iter = image1Buf.begin();
        while ( pImage0Iter->t < tImage ){
            pImage0Iter = image0Buf.erase( pImage0Iter ) ;
        }
        while ( pImage1Iter->t < tImage ){
            pImage1Iter = image1Buf.erase( pImage1Iter ) ;
        }
        image0_queue_mtx.unlock();
        image1_queue_mtx.unlock();

        //imu_queue_mtx.lock();
        int imuNum = 0;
        currentIMU_iter = imuQueue.begin() ;
        while( currentIMU_iter->header.stamp < tImage )
        {
            imuNum++;
            gravity_b0(0) += currentIMU_iter->linear_acceleration.x;
            gravity_b0(1) += currentIMU_iter->linear_acceleration.y;
            gravity_b0(2) += currentIMU_iter->linear_acceleration.z;
            currentIMU_iter = imuQueue.erase(currentIMU_iter);
        }
        imu_queue_mtx.unlock();
        gravity_b0 /= imuNum ;
        //gravity_b0 = -gravity_b0 ;
        break ;
    }
    initialTime = tImage ;
    std::cout << "gravity_b0 =\n" ;
    std::cout << gravity_b0.transpose() << "\n" ;
    monoOdometry->gravity_b0 = gravity_b0 ;
    printf("gravity_b0.norm() = %lf\n", gravity_b0.norm() );

    //imu_queue_mtx.unlock();
    cv::Mat image0 = pImage0Iter->image.clone();
    cv::Mat image1 = pImage1Iter->image.clone();
    //image1_queue_mtx.unlock();
    //image0_queue_mtx.unlock();

    monoOdometry->insertFrame(imageSeqNumber, image0, tImage,
                              Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() );
    cv::Mat disparity, depth ;
#if CV_MAJOR_VERSION == 2
    monoOdometry->bm_(image0, image1, disparity, CV_32F);
#elif CV_MAJOR_VERSION == 3
    disparity.create(image0.size(),CV_32F);
    monoOdometry->bm_->compute(image0, image1, disparity);
#endif

    calculateDepthImage(disparity, depth, 0.11, fx );
    monoOdometry->currentKeyFrame = monoOdometry->slidingWindow[0] ;
    monoOdometry->currentKeyFrame->setDepthFromGroundTruth( (float*)depth.data ) ;

    monoOdometry->currentKeyFrame->keyFrameFlag = true ;
    monoOdometry->currentKeyFrame->cameraLinkList.clear() ;
    monoOdometry->RefToFrame = Sophus::SE3() ;

    monoOdometry->margin.initPrior();
    monoOdometry->updateTrackingReference();

//    pubOdometry(Eigen::Vector3d::Zero(),
//            Eigen::Vector3d::Zero(),
//            Eigen::Matrix3d::Identity(),
//            monoOdometry->pub_odometry, monoOdometry->pub_pose,
//            0, R_vi_2_odometry,
//            true, tImage );
    lastLoopClorsureTime = tImage ;
}

void LiveSLAMWrapper::pubCameraLink()
{
    cv::Mat linkListMap(500, 500, CV_8UC3 ) ;
    linkListMap.setTo( cv::Vec3b(200,200,200));
#if CV_MAJOR_VERSION == 2
    cv::Vector<cv::Point2f> locations(slidingWindowSize) ;
#elif CV_MAJOR_VERSION == 3
    std::vector<cv::Point2f> locations(slidingWindowSize) ;
#endif

    double angle_K = 2.0*PI/slidingWindowSize ;
    double r = 200.0 ;
    for ( int i = 0 ; i < slidingWindowSize ; i++ )
    {
        locations[i].x = sin(angle_K*i)*r + 250.0 ;
        locations[i].y = cos(angle_K*i)*r + 250.0 ;
        if ( monoOdometry->slidingWindow[i] == nullptr ){
            continue ;
        }
        if ( monoOdometry->slidingWindow[i]->keyFrameFlag ){
            cv::circle(linkListMap, locations[i], 6, cv::Scalar(255, 0, 0), 5);
        }
        else{
            cv::circle(linkListMap, locations[i], 6, cv::Scalar(0, 0, 255), 5);
        }
        if ( i == monoOdometry->head ){
            cv::circle(linkListMap, locations[i], 6, cv::Scalar(0, 255, 0), 5);
        }
        cv::putText(linkListMap, boost::to_string(i), locations[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    }
    int cnt = 0 ;
    for( int i = 0 ; i < monoOdometry->numOfState ; i++ )
    {
        int idx = monoOdometry->head + i ;
        if ( idx >= slidingWindowSize ){
            idx -= slidingWindowSize ;
        }
        if ( monoOdometry->slidingWindow[i] == nullptr ){
            continue ;
        }
        if ( monoOdometry->slidingWindow[idx]->keyFrameFlag == false){
            continue;
        }
        list<int>::iterator iter =  monoOdometry->slidingWindow[idx]->cameraLinkList.begin();
        for (; iter !=  monoOdometry->slidingWindow[idx]->cameraLinkList.end(); iter++ )
        {
            int linkID = *iter;
            cv::line(linkListMap, locations[idx], locations[linkID], cv::Scalar(0, 255, 255), 3);
            cnt++ ;
        }
    }
    cv::imshow("linkListMap", linkListMap ) ;
    cv::waitKey(1) ;
}

void LiveSLAMWrapper::pubPointCloud(int num, ros::Time imageTimeStamp, Eigen::Matrix3d R_vi_2_odometry )
{
    sensor_msgs::PointCloud2 pc2 ;
    pc2.header.frame_id = "/map";//world
    pc2.header.stamp = imageTimeStamp ;
    pc2.height = 1 ;
    pc2.width = num ;
    pc2.is_bigendian = false ;
    pc2.is_dense = true ;
    pc2.point_step = sizeof(float) * 3 ;
    pc2.row_step = pc2.point_step * pc2.width ;

    sensor_msgs::PointField field;
    pc2.fields.resize(3);
    string f_name[3] = {"x", "y", "z"};
    for (size_t idx = 0; idx < 3; ++idx)
    {
        field.name = f_name[idx];
        field.offset = idx * sizeof(float);
        field.datatype = sensor_msgs::PointField::FLOAT32;
        field.count = 1;
        pc2.fields[idx] = field;
    }
    pc2.data.clear();
    pc2.data.reserve( pc2.row_step );

    vector<float> pt32;
    pt32.resize(num*3);
    int level = 0 ;
    int w = monoOdometry->currentKeyFrame->width(level);
    int h = monoOdometry->currentKeyFrame->height(level);
    float fxInvLevel = monoOdometry->currentKeyFrame->fxInv(level);
    float fyInvLevel = monoOdometry->currentKeyFrame->fxInv(level);
    float cxInvLevel = monoOdometry->currentKeyFrame->fxInv(level);
    float cyInvLevel = monoOdometry->currentKeyFrame->fxInv(level);
    const float* pyrIdepthSource = monoOdometry->currentKeyFrame->idepth(level);
    const float* pyrIdepthVarSource = monoOdometry->currentKeyFrame->idepthVar(level);
    Eigen::Vector3f posDataPT ;
    Eigen::Vector3f posDataOutput ;
    Eigen::Matrix3f R_output ;
    R_output << R_vi_2_odometry(0, 0), R_vi_2_odometry(0, 1), R_vi_2_odometry(0, 2),
            R_vi_2_odometry(1, 0), R_vi_2_odometry(1, 1), R_vi_2_odometry(1, 2),
            R_vi_2_odometry(2, 0), R_vi_2_odometry(2, 1), R_vi_2_odometry(2, 2) ;

    int k = 0 ;
    for(int x=1; x<w-1; x++)
    {
        for(int y=1; y<h-1; y++)
        {
            int idx = x + y*w;

            if(pyrIdepthVarSource[idx] <= 0 || pyrIdepthSource[idx] == 0) continue;

            posDataPT = (1.0f / pyrIdepthSource[idx]) * Eigen::Vector3f(fxInvLevel*x+cxInvLevel,fyInvLevel*y+cyInvLevel,1);
            posDataOutput = R_output*posDataPT ;
            pt32[k++] = posDataOutput(0) ;
            pt32[k++] = posDataOutput(1) ;
            pt32[k++] = posDataOutput(2) ;
        }
    }

    uchar * pt_int = reinterpret_cast<uchar *>(pt32.data());
    for (size_t idx = 0; idx < pc2.row_step; ++idx){
        pc2.data.push_back(pt_int[idx]);
    }
    monoOdometry->pub_cloud.publish(pc2) ;
}

void LiveSLAMWrapper::BALoop()
{
    ros::Rate BARate(2000) ;
    list<ImageMeasurement>::iterator iterImage ;
    std::list<visensor_node::visensor_imu>::iterator iterIMU ;
    cv::Mat image0 ;
    cv::Mat image1 ;
    cv::Mat gradientMapForDebug(height, width, CV_8UC3) ;
    sensor_msgs::Image msg;
    double t ;

    while ( nh.ok() )
    {
        monoOdometry->frameInfoList_mtx.lock();
        int ttt = (monoOdometry->frameInfoListTail - monoOdometry->frameInfoListHead);
        if ( ttt < 0 ){
            ttt += frameInfoListSize ;
        }
        //printf("[BA thread] sz=%d\n", ttt ) ;
        if ( ttt < 1 ){
            monoOdometry->frameInfoList_mtx.unlock();
            BARate.sleep() ;
            continue ;
        }
        for ( int sz ; ; )
        {
            monoOdometry->frameInfoListHead++ ;
            if ( monoOdometry->frameInfoListHead >= frameInfoListSize ){
                monoOdometry->frameInfoListHead -= frameInfoListSize ;
            }
            sz = monoOdometry->frameInfoListTail - monoOdometry->frameInfoListHead ;
            if ( sz == 0 ){
                break ;
            }
            if ( monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].keyFrameFlag ){
                break ;
            }
        }
        ros::Time imageTimeStamp = monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].t ;
        monoOdometry->frameInfoList_mtx.unlock();

        //Pop out the image list
        image1_queue_mtx.lock();
        iterImage = image1Buf.begin() ;
        while ( iterImage->t < imageTimeStamp ){
            iterImage = image1Buf.erase( iterImage ) ;
        }
        image1 = iterImage->image.clone();
        image1_queue_mtx.unlock();

        image0_queue_mtx.lock();
        iterImage = image0Buf.begin() ;
        while ( iterImage->t < imageTimeStamp ){
            iterImage = image0Buf.erase( iterImage ) ;
        }
        image0 = iterImage->image.clone();
        image0_queue_mtx.unlock();

        imu_queue_mtx.lock();
        iterIMU = imuQueue.begin() ;
        Vector3d linear_acceleration;
        Vector3d angular_velocity;

        //std::cout << "imageTime=" << imageTimeStamp << std::endl;
        while ( iterIMU->header.stamp < imageTimeStamp )
        {
            linear_acceleration(0) = iterIMU->linear_acceleration.x;
            linear_acceleration(1) = iterIMU->linear_acceleration.y;
            linear_acceleration(2) = iterIMU->linear_acceleration.z;
            angular_velocity(0) = iterIMU->angular_velocity.x;
            angular_velocity(1) = iterIMU->angular_velocity.y;
            angular_velocity(2) = iterIMU->angular_velocity.z;

//            to_pub_info.x = angular_velocity(0)*180/PI ;
//            to_pub_info.y = angular_velocity(1)*180/PI ;
//            to_pub_info.z = angular_velocity(2)*180/PI ;
//            monoOdometry->pub_angular_velocity.publish( to_pub_info ) ;

//            outFile << to_pub_info.x << " "
//                    << to_pub_info.y << " "
//                    << to_pub_info.z << "\n";


            double pre_t = iterIMU->header.stamp.toSec();
            iterIMU = imuQueue.erase(iterIMU);
            double next_t = iterIMU->header.stamp.toSec();
            double dt = next_t - pre_t ;

//            std::cout << linear_acceleration.transpose() << std::endl ;
//            std::cout << angular_velocity.transpose() << std::endl ;
            monoOdometry->processIMU( dt, linear_acceleration, angular_velocity );
        }
        imu_queue_mtx.unlock();

        //propagate the last frame info to the current frame
        Frame* lastFrame = monoOdometry->slidingWindow[monoOdometry->tail].get();
        float dt = lastFrame->timeIntegral;

        Vector3d T_bk1_2_b0 = lastFrame->T_bk_2_b0 - 0.5 * gravity_b0 * dt *dt
                + lastFrame->R_bk_2_b0*(lastFrame->v_bk * dt  + lastFrame->alpha_c_k);
        Vector3d v_bk1 = lastFrame->R_k1_k.transpose() *
                (lastFrame->v_bk - lastFrame->R_bk_2_b0.transpose() * gravity_b0 * dt
                 + lastFrame->beta_c_k);
        Matrix3d R_bk1_2_b0 = lastFrame->R_bk_2_b0 * lastFrame->R_k1_k;

        monoOdometry->insertFrame(imageSeqNumber, image0, imageTimeStamp, R_bk1_2_b0, T_bk1_2_b0, v_bk1);
        Frame* currentFrame = monoOdometry->slidingWindow[monoOdometry->tail].get();
        Frame* keyFrame = monoOdometry->currentKeyFrame.get();
        if ( monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].keyFrameFlag )
        {
            //prepare key frame
            cv::Mat disparity, depth ;
#if CV_MAJOR_VERSION == 2
            monoOdometry->bm_(image0, image1, disparity, CV_32F);
#elif CV_MAJOR_VERSION == 3
            disparity.create(image0.size(),CV_32F);
            monoOdometry->bm_->compute(image0, image1, disparity);
#endif
            calculateDepthImage(disparity, depth, 0.11, fx );
            currentFrame->setDepthFromGroundTruth( (float*)depth.data ) ;

#ifdef PRINT_DEBUG_INFO
            //pub debugMap
            cv::cvtColor(image0, gradientMapForDebug, CV_GRAY2BGR);
            monoOdometry->generateDubugMap(currentFrame, gradientMapForDebug);
            msg.header.stamp = imageTimeStamp;
            sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::BGR8, height,
                                   width, width*3, gradientMapForDebug.data );
            monoOdometry->pub_gradientMapForDebug.publish(msg) ;
#endif
            int preKeyFrameID = monoOdometry->currentKeyFrame->id() ;

            //set key frame
            monoOdometry->currentKeyFrame = monoOdometry->slidingWindow[monoOdometry->tail] ;
            monoOdometry->currentKeyFrame->keyFrameFlag = true ;
            monoOdometry->currentKeyFrame->cameraLinkList.clear() ;

            //reset the initial guess
            monoOdometry->RefToFrame = Sophus::SE3() ;

            //update tracking reference
            monoOdometry->updateTrackingReference();

#ifdef PUB_POINT_CLOUD
            pubPointCloud(valid_num, imageTimeStamp, R_vi_2_odometry);
#endif
            //unlock dense tracking
            monoOdometry->tracking_mtx.lock();
            monoOdometry->lock_densetracking = false;
            monoOdometry->tracking_mtx.unlock();

            if ( (imageTimeStamp - lastLoopClorsureTime).toSec() > 0.18 )
            {
                //add possible loop closure link
                t = (double)cvGetTickCount()  ;
                monoOdometry->setReprojectionListRelateToLastestKeyFrame( monoOdometry->head, preKeyFrameID,
                                                                          monoOdometry->slidingWindow[monoOdometry->tail].get(),
                                                                          R_i_2_c, T_i_2_c ) ;
                ROS_WARN("loop closure link cost time: %f", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000) );
                t = (double)cvGetTickCount()  ;
                lastLoopClorsureTime = imageTimeStamp ;
            }
        }

        if ( monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].trust )
        {
//            cout << "insert camera link" << endl ;
//            cout << monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].T_k_2_c << endl ;

            monoOdometry->insertCameraLink(keyFrame, currentFrame,
                          monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].R_k_2_c,
                          monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].T_k_2_c,
                          monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].lastestATA );
        }

        int control_flag = 0 ;
        Vector3d preBAt = currentFrame->T_bk_2_b0 ;


//        cout << "[-BA]current Position: " << currentFrame->T_bk_2_b0.transpose() << endl;
//        cout << "[-BA]current Velocity: " << currentFrame->v_bk.transpose() << endl;

        //BA
        t = (double)cvGetTickCount()  ;
        monoOdometry->BA();
        printf("BA cost time: %f\n", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000) );
        t = (double)cvGetTickCount()  ;

        //cout << "[BA-]current Position: " << currentFrame->T_bk_2_b0.transpose() << endl;
        //cout << "[BA-]current Velocity: " << currentFrame->v_bk.transpose() << endl;

        if ( (currentFrame->T_bk_2_b0 - preBAt ).norm() > 0.1 ){
            control_flag = 1 ; //loop_closure or other sudden position change case
        }
        if ( monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].trust == false ){
            control_flag = 2 ; //only IMU link, dense tracking fails
        }

#ifdef PUB_GRAPH
        pubCameraLink();
#endif

        //marginalziation
        monoOdometry->twoWayMarginalize();
        monoOdometry->setNewMarginalzationFlag();

//        R_vi_2_odometry.setIdentity() ;
//        if ( onUAV )
//        {
//            pubOdometry(monoOdometry->slidingWindow[monoOdometry->tail]->T_bk_2_b0,
//                    monoOdometry->slidingWindow[monoOdometry->tail]->v_bk,
//                    monoOdometry->slidingWindow[monoOdometry->tail]->R_bk_2_b0,
//                    monoOdometry->pub_odometry, monoOdometry->pub_pose,
//                    control_flag, R_vi_2_odometry,
//                    monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].keyFrameFlag, imageTimeStamp );
//        }
//        else
//        {
//            pubOdometry(monoOdometry->slidingWindow[monoOdometry->tail]->T_bk_2_b0,
//                    monoOdometry->slidingWindow[monoOdometry->tail]->v_bk,
//                    monoOdometry->slidingWindow[monoOdometry->tail]->R_bk_2_b0,
//                    monoOdometry->pub_odometry, monoOdometry->pub_pose,
//                    control_flag, Eigen::Matrix3d::Identity(),
//                    monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].keyFrameFlag, imageTimeStamp );
//        }

#ifdef PRINT_DEBUG_INFO
        int colorFlag = 0 ;
        colorFlag = monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].keyFrameFlag ;
        if (  monoOdometry->frameInfoList[monoOdometry->frameInfoListHead].trust == false ){
            colorFlag = 2 ;
        }
        if ( onUAV ){
            pubPath(monoOdometry->slidingWindow[monoOdometry->tail]->T_bk_2_b0,
                    colorFlag,
                    monoOdometry->path_line, monoOdometry->pub_path, R_vi_2_odometry);
        }
        else{
            pubPath(monoOdometry->slidingWindow[monoOdometry->tail]->T_bk_2_b0,
                    colorFlag,
                    monoOdometry->path_line, monoOdometry->pub_path, Eigen::Matrix3d::Identity() );
        }
#endif

#ifdef  PUB_TF
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        Vector3d t_translation = R_vi_2_odometry * monoOdometry->slidingWindow[monoOdometry->tail]->T_bk_2_b0 ;
        Quaterniond t_q(monoOdometry->slidingWindow[monoOdometry->tail]->R_bk_2_b0) ;
        transform.setOrigin(tf::Vector3(t_translation(0),
                                t_translation(1),
                                t_translation(2)) );
        tf::Quaternion q;
        Quaterniond tt_q(R_vi_2_odometry * monoOdometry->slidingWindow[monoOdometry->tail]->R_bk_2_b0 * R_vi_2_odometry.transpose());
        q.setW(tt_q.w());
        q.setX(tt_q.x());
        q.setY(tt_q.y());
        q.setZ(tt_q.z());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "densebody"));
#endif
        logCameraPose() ;

//        int preIndex = monoOdometry->tail - 1 ;
//        if ( preIndex < 0 ){
//            preIndex += slidingWindowSize ;
//        }
//        Vector3d tt_dist = (monoOdometry->slidingWindow[monoOdometry->tail]->T_bk_2_b0 -
//                monoOdometry->slidingWindow[preIndex]->T_bk_2_b0) ;
//        Matrix3d tt_rotate = monoOdometry->slidingWindow[monoOdometry->tail]->R_bk_2_b0.transpose() *
//                monoOdometry->slidingWindow[preIndex]->R_bk_2_b0 ;
//        Quaterniond tt_q(tt_rotate) ;

//        to_pub_info.x = monoOdometry->slidingWindow[monoOdometry->tail]->v_bk(0) ;
//        to_pub_info.y = monoOdometry->slidingWindow[monoOdometry->tail]->v_bk(1) ;
//        to_pub_info.z = monoOdometry->slidingWindow[monoOdometry->tail]->v_bk(2) ;
//        monoOdometry->pub_linear_velocity.publish(to_pub_info) ;


    }
}

void LiveSLAMWrapper::Loop()
{
    std::list<visensor_node::visensor_imu>::reverse_iterator reverse_iterImu ;
    std::list<ImageMeasurement>::iterator  pIter ;
    ros::Time imageTimeStamp ;
    cv::Mat   image0 ;
    cv::Mat   image1 ;
    ros::Rate r(1000.0);
    while ( nh.ok() )
    {
        monoOdometry->tracking_mtx.lock();
        bool tmpFlag = monoOdometry->lock_densetracking ;
        monoOdometry->tracking_mtx.unlock();
        //printf("tmpFlag = %d\n", tmpFlag ) ;
        if ( tmpFlag == true ){
            r.sleep() ;
            continue ;
        }
        image0_queue_mtx.lock();
        image1_queue_mtx.lock();
        imu_queue_mtx.lock();
        pIter = pImage0Iter ;
        pIter++ ;
        if ( pIter == image0Buf.end() ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        imageTimeStamp = pIter->t ;

        pIter = pImage1Iter ;
        pIter++ ;
        if ( pIter == image1Buf.end() ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        if ( image1Buf.rbegin()->t < imageTimeStamp )
        {
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        reverse_iterImu = imuQueue.rbegin() ;
        //        printf("%d %d\n", imuQueue.size() < 10, reverse_iterImu->header.stamp <= imageTimeStamp ) ;
        if ( imuQueue.size() < 1 || reverse_iterImu->header.stamp < imageTimeStamp ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        imu_queue_mtx.unlock();

        while ( pImage1Iter->t < imageTimeStamp ){
            pImage1Iter++ ;
        }
        pImage0Iter++ ;
        //std::cout << imageTimeStamp.toNSec() << "\n" ;
        //std::cout << "[dt-image] " << imageTimeStamp << std::endl ;
        //std::cout << "[dt-imu] " << reverse_iterImu->header.stamp << " " << imuQueue.size() << std::endl ;
        ros::Time preTime = imageTimeStamp ;
        //pImage1Iter++ ;

        image1 = pImage1Iter->image.clone();
        image0 = pImage0Iter->image.clone();
        image1_queue_mtx.unlock();
        image0_queue_mtx.unlock();

        imu_queue_mtx.lock();
        Quaterniond q, dq ;
        q.setIdentity() ;
        while ( currentIMU_iter->header.stamp < imageTimeStamp )
        {
            double pre_t = currentIMU_iter->header.stamp.toSec();
            currentIMU_iter++ ;
            double next_t = currentIMU_iter->header.stamp.toSec();
            double dt = next_t - pre_t ;

            //prediction for dense tracking
            dq.x() = currentIMU_iter->angular_velocity.x*dt*0.5 ;
            dq.y() = currentIMU_iter->angular_velocity.y*dt*0.5 ;
            dq.z() = currentIMU_iter->angular_velocity.z*dt*0.5 ;
            dq.w() =  sqrt( 1 - SQ(dq.x()) * SQ(dq.y()) * SQ(dq.z()) ) ;
            q = (q * dq).normalized();
        }
        imu_queue_mtx.unlock();

        // process image
        //Util::displayImage("MyVideo", image.data);
        Matrix3d deltaR(q) ;

        //puts("444") ;

        //        cv::imshow("img0", image0 ) ;
        //        cv::imshow("img1", image1 ) ;
        //        cv::waitKey(1) ;

        ++imageSeqNumber;
        assert(image0.elemSize() == 1);
        assert(image1.elemSize() == 1);
        assert(fx != 0 || fy != 0);
//        if(!isInitialized)
//        {
//            monoOdometry->insertFrame(imageSeqNumber, image1, imageTimeStamp,
//                                      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() );
//            cv::Mat disparity, depth ;
//            monoOdometry->bm_(image1, image0, disparity, CV_32F);
//            calculateDepthImage(disparity, depth, 0.11, fx );
//            monoOdometry->currentKeyFrame = monoOdometry->slidingWindow[0] ;
//            monoOdometry->currentKeyFrame->setDepthFromGroundTruth( (float*)depth.data ) ;

//            monoOdometry->currentKeyFrame->keyFrameFlag = true ;
//            monoOdometry->currentKeyFrame->cameraLinkList.clear() ;
//            monoOdometry->RefToFrame = Sophus::SE3() ;
//            isInitialized = true;
//        }
//        else if(isInitialized && monoOdometry != nullptr)
//        {
            monoOdometry->trackFrame(image0, imageSeqNumber, imageTimeStamp, deltaR, R_i_2_c, T_i_2_c );
//        }

	}
}

void LiveSLAMWrapper::logCameraPose()
{
    if ( log_cameraPoseID%3 == 0 )
    {
        int k = monoOdometry->tail ;
        Quaterniond q(monoOdometry->R[k]) ;
        outFile << " " << log_cameraPoseID << " "
                << monoOdometry->T[k](0) << " "
                << monoOdometry->T[k](1) << " "
                << monoOdometry->T[k](2) << " "
                << q.x() << " "
                << q.y() << " "
                << q.z() << " "
                << q.w() << "\n" ;
        char tmp[128] ;
        sprintf(tmp, "/home/ygling2008/new_vins_data/%04d.png", log_cameraPoseID ) ;
        cv::imwrite(tmp, lastestImg) ;
    }

    log_cameraPoseID++ ;
//    Sophus::Quaterniond quat = camToWorld.unit_quaternion();
//    Eigen::Vector3d trans = camToWorld.translation();

//	char buffer[1000];
//	int num = snprintf(buffer, 1000, "%f %f %f %f %f %f %f %f\n",
//			time,
//			trans[0],
//			trans[1],
//			trans[2],
//			quat.x(),
//			quat.y(),
//			quat.z(),
//			quat.w());

//	if(outFile == 0)
//		outFile = new std::ofstream(outFileName.c_str());
//	outFile->write(buffer,num);
//	outFile->flush();
}

}

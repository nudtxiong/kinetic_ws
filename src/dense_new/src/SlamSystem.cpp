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

#include "SlamSystem.h"
#include "util/settings.h"
#include "DataStructures/Frame.h"
#include "Tracking/SE3Tracker.h"
#include "Tracking/TrackingReference.h"
#include "LiveSLAMWrapper.h"
#include "util/globalFuncs.h"
#include "IOWrapper/ImageDisplay.h"
#include "DataStructures/FrameMemory.h"
#include <deque>
#include "sensor_msgs/PointCloud2.h"

// for mkdir
#include <sys/types.h>
#include <sys/stat.h>
#include "opencv2/opencv.hpp"

using namespace lsd_slam;
using namespace Eigen;

SlamSystem::SlamSystem(int w, int h, Eigen::Matrix3f K, ros::NodeHandle &n)
{
//    if(w%16 != 0 || h%16!=0)
//    {
//        printf("image dimensions must be multiples of 16! Please crop your images / video accordingly.\n");
//        assert(false);
//    }

	this->width = w;
	this->height = h;
	this->K = K;
    this->nh = n ;
	trackingIsGood = true;
    lock_densetracking = false ;
    currentKeyFrame =  nullptr;
	createNewKeyFrame = false;

	tracker = new SE3Tracker(w,h,K);
    trackingReference = new TrackingReference();

    trackerConstraint = new SE3Tracker(w,h,K);
    trackingReferenceConstraint = new TrackingReference();

    int maxDisparity = 64 ;
    int blockSize = 21 ;
#if CV_MAJOR_VERSION == 2
     bm_ = cv::StereoBM( cv::StereoBM::BASIC_PRESET, maxDisparity, blockSize) ;
#elif CV_MAJOR_VERSION == 3
     bm_ = cv::StereoBM::create(maxDisparity, blockSize);
     bm_->setPreFilterType(bm_->PREFILTER_XSOBEL);

     //bm_->setTextureThreshold(0);
#endif
    initRosPub() ;

    head = 0 ;
    tail = -1 ;
    numOfState = 0 ;
    frameInfoListHead = frameInfoListTail = 0 ;

	lastTrackingClosenessScore = 0;
	msTrackFrame = msOptimizationIteration = msFindConstraintsItaration = msFindReferences = 0;
	nTrackFrame = nOptimizationIteration = nFindConstraintsItaration = nFindReferences = 0;
	nAvgTrackFrame = nAvgOptimizationIteration = nAvgFindConstraintsItaration = nAvgFindReferences = 0;
	gettimeofday(&lastHzUpdate, NULL);

}

SlamSystem::~SlamSystem()
{
	delete trackingReference;
	delete tracker;

    delete trackingReferenceConstraint ;
    delete trackerConstraint ;

    for( int i = 0 ; i < slidingWindowSize ; i++ ){
        slidingWindow[i].reset();
    }
    //lastTrackedFrame.reset();
    currentKeyFrame.reset();
	FrameMemory::getInstance().releaseBuffes();
	Util::closeAllWindows();
}

void SlamSystem::debugDisplayDepthMap()
{
//	double scale = 1;
//	if(currentKeyFrame != 0 && currentKeyFrame != 0)
//		scale = currentKeyFrame->getScaledCamToWorld().scale();
//	// debug plot depthmap
//	char buf1[200];

//    snprintf(buf1,200,"dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/; usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
//			100*currentKeyFrame->numPoints/(float)(width*height),
//			100*tracking_lastGoodPerBad,
//			scale,
//			tracking_lastResidual,
//            100*tracking_lastUsage );


//	if(onSceenInfoDisplay)
//        printMessageOnCVImage(map->debugImageDepth, buf1 );
//	if (displayDepthMap)
//		Util::displayImage( "DebugWindow DEPTH", map->debugImageDepth, false );

//	int pressedKey = Util::waitKey(1);
//	handleKey(pressedKey);
}

void SlamSystem::initRosPub()
{
    pub_path = nh.advertise<visualization_msgs::Marker>("/denseVO/path", 1000);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/denseVO/cloud", 1000);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/denseVO/pose", 1000);
    pub_resudualMap = nh.advertise<sensor_msgs::Image>("denseVO/residualMap", 100 );
    pub_reprojectMap = nh.advertise<sensor_msgs::Image>("denseVO/reprojectMap", 100 );
    pub_gradientMapForDebug = nh.advertise<sensor_msgs::Image>("denseVO/debugMap", 100 );
    pub_denseTracking = nh.advertise<geometry_msgs::Vector3>("denseVO/dt", 100);
    pub_angular_velocity = nh.advertise<geometry_msgs::Vector3>("denseVO/angular_velocity", 100);;
    pub_linear_velocity = nh.advertise<geometry_msgs::Vector3>("denseVO/linear_velocity", 100);;

    path_line.header.frame_id    = "world";
    path_line.header.stamp       = ros::Time::now();
    path_line.ns                 = "dense_vo";
    path_line.action             = visualization_msgs::Marker::ADD;
    path_line.pose.orientation.w = 1.0;
    path_line.type               = visualization_msgs::Marker::LINE_STRIP;
    path_line.scale.x            = 0.01 ;
    path_line.color.a            = 1.0;
    path_line.color.r            = 1.0;
    path_line.id                 = 1;
    path_line.points.push_back( geometry_msgs::Point());
    path_line.colors.push_back( std_msgs::ColorRGBA() );
    pub_path.publish(path_line);
}

void SlamSystem::generateDubugMap(Frame* currentFrame, cv::Mat& gradientMapForDebug )
{
    int n = currentFrame->height() ;
    int m = currentFrame->width() ;
    const float* pIdepth = currentFrame->idepth(0) ;
    for ( int i = 0 ; i < n ; i++ )
    {
        for( int j = 0 ; j < m ; j++ )
        {
            if (  *pIdepth > 0 ){
                gradientMapForDebug.at<cv::Vec3b>(i, j)[0] = 0;
                gradientMapForDebug.at<cv::Vec3b>(i, j)[1] = 255;
                gradientMapForDebug.at<cv::Vec3b>(i, j)[2] = 0;
            }
            pIdepth++ ;
        }
    }
}

void SlamSystem::setDepthInit(cv::Mat img0, cv::Mat img1, double timeStamp, int id)
{
    cv::Mat disparity, depth ;
#if CV_MAJOR_VERSION == 2
    bm_(img1, img0, disparity, CV_32F);
#elif CV_MAJOR_VERSION == 3
    disparity.create(img1.size(),CV_32F);
    bm_->compute(img1, img0, disparity);
#endif

    calculateDepthImage(disparity, depth, 0.11, K(0, 0) );

    currentKeyFrame.reset(new Frame(id, width, height, K, timeStamp, img1.data,
                                    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() ));
    Frame* frame = currentKeyFrame.get() ;
    frame->setDepthFromGroundTruth( (float*)depth.data );
    if ( printDebugInfo ){
        cv::cvtColor(img1, gradientMapForDebug, CV_GRAY2BGR ) ;
        generateDubugMap(frame, gradientMapForDebug ) ;
        sensor_msgs::Image msg;
        msg.header.stamp = ros::Time() ;
        sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::BGR8, height, width, width*3,
                               gradientMapForDebug.data );
        pub_gradientMapForDebug.publish(msg) ;
    }
    frame->R_bk_2_b0.setIdentity() ;
    frame->T_bk_2_b0.setZero() ;
    frame->v_bk.setZero() ;
    RefToFrame = SE3();
    slidingWindow[tail] = currentKeyFrame ;
//    std::cout << RefToFrame.rotationMatrix() << std::endl ;
//    std::cout << RefToFrame.translation() << std::endl ;
}

void SlamSystem::copyStateData( int preStateID )
{
//  //copy the lastest data to the second lastest

//  //1. basic data
//  slidingWindow[preStateID]->R_bk_2_b0 = slidingWindow[tail]->R_bk_2_b0;
//  slidingWindow[preStateID]->T_bk_2_b0 = slidingWindow[tail]->T_bk_2_b0;
//  slidingWindow[preStateID]->v_bk = slidingWindow[tail]->v_bk;

//  slidingWindow[preStateID]->alpha_c_k = slidingWindow[tail]->alpha_c_k;
//  slidingWindow[preStateID]->beta_c_k = slidingWindow[tail]->beta_c_k;
//  slidingWindow[preStateID]->R_k1_k = slidingWindow[tail]->R_k1_k;
//  slidingWindow[preStateID]->P_k = slidingWindow[tail]->P_k;
//  slidingWindow[preStateID]->timeIntegral = slidingWindow[tail]->timeIntegral;

//  slidingWindow[preStateID]->keyFrameFlag = slidingWindow[tail]->keyFrameFlag;
//  slidingWindow[preStateID]->imuLinkFlag = slidingWindow[tail]->imuLinkFlag;
//  slidingWindow[preStateID]->timestamp = slidingWindow[tail]->timestamp;

//  int n = height;
//  int m = width;
//  for (int i = 0; i < maxPyramidLevel; i++)
//  {
//    memcpy(slidingWindow[preStateID]->intensity[i], slidingWindow[tail]->intensity[i], n*m*sizeof(unsigned char));
//    if (slidingWindow[tail]->keyFrameFlag )
//    {
//      memcpy(slidingWindow[preStateID]->depthImage[i], slidingWindow[tail]->depthImage[i], n*m*sizeof(float));
//      memcpy(slidingWindow[preStateID]->gradientX[i], slidingWindow[tail]->gradientX[i], n*m*sizeof(double));
//      memcpy(slidingWindow[preStateID]->gradientY[i], slidingWindow[tail]->gradientY[i], n*m*sizeof(double));

//      slidingWindow[preStateID]->totalNumOfValidPixels[i] = slidingWindow[tail]->totalNumOfValidPixels[i];

//      int sz = slidingWindow[tail]->pixelInfo[i].Aij.size();
//      slidingWindow[preStateID]->pixelInfo[i].Aij.clear();
//      slidingWindow[preStateID]->pixelInfo[i].Aij.resize(sz);
//      slidingWindow[preStateID]->pixelInfo[i].AijTAij.clear();
//      slidingWindow[preStateID]->pixelInfo[i].AijTAij.resize(sz);
//      for (int j = 0; j < sz; j++)
//      {
//        slidingWindow[preStateID]->pixelInfo[i].Aij[j] = slidingWindow[tail]->pixelInfo[i].Aij[j];
//        slidingWindow[preStateID]->pixelInfo[i].AijTAij[j] = slidingWindow[tail]->pixelInfo[i].AijTAij[j];
//      }
////        slidingWindow[preStateID]->pixelInfo[i].Aij.swap( slidingWindow[tail]->pixelInfo[i].Aij );
////        slidingWindow[preStateID]->pixelInfo[i].AijTAij.swap( slidingWindow[tail]->pixelInfo[i].AijTAij );
//      slidingWindow[preStateID]->pixelInfo[i].piList = slidingWindow[tail]->pixelInfo[i].piList ;
//      slidingWindow[preStateID]->pixelInfo[i].intensity = slidingWindow[tail]->pixelInfo[i].intensity ;
//      slidingWindow[preStateID]->pixelInfo[i].goodPixel = slidingWindow[tail]->pixelInfo[i].goodPixel ;
//    }
//    n >>= 1;
//    m >>= 1;
//  }

  //2. maintain reprojection list
  for (int i = numOfState - 2; i >= 0; i--)
  {
    int k = head + i;
    if (k >= slidingWindowSize){
      k -= slidingWindowSize;
    }
    //delete the old link
    list<int>::iterator iter;
    for (iter = slidingWindow[k]->cameraLinkList.begin(); iter != slidingWindow[k]->cameraLinkList.end();)
    {
      if (*iter == preStateID){
        iter = slidingWindow[k]->cameraLinkList.erase(iter);
      }
      else{
        iter++;
      }
    }

    //rebuild the new link
    for (iter = slidingWindow[k]->cameraLinkList.begin(); iter != slidingWindow[k]->cameraLinkList.end(); iter++)
    {
      if (*iter == tail)
      {
        slidingWindow[k]->cameraLink[preStateID].R_bi_2_bj = slidingWindow[k]->cameraLink[tail].R_bi_2_bj;
        slidingWindow[k]->cameraLink[preStateID].T_bi_2_bj = slidingWindow[k]->cameraLink[tail].T_bi_2_bj;
        slidingWindow[k]->cameraLink[preStateID].P_inv = slidingWindow[k]->cameraLink[tail].P_inv;
        *iter = preStateID;
      }
    }
  }

  numOfState--;
  //swap the pointer
  //ROS_WARN("preStateID = %d", preStateID ) ;
  slidingWindow[preStateID].reset()  ;
  //ROS_WARN("preStateID = %d", preStateID ) ;
  slidingWindow[preStateID] = slidingWindow[tail] ;
  tail = preStateID;

  preStateID = tail - 1;
  if (preStateID < 0){
    preStateID += slidingWindowSize;
  }
  slidingWindow[preStateID]->imuLinkFlag = false;
}


void SlamSystem::twoWayMarginalize()
{
    if (twoWayMarginalizatonFlag == false)
    {
      //marginalized the oldest frame
      if (numOfState == slidingWindowSize)
      {
        vector<Vector3d>T(slidingWindowSize);
        vector<Vector3d>vel(slidingWindowSize);
        vector<Matrix3d>R(slidingWindowSize);
        for (int i = 0; i < slidingWindowSize; i++)
        {
            if ( slidingWindow[i] != nullptr )
            {
                R[i] = slidingWindow[i]->R_bk_2_b0;
                T[i] = slidingWindow[i]->T_bk_2_b0;
                vel[i] = slidingWindow[i]->v_bk;
            }
        }

        margin.size++;
        //1. IMU constraints
        if ( slidingWindow[head]->imuLinkFlag )
        {
          int k = head ;
          int k1 = k + 1;
          if (k1 >= slidingWindowSize){
            k1 -= slidingWindowSize;
          }
          VectorXd residualIMU = math.ResidualImu(T[k], vel[k], R[k],
                                                  T[k1], vel[k1], R[k1],
                                                  gravity_b0, slidingWindow[k]->timeIntegral,
                                                  slidingWindow[k]->alpha_c_k, slidingWindow[k]->beta_c_k, slidingWindow[k]->R_k1_k);
          MatrixXd H_k1_2_k = math.JacobianImu(T[k], vel[k], R[k],
                                               T[k1], vel[k1], R[k1],
                                               gravity_b0, slidingWindow[k]->timeIntegral);
          MatrixXd H_k1_2_k_T = H_k1_2_k.transpose();

          H_k1_2_k_T *= slidingWindow[k]->P_k.inverse();
          H_k1_2_k_T = H_k1_2_k.transpose();
          H_k1_2_k_T *= slidingWindow[k]->P_k.inverse();

          margin.Ap.block(0, 0, 18, 18) += H_k1_2_k_T  *  H_k1_2_k;
          margin.bp.segment(0, 18) += H_k1_2_k_T * residualIMU;
        }

        //2. camera constraints
        int currentStateID = head;
        MatrixXd H_i_2_j(9, 18);
        MatrixXd H_i_2_j_T;
        VectorXd residualCamera(9);
        MatrixXd tmpP_inv(9, 9);
        MatrixXd tmpHTH;
        VectorXd tmpHTb;
        list<int>::iterator iter = slidingWindow[currentStateID]->cameraLinkList.begin();
        for (; iter != slidingWindow[currentStateID]->cameraLinkList.end(); iter++)
        {
          int linkID = *iter;

          H_i_2_j = math.JacobianDenseTracking(T[currentStateID], R[currentStateID], T[linkID], R[linkID] ) ;
          residualCamera = math.ResidualDenseTracking(T[currentStateID], R[currentStateID], T[linkID], R[linkID],
                                                      slidingWindow[currentStateID]->cameraLink[linkID].T_bi_2_bj,
                                                      slidingWindow[currentStateID]->cameraLink[linkID].R_bi_2_bj ) ;
          tmpP_inv.setZero();
          tmpP_inv.block(0, 0, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(0, 0, 3, 3) ;
          tmpP_inv.block(0, 6, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(0, 3, 3, 3) ;
          tmpP_inv.block(6, 0, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(3, 0, 3, 3) ;
          tmpP_inv.block(6, 6, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(3, 3, 3, 3) ;
          double r_v = residualCamera.segment(0, 3).norm() ;
          if ( r_v > huber_r_v ){
            tmpP_inv *= huber_r_v/r_v ;
          }
          double r_w = residualCamera.segment(6, 3).norm() ;
          if ( r_w > huber_r_w ){
            tmpP_inv *= huber_r_w/r_w ;
          }

          H_i_2_j_T = H_i_2_j.transpose();
          H_i_2_j_T *= tmpP_inv;
          tmpHTH = H_i_2_j_T  *  H_i_2_j;
          tmpHTb = H_i_2_j_T * residualCamera;

          int currentStateIDIndex = 0 ;
          int linkIDIndex = linkID - head;
          if (linkIDIndex < 0){
            linkIDIndex += slidingWindowSize;
          }

          margin.Ap.block(currentStateIDIndex * 9, currentStateIDIndex * 9, 9, 9) += tmpHTH.block(0, 0, 9, 9);
          margin.Ap.block(currentStateIDIndex * 9, linkIDIndex * 9, 9, 9) += tmpHTH.block(0, 9, 9, 9);
          margin.Ap.block(linkIDIndex * 9, currentStateIDIndex * 9, 9, 9) += tmpHTH.block(9, 0, 9, 9);
          margin.Ap.block(linkIDIndex * 9, linkIDIndex * 9, 9, 9) += tmpHTH.block(9, 9, 9, 9);

          margin.bp.segment(currentStateIDIndex * 9, 9) += tmpHTb.segment(0, 9);
          margin.bp.segment(linkIDIndex * 9, 9) += tmpHTb.segment(9, 9);
        }

        //3. marginalization
        margin.popEndState();

        //pop the oldest state
        head++;
        if (head >= slidingWindowSize){
          head -= slidingWindowSize;
        }
        numOfState--;
      }
      else
      {
        margin.size++;
      }
    }
    else
    {
      //marginalized the second newest frame
      vector<Vector3d>T(slidingWindowSize);
      vector<Vector3d>vel(slidingWindowSize);
      vector<Matrix3d>R(slidingWindowSize);
      for (int i = 0; i < slidingWindowSize; i++)
      {
          if ( slidingWindow[i] != nullptr )
          {
              R[i] = slidingWindow[i]->R_bk_2_b0;
              T[i] = slidingWindow[i]->T_bk_2_b0;
              vel[i] = slidingWindow[i]->v_bk;
          }
      }

      margin.size++;
      int preStateID = tail - 1;
      if (preStateID < 0){
        preStateID += slidingWindowSize;
      }

      MatrixXd tmpHTH;
      VectorXd tmpHTb;

      MatrixXd H_k1_2_k(9, 18);
      MatrixXd H_k1_2_k_T;
      VectorXd residualIMU(9);

      MatrixXd H_i_2_j(9, 18);
      MatrixXd H_i_2_j_T;
      VectorXd residualCamera(9);
      MatrixXd tmpP_inv(9, 9);
      //1.  IMU constrains
      for (int i = numOfState - 2; i >= numOfState - 3; i--)
      {
        int k = head + i;
        if (k >= slidingWindowSize){
          k -= slidingWindowSize;
        }
        if (slidingWindow[k]->imuLinkFlag == false){
          continue;
        }
        int k1 = k + 1;
        if (k1 >= slidingWindowSize){
          k1 -= slidingWindowSize;
        }
        residualIMU = math.ResidualImu(T[k], vel[k], R[k],
                                                T[k1], vel[k1], R[k1],
                                                gravity_b0, slidingWindow[k]->timeIntegral,
                                                slidingWindow[k]->alpha_c_k, slidingWindow[k]->beta_c_k, slidingWindow[k]->R_k1_k);
        H_k1_2_k = math.JacobianImu(T[k], vel[k], R[k],
                                             T[k1], vel[k1], R[k1],
                                             gravity_b0, slidingWindow[k]->timeIntegral);
        H_k1_2_k_T = H_k1_2_k.transpose();
        H_k1_2_k_T *= slidingWindow[k]->P_k.inverse();

        margin.Ap.block(i * 9, i * 9, 18, 18) += H_k1_2_k_T  *  H_k1_2_k;
        margin.bp.segment(i * 9, 18) += H_k1_2_k_T * residualIMU;
      }

      //2. camera constrains
      for (int i = numOfState-3; i >= 0; i-- )
      {
        int currentStateID = head + i;
        if (currentStateID >= slidingWindowSize){
          currentStateID -= slidingWindowSize;
        }
        if (slidingWindow[currentStateID]->keyFrameFlag == false){
          continue;
        }

        list<int>::iterator iter = slidingWindow[currentStateID]->cameraLinkList.begin();
        for (; iter != slidingWindow[currentStateID]->cameraLinkList.end(); iter++)
        {
          int linkID = *iter;

          if (linkID != preStateID){
            continue;
          }
          H_i_2_j = math.JacobianDenseTracking(T[currentStateID], R[currentStateID], T[linkID], R[linkID] ) ;
          residualCamera = math.ResidualDenseTracking(T[currentStateID], R[currentStateID], T[linkID], R[linkID],
                                                      slidingWindow[currentStateID]->cameraLink[linkID].T_bi_2_bj,
                                                      slidingWindow[currentStateID]->cameraLink[linkID].R_bi_2_bj ) ;
          tmpP_inv.setZero();
          tmpP_inv.block(0, 0, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(0, 0, 3, 3) ;
          tmpP_inv.block(0, 6, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(0, 3, 3, 3) ;
          tmpP_inv.block(6, 0, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(3, 0, 3, 3) ;
          tmpP_inv.block(6, 6, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(3, 3, 3, 3) ;
          double r_v = residualCamera.segment(0, 3).norm() ;
          if ( r_v > huber_r_v ){
            tmpP_inv *= huber_r_v/r_v ;
          }
          double r_w = residualCamera.segment(6, 3).norm() ;
          if ( r_w > huber_r_w ){
            tmpP_inv *= huber_r_w/r_w ;
          }

          H_i_2_j_T = H_i_2_j.transpose();
          H_i_2_j_T *= tmpP_inv;

          tmpHTH = H_i_2_j_T  *  H_i_2_j;
          tmpHTb = H_i_2_j_T * residualCamera;

          int currentStateIDIndex = currentStateID - head;
          if (currentStateIDIndex < 0){
            currentStateIDIndex += slidingWindowSize;
          }
          int linkIDIndex = linkID - head;
          if (linkIDIndex < 0){
            linkIDIndex += slidingWindowSize;
          }

          margin.Ap.block(currentStateIDIndex * 9, currentStateIDIndex * 9, 9, 9) += tmpHTH.block(0, 0, 9, 9);
          margin.Ap.block(currentStateIDIndex * 9, linkIDIndex * 9, 9, 9) += tmpHTH.block(0, 9, 9, 9);
          margin.Ap.block(linkIDIndex * 9, currentStateIDIndex * 9, 9, 9) += tmpHTH.block(9, 0, 9, 9);
          margin.Ap.block(linkIDIndex * 9, linkIDIndex * 9, 9, 9) += tmpHTH.block(9, 9, 9, 9);

          margin.bp.segment(currentStateIDIndex * 9, 9) += tmpHTb.segment(0, 9);
          margin.bp.segment(linkIDIndex * 9, 9) += tmpHTb.segment(9, 9);
        }
      }


      //3. marginalization
      margin.popFrontState();

      //double t = (double)cvGetTickCount();
      copyStateData( preStateID );
      //printf("copy time: %f\n", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));
    }
}

void SlamSystem::setNewMarginalzationFlag()
{
    if ( slidingWindow[tail]->keyFrameFlag ){
      twoWayMarginalizatonFlag = false;
    }
    else {
      twoWayMarginalizatonFlag = true;
    }
}

void SlamSystem::BA()
{
    R.resize(numOfState);
    T.resize(numOfState);
    vel.resize(numOfState);
    for (int i = 0; i < numOfState ; i++)
    {
        int k = head + i;
        if (k >= slidingWindowSize){
          k -= slidingWindowSize;
        }
        R[k] = slidingWindow[k]->R_bk_2_b0;
        T[k] = slidingWindow[k]->T_bk_2_b0;
        vel[k] = slidingWindow[k]->v_bk;
    }

    int sizeofH = 9 * numOfState;
    MatrixXd HTH(sizeofH, sizeofH);
    VectorXd HTb(sizeofH);
    MatrixXd tmpHTH;
    VectorXd tmpHTb;

    MatrixXd H_k1_2_k(9, 18);
    MatrixXd H_k1_2_k_T;
    VectorXd residualIMU(9);

    MatrixXd H_i_2_j(9, 18);
    MatrixXd H_i_2_j_T;
    VectorXd residualCamera(9);
    MatrixXd tmpP_inv(9, 9);

    for (int iterNum = 0; iterNum < maxIterationBA; iterNum++)
    {
      HTH.setZero();
      HTb.setZero();

      //1. prior constraints
      int m_sz = margin.size;
      VectorXd dx = VectorXd::Zero(STATE_SZ(numOfState - 1));

      if (m_sz != (numOfState - 1)){
        assert("prior matrix goes wrong!!!");
      }

      for (int i = numOfState - 2; i >= 0; i--)
      {
        int k = head + i;
        if (k >= slidingWindowSize){
          k -= slidingWindowSize;
        }
        //dp, dv, dq
        dx.segment(STATE_SZ(i), 3) = T[k] - slidingWindow[k]->T_bk_2_b0;
        dx.segment(STATE_SZ(i) + 3, 3) = vel[k] - slidingWindow[k]->v_bk;
        dx.segment(STATE_SZ(i) + 6, 3) = Quaterniond(slidingWindow[k]->R_bk_2_b0.transpose() * R[k]).vec() * 2.0;
      }
      HTH.block(0, 0, STATE_SZ(m_sz), STATE_SZ(m_sz)) += margin.Ap.block(0, 0, STATE_SZ(m_sz), STATE_SZ(m_sz));
      HTb.segment(0, STATE_SZ(m_sz)) -= margin.Ap.block(0, 0, STATE_SZ(m_sz), STATE_SZ(m_sz))*dx;
      HTb.segment(0, STATE_SZ(m_sz)) -= margin.bp.segment(0, STATE_SZ(m_sz));

      //2. imu constraints
      for (int i = numOfState-2; i >= 0; i-- )
      {
        int k = head + i;
        if (k >= slidingWindowSize){
          k -= slidingWindowSize;
        }
        if ( slidingWindow[k]->imuLinkFlag == false){
          continue;
        }
        int k1 = k + 1;
        if (k1 >= slidingWindowSize){
          k1 -= slidingWindowSize;
        }
        residualIMU = math.ResidualImu(T[k], vel[k], R[k],
                                   T[k1], vel[k1], R[k1],
                                   gravity_b0, slidingWindow[k]->timeIntegral,
                                   slidingWindow[k]->alpha_c_k, slidingWindow[k]->beta_c_k, slidingWindow[k]->R_k1_k);
        H_k1_2_k = math.JacobianImu(T[k], vel[k], R[k],
                                   T[k1], vel[k1], R[k1],
                                   gravity_b0, slidingWindow[k]->timeIntegral);
        H_k1_2_k_T = H_k1_2_k.transpose();
        H_k1_2_k_T *= slidingWindow[k]->P_k.inverse();
        HTH.block(i * 9, i * 9, 18, 18) += H_k1_2_k_T  *  H_k1_2_k;
        HTb.segment(i * 9, 18) -= H_k1_2_k_T * residualIMU;
      }

      //3. camera constraints
      for (int i = 0; i < numOfState; i++)
      {
        int currentStateID = head + i;
        if (currentStateID >= slidingWindowSize){
          currentStateID -= slidingWindowSize;
        }
        if (slidingWindow[currentStateID]->keyFrameFlag == false){
          continue;
        }

        list<int>::iterator iter = slidingWindow[currentStateID]->cameraLinkList.begin();
        for (; iter != slidingWindow[currentStateID]->cameraLinkList.end(); iter++ )
        {
          int linkID = *iter;

          H_i_2_j = math.JacobianDenseTracking(T[currentStateID], R[currentStateID], T[linkID], R[linkID] ) ;
          residualCamera = math.ResidualDenseTracking(T[currentStateID], R[currentStateID], T[linkID], R[linkID],
                                                      slidingWindow[currentStateID]->cameraLink[linkID].T_bi_2_bj,
                                                      slidingWindow[currentStateID]->cameraLink[linkID].R_bi_2_bj ) ;
//          residualCamera.segment(0, 3) = R[linkID].transpose()*(T[currentStateID] - T[linkID]) - slidingWindow[currentStateID]->cameraLink[linkID].T_bi_2_bj;
//          residualCamera.segment(3, 3).setZero();
//          residualCamera.segment(6, 3) = 2.0 * (Quaterniond(slidingWindow[currentStateID]->cameraLink[linkID].R_bi_2_bj.transpose()) * Quaterniond(R[linkID].transpose()*R[currentStateID])).vec();

          tmpP_inv.setZero();
          tmpP_inv.block(0, 0, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(0, 0, 3, 3) ;
          tmpP_inv.block(0, 6, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(0, 3, 3, 3) ;
          tmpP_inv.block(6, 0, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(3, 0, 3, 3) ;
          tmpP_inv.block(6, 6, 3, 3) = slidingWindow[currentStateID]->cameraLink[linkID].P_inv.block(3, 3, 3, 3) ;
          double r_v = residualCamera.segment(0, 3).norm() ;
          if ( r_v > huber_r_v ){
            tmpP_inv *= huber_r_v/r_v ;
          }
          double r_w = residualCamera.segment(6, 3).norm() ;
          if ( r_w > huber_r_w ){
            tmpP_inv *= huber_r_w/r_w ;
          }

          H_i_2_j_T = H_i_2_j.transpose();
          H_i_2_j_T *= tmpP_inv;

          tmpHTH = H_i_2_j_T  *  H_i_2_j;
          tmpHTb = H_i_2_j_T * residualCamera;

          int currentStateIDIndex = currentStateID - head;
          if ( currentStateIDIndex < 0){
            currentStateIDIndex += slidingWindowSize;
          }
          int linkIDIndex = linkID - head  ;
          if (linkIDIndex < 0){
            linkIDIndex += slidingWindowSize;
          }

          HTH.block(currentStateIDIndex * 9, currentStateIDIndex * 9, 9, 9) += tmpHTH.block(0, 0, 9, 9);
          HTH.block(currentStateIDIndex * 9, linkIDIndex * 9, 9, 9) += tmpHTH.block(0, 9, 9, 9);
          HTH.block(linkIDIndex * 9, currentStateIDIndex * 9, 9, 9) += tmpHTH.block(9, 0, 9, 9);
          HTH.block(linkIDIndex * 9, linkIDIndex * 9, 9, 9) += tmpHTH.block(9, 9, 9, 9);

          HTb.segment(currentStateIDIndex * 9, 9) -= tmpHTb.segment(0, 9);
          HTb.segment(linkIDIndex * 9, 9) -= tmpHTb.segment(9, 9);
        }
      }
//      printf("[numList in BA]=%d\n", numList ) ;

      //solve the BA
      //cout << "HTH\n" << HTH << endl;

      LLT<MatrixXd> lltOfHTH = HTH.llt();
      ComputationInfo info = lltOfHTH.info();
      if (info == Success)
      {
        VectorXd dx = lltOfHTH.solve(HTb);

 //       printf("[BA] %d %f\n",iterNum, dx.norm() ) ;

 //       cout << iterNum << endl ;
 //       cout << dx.transpose() << endl ;

        //cout << "iteration " << iterNum << "\n" << dx << endl;
#ifdef DEBUG_INFO
        geometry_msgs::Vector3 to_pub ;
        to_pub.x = dx.norm() ;
        printf("%d %f\n",iterNum, to_pub.x ) ;
        pub_BA.publish( to_pub ) ;
#endif

        VectorXd errorUpdate(9);
        for (int i = 0; i < numOfState; i++)
        {
          int k = head + i;
          if (k >= slidingWindowSize){
            k -= slidingWindowSize;
          }
          errorUpdate = dx.segment(i * 9, 9);
          T[k] += errorUpdate.segment(0, 3);
          vel[k] += errorUpdate.segment(3, 3);

          Quaterniond q(R[k]);
          Quaterniond dq;
          dq.x() = errorUpdate(6) * 0.5;
          dq.y() = errorUpdate(7) * 0.5;
          dq.z() = errorUpdate(8) * 0.5;
          dq.w() = sqrt(1 - SQ(dq.x()) * SQ(dq.y()) * SQ(dq.z()));
          R[k] = (q * dq).normalized().toRotationMatrix();
        }
        //cout << T[head].transpose() << endl;
      }
      else
      {
        ROS_WARN("LLT error!!!");
        iterNum = maxIterationBA;
        //cout << HTH << endl;
        //FullPivLU<MatrixXd> luHTH(HTH);
        //printf("rank = %d\n", luHTH.rank() ) ;
        //HTH.rank() ;
      }
    }

    // Include correction for information vector
    int m_sz = margin.size;
    VectorXd r0 = VectorXd::Zero(STATE_SZ(numOfState - 1));
    for (int i = numOfState - 2; i >= 0; i--)
    {
      int k = head + i;
      if (k >= slidingWindowSize){
        k -= slidingWindowSize;
      }
      //dp, dv, dq
      r0.segment(STATE_SZ(i), 3) = T[k] - slidingWindow[k]->T_bk_2_b0;
      r0.segment(STATE_SZ(i) + 3, 3) = vel[k] - slidingWindow[k]->v_bk;
      r0.segment(STATE_SZ(i) + 6, 3) = Quaterniond(slidingWindow[k]->R_bk_2_b0.transpose() * R[k]).vec() * 2.0;
    }
    margin.bp.segment(0, STATE_SZ(m_sz)) += margin.Ap.block(0, 0, STATE_SZ(m_sz), STATE_SZ(m_sz))*r0;

    for (int i = 0; i < numOfState ; i++)
    {
        int k = head + i;
        if (k >= slidingWindowSize){
          k -= slidingWindowSize;
        }
        slidingWindow[k]->R_bk_2_b0 = R[k];
        slidingWindow[k]->T_bk_2_b0 = T[k];
        slidingWindow[k]->v_bk = vel[k];
    }
}


void SlamSystem::insertFrame(int imageSeqNumber, cv::Mat img, ros::Time imageTimeStamp, Matrix3d R, Vector3d T, Vector3d vel )
{
    tail++ ;
    numOfState++;
//    if ( numOfState == slidingWindowSize ){
//        head++ ;
//        if ( head >= slidingWindowSize ){
//            head -= slidingWindowSize ;
//        }
//        numOfState-- ;
//    }
    if (tail >= slidingWindowSize){
      tail -= slidingWindowSize;
    }
    slidingWindow[tail].reset(
                new Frame( tail, width, height, K, imageTimeStamp.toSec(), img.data, R, T, vel )
                );
}

void SlamSystem::insertCameraLink(Frame* keyFrame, Frame* currentFrame,
        const Matrix3d& R_k_2_c, const Vector3d& T_k_2_c, const MatrixXd& lastestATA )
{
    int id = currentFrame->id() ;
    keyFrame->cameraLinkList.push_back(id);
    keyFrame->cameraLink[id].R_bi_2_bj = R_k_2_c;
    keyFrame->cameraLink[id].T_bi_2_bj = T_k_2_c;
    keyFrame->cameraLink[id].P_inv = lastestATA;
  //keyFrame->cameraLink[currentFrame->id].T_trust = T_trust ;
}

void SlamSystem::setReprojectionListRelateToLastestKeyFrame(int begin, int end, Frame* current,
                                                            const Eigen::Matrix3d& R_i_2_c, const Eigen::Vector3d& T_i_2_c )
{
    int num = end - begin;
    if ( num < 0 ){
        num += slidingWindowSize ;
    }
    int trackFrameCnt = 0 ;
    for (int i = 0; i < num; i++)
    {
        int ref_id = begin + i;
        if (ref_id >=  slidingWindowSize) {
            ref_id -= slidingWindowSize;
        }
        if ( slidingWindow[ref_id]->keyFrameFlag == false
             || trackFrameCnt > 10
             ){
            continue;
        }

        double closenessTH = 1.0 ;
        double distanceTH = closenessTH * 15 / (KFDistWeight*KFDistWeight);
        //double cosAngleTH = 1.0 - 0.25 * closenessTH ;

        //euclideanOverlapCheck
        double distFac = slidingWindow[ref_id]->meanIdepth ;
        Eigen::Vector3d dd = ( slidingWindow[ref_id]->T_bk_2_b0 - current->T_bk_2_b0) * distFac;
        if( dd.dot(dd) > distanceTH) continue;

//        Eigen::Quaterniond qq( slidingWindow[ref_id]->R_bk_2_b0.transpose() * current->R_bk_2_b0) ;
//        Eigen::Vector3d aa = qq.vec()*2.0 ;
//        double dirDotProd = aa.dot( aa ) ;
//        if(dirDotProd < cosAngleTH) continue;


        Matrix3d R_i_2_j ;
        Vector3d T_i_2_j ;
        SE3 c2f_init ;
        //check from current to ref
        R_i_2_j = slidingWindow[ref_id]->R_bk_2_b0.transpose() * current->R_bk_2_b0 ;
        T_i_2_j = -slidingWindow[ref_id]->R_bk_2_b0.transpose() * ( slidingWindow[ref_id]->T_bk_2_b0 - current->T_bk_2_b0 ) ;
        c2f_init.setRotationMatrix(R_i_2_j);
        c2f_init.translation() = T_i_2_j ;
        trackerConstraint->trackFrameOnPermaref(current, slidingWindow[ref_id].get(), c2f_init ) ;
        if ( trackerConstraint->trackingWasGood == false ){
            //ROS_WARN("first check fail") ;
            continue ;
        }
        //ROS_WARN("pass first check") ;

        //check from ref to current
        R_i_2_j = current->R_bk_2_b0.transpose() * slidingWindow[ref_id]->R_bk_2_b0 ;
        T_i_2_j = -current->R_bk_2_b0.transpose() * ( current->T_bk_2_b0 - slidingWindow[ref_id]->T_bk_2_b0 ) ;
        c2f_init.setRotationMatrix(R_i_2_j);
        c2f_init.translation() = T_i_2_j ;
        trackerConstraint->trackFrameOnPermaref(slidingWindow[ref_id].get(), current, c2f_init ) ;
        if ( trackerConstraint->trackingWasGood == false ){
            //ROS_WARN("second check fail") ;
            continue ;
        }
        //ROS_WARN("pass second check") ;

        //Pass the cross check
        if (  trackingReferenceConstraint->keyframe != slidingWindow[ref_id].get() ){
             trackingReferenceConstraint->importFrame( slidingWindow[ref_id].get() );
        }

        SE3 RefToFrame = trackerConstraint->trackFrame( trackingReferenceConstraint, current,
                                   c2f_init );
        trackFrameCnt++ ;
        //float tracking_lastResidual = trackerConstraint->lastResidual;
        //float tracking_lastUsage = trackerConstraint->pointUsage;
        //float tracking_lastGoodPerBad = trackerConstraint->lastGoodCount / (trackerConstraint->lastGoodCount + trackerConstraint->lastBadCount);
        float tracking_lastGoodPerTotal = trackerConstraint->lastGoodCount / (current->width(SE3TRACKING_MIN_LEVEL)*current->height(SE3TRACKING_MIN_LEVEL));
        Sophus::Vector3d dist = RefToFrame.translation() * slidingWindow[ref_id]->meanIdepth;
        float minVal = 1.0f;
        float lastTrackingClosenessScore = getRefFrameScore(dist.dot(dist), trackerConstraint->pointUsage, KFDistWeight, KFUsageWeight);
        if ( trackerConstraint->trackingWasGood == false
             ||  tracking_lastGoodPerTotal < MIN_GOODPERALL_PIXEL
             || lastTrackingClosenessScore > minVal
             )
        {
            continue ;
        }



#ifdef PROJECT_TO_IMU_CENTER
        Eigen::Matrix3d r_i_2_j = RefToFrame.rotationMatrix().cast<double>();
        Eigen::Vector3d t_i_2_j = RefToFrame.translation().cast<double>();
        Eigen::Matrix3d final_R = R_i_2_c.transpose()*r_i_2_j*R_i_2_c;
        Eigen::Vector3d final_T = R_i_2_c.transpose()*(r_i_2_j*T_i_2_c + t_i_2_j ) - R_i_2_c.transpose()*T_i_2_c ;
#else
        Eigen::Matrix3d final_R = RefToFrame.rotationMatrix().cast<double>();
        Eigen::Vector3d final_T = RefToFrame.translation().cast<double>();
#endif

        //ROS_WARN("[add link, from %d to %d]", slidingWindow[ref_id]->id(), current->id() ) ;
        insertCameraLink( slidingWindow[ref_id].get(), current,
                          final_R,
                          final_T,
                          MatrixXd::Identity(6, 6)*DENSE_TRACKING_WEIGHT ) ;
        break ;
    }
}

void SlamSystem::processIMU(double dt, const Vector3d&linear_acceleration, const Vector3d &angular_velocity)
{
    Quaterniond dq;

     dq.x() = angular_velocity(0)*dt*0.5;
     dq.y() = angular_velocity(1)*dt*0.5;
     dq.z() = angular_velocity(2)*dt*0.5;
     dq.w() = sqrt(1 - SQ(dq.x()) * SQ(dq.y()) * SQ(dq.z()));

     Matrix3d deltaR(dq);
     //R_c_0 = R_c_0 * deltaR;
     //T_c_0 = ;
     Frame *current = slidingWindow[tail].get();

     Matrix<double, 9, 9> F = Matrix<double, 9, 9>::Zero();
     F.block<3, 3>(0, 3) = Matrix3d::Identity();
     F.block<3, 3>(3, 6) = -current->R_k1_k* vectorToSkewMatrix(linear_acceleration);
     F.block<3, 3>(6, 6) = -vectorToSkewMatrix(angular_velocity);

     Matrix<double, 6, 6> Q = Matrix<double, 6, 6>::Zero();
     Q.block<3, 3>(0, 0) = acc_cov;
     Q.block<3, 3>(3, 3) = gyr_cov;

     Matrix<double, 9, 6> G = Matrix<double, 9, 6>::Zero();
     G.block<3, 3>(3, 0) = -current->R_k1_k;
     G.block<3, 3>(6, 3) = -Matrix3d::Identity();

     current->P_k = (Matrix<double, 9, 9>::Identity() + dt * F) * current->P_k * (Matrix<double, 9, 9>::Identity() + dt * F).transpose() + (dt * G) * Q * (dt * G).transpose();
     //current->R_k1_k = current->R_k1_k*deltaR;
     current->alpha_c_k += current->beta_c_k*dt + current->R_k1_k*linear_acceleration * dt * dt * 0.5 ;
     current->beta_c_k += current->R_k1_k*linear_acceleration*dt;
     current->R_k1_k = current->R_k1_k*deltaR;
     current->timeIntegral += dt;
}

void SlamSystem::updateTrackingReference()
{
    if (  trackingReference->keyframe != currentKeyFrame.get() ){
        trackingReference->importFrame( currentKeyFrame.get() );
        currentKeyFrame->setPermaRef( trackingReference );
    }
}

void SlamSystem::trackFrame(cv::Mat img0, unsigned int frameID,
                            ros::Time imageTimeStamp, Eigen::Matrix3d deltaR,
                            const Eigen::Matrix3d R_i_2_c,const Eigen::Vector3d T_i_2_c
                            )
{
	// Create new frame
    std::shared_ptr<Frame> trackingNewFrame(
                new Frame( frameID, width, height, K, imageTimeStamp.toSec(), img0.data,
                           Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() )
                );
    //updateTrackingReference() ;

    //initial guess
    SE3 RefToFrame_initialEstimate ;
    RefToFrame_initialEstimate.setRotationMatrix(  deltaR.transpose()*RefToFrame.rotationMatrix() );
    RefToFrame_initialEstimate.translation() =
            deltaR.transpose()*RefToFrame.translation() ;

    //track
	struct timeval tv_start, tv_end;
	gettimeofday(&tv_start, NULL);
    RefToFrame = tracker->trackFrame( trackingReference, trackingNewFrame.get(),
                               RefToFrame_initialEstimate );
	gettimeofday(&tv_end, NULL);

//    Eigen::Matrix3d R_k_2_c = RefToFrame.rotationMatrix();
//    Eigen::Vector3d T_k_2_c = RefToFrame.translation();
//    Matrix3d R_bk1_2_b0 = trackingReference->keyframe->R_bk_2_b0 * R_k_2_c.transpose();
//    Vector3d T_bk1_2_b0 = trackingReference->keyframe->T_bk_2_b0 + R_bk1_2_b0*T_k_2_c ;
//    pubOdometry(-T_bk1_2_b0, R_bk1_2_b0, pub_odometry, pub_pose );
//    pubPath(-T_bk1_2_b0, path_line, pub_path );

    //debug information
    //msTrackFrame = 0.9*msTrackFrame + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
    msTrackFrame = (tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f ;
    printf("msTrackFrame = %0.f\n", msTrackFrame ) ;
	nTrackFrame++;
	tracking_lastResidual = tracker->lastResidual;
	tracking_lastUsage = tracker->pointUsage;
	tracking_lastGoodPerBad = tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount);
	tracking_lastGoodPerTotal = tracker->lastGoodCount / (trackingNewFrame->width(SE3TRACKING_MIN_LEVEL)*trackingNewFrame->height(SE3TRACKING_MIN_LEVEL));

//    geometry_msgs::Vector3 v_pub ;
//    Vector3d translation = RefToFrame.translation() ;
//    v_pub.x = translation(0) ;
//    v_pub.y = translation(1) ;
//    v_pub.z = translation(2) ;
//    pub_denseTracking.publish( v_pub ) ;

	// Keyframe selection
    createNewKeyFrame = false ;
    //printf("tracking_lastGoodPerTotal = %f\n", tracking_lastGoodPerTotal ) ;
    if ( trackingReference->keyframe->numFramesTrackedOnThis > MIN_NUM_MAPPED )
	{
        Sophus::Vector3d dist = RefToFrame.translation() * currentKeyFrame->meanIdepth;
        float minVal = 1.0f;

        lastTrackingClosenessScore = getRefFrameScore(dist.dot(dist), tracker->pointUsage, KFDistWeight, KFUsageWeight);
        if (lastTrackingClosenessScore > minVal || tracker->trackingWasGood == false
                || tracking_lastGoodPerTotal < MIN_GOODPERALL_PIXEL
                )
		{
			createNewKeyFrame = true;

           // if(enablePrintDebugInfo && printKeyframeSelectionInfo)
           //     printf("[insert KF] dist %.3f + usage %.3f = %.3f > 1\n", dist.dot(dist), tracker->pointUsage, lastTrackingClosenessScore );
        }
		else
		{
        //	if(enablePrintDebugInfo && printKeyframeSelectionInfo)
        //       printf("SKIPPD %d on %d! dist %.3f + usage %.3f = %.3f < 1\n",trackingNewFrame->id(),trackingNewFrame->getTrackingParent()->id(), dist.dot(dist), tracker->pointUsage, lastTrackingClosenessScore );
		}
	}
    if ( tracker->diverged ){
        createNewKeyFrame = true ;
    }
    frameInfoList_mtx.lock();
    int tmpTail = frameInfoListTail+1 ;
    if ( tmpTail >= frameInfoListSize ){
        tmpTail -= frameInfoListSize;
    }
    FRAMEINFO& tmpFrameInfo = frameInfoList[tmpTail] ;
    tmpFrameInfo.t = imageTimeStamp ;

#ifdef PROJECT_TO_IMU_CENTER
    Eigen::Matrix3d r_k_2_c = RefToFrame.rotationMatrix().cast<double>();
    Eigen::Vector3d t_k_2_c = RefToFrame.translation().cast<double>();
    tmpFrameInfo.R_k_2_c = R_i_2_c.transpose()*r_k_2_c*R_i_2_c;
    tmpFrameInfo.T_k_2_c = R_i_2_c.transpose()*(r_k_2_c*T_i_2_c + t_k_2_c ) - R_i_2_c.transpose()*T_i_2_c ;
#else
    tmpFrameInfo.R_k_2_c = RefToFrame.rotationMatrix().cast<double>();
    tmpFrameInfo.T_k_2_c = RefToFrame.translation().cast<double>();
#endif

//    ROS_WARN("trackFrame = ") ;
//    std::cout << tmpFrameInfo.T_k_2_c.transpose() << std::endl;

    tmpFrameInfo.trust = tracker->trackingWasGood ;
    tmpFrameInfo.keyFrameFlag = createNewKeyFrame ;
    tmpFrameInfo.lastestATA = MatrixXd::Identity(6, 6)*DENSE_TRACKING_WEIGHT ;
    frameInfoListTail = tmpTail ;
    frameInfoList_mtx.unlock();

    if ( createNewKeyFrame == true ){
        tracking_mtx.lock();
        lock_densetracking = true ;
        tracking_mtx.unlock();
    }
}


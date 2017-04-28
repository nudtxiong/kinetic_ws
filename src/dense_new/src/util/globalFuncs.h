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
#include <opencv2/core/core.hpp>
#include "../util/settings.h"
#include "../util/SophusUtil.h"
#include "../DataStructures/Frame.h"

namespace lsd_slam
{

//template< typename T >
//class NotifyBuffer;

//class Frame;

SE3 SE3CV2Sophus(const cv::Mat& R, const cv::Mat& t);

void printMessageOnCVImage(cv::Mat &image, std::string line1);

// reads interpolated element from a uchar* array
// SSE2 optimization possible
inline float getInterpolatedElement(const float* const mat, const float x, const float y, const int width)
{
	//stats.num_pixelInterpolations++;

	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const float* bp = mat +ix+iy*width;


	float res =   dxdy * bp[1+width]
				+ (dy-dxdy) * bp[width]
				+ (dx-dxdy) * bp[1]
				+ (1-dx-dy+dxdy) * bp[0];

	return res;
}

inline Eigen::Vector3f getInterpolatedElement43(const Eigen::Vector4f* const mat, const float x, const float y, const int width)
{
	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const Eigen::Vector4f* bp = mat +ix+iy*width;


	return dxdy * *(const Eigen::Vector3f*)(bp+1+width)
	        + (dy-dxdy) * *(const Eigen::Vector3f*)(bp+width)
	        + (dx-dxdy) * *(const Eigen::Vector3f*)(bp+1)
			+ (1-dx-dy+dxdy) * *(const Eigen::Vector3f*)(bp);
}

inline Eigen::Vector4f getInterpolatedElement44(const Eigen::Vector4f* const mat, const float x, const float y, const int width)
{
	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const Eigen::Vector4f* bp = mat +ix+iy*width;


	return dxdy * *(bp+1+width)
	        + (dy-dxdy) * *(bp+width)
	        + (dx-dxdy) * *(bp+1)
			+ (1-dx-dy+dxdy) * *(bp);
}

inline Eigen::Vector2f getInterpolatedElement42(const Eigen::Vector4f* const mat, const float x, const float y, const int width)
{
	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const Eigen::Vector4f* bp = mat +ix+iy*width;


	return dxdy * *(const Eigen::Vector2f*)(bp+1+width)
	        + (dy-dxdy) * *(const Eigen::Vector2f*)(bp+width)
	        + (dx-dxdy) * *(const Eigen::Vector2f*)(bp+1)
			+ (1-dx-dy+dxdy) * *(const Eigen::Vector2f*)(bp);
}
inline void fillCvMat(cv::Mat* mat, cv::Vec3b color)
{
	for(int y=0;y<mat->size().height;y++)
		for(int x=0;x<mat->size().width;x++)
			mat->at<cv::Vec3b>(y,x) = color;
}

inline void setPixelInCvMat(cv::Mat* mat, cv::Vec3b color, int xx, int yy, int lvlFac)
{
	for(int x=xx*lvlFac; x < (xx+1)*lvlFac && x < mat->size().width;x++)
		for(int y=yy*lvlFac; y < (yy+1)*lvlFac && y < mat->size().height;y++)
			mat->at<cv::Vec3b>(y,x) = color;
}

inline cv::Vec3b getGrayCvPixel(float val)
{
	if(val < 0) val = 0; if(val>255) val=255;
	return cv::Vec3b(val,val,val);
}

cv::Mat getDepthRainbowPlot(Frame* kf, int lvl=0);
cv::Mat getDepthRainbowPlot(const float* idepth, const float* idepthVar, const float* gray, int width, int height);
cv::Mat getVarRedGreenPlot(const float* idepthVar, const float* gray, int width, int height);
}

inline float getRefFrameScore(float distanceSquared, float usage, float KFDistWeight, float KFUsageWeight)
{
    return distanceSquared*KFDistWeight*KFDistWeight
            + (1-usage)*(1-usage) * KFUsageWeight * KFUsageWeight;
}

inline void calculateDepthImage(cv::Mat disparity, cv::Mat& depthImage, float baseline, float f)
{
    int n = disparity.rows ;
    int m = disparity.cols ;

    depthImage.create(n, m, CV_32F);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            float d = disparity.at<float>(i, j);
            if ( d < 3.0 ) {
                depthImage.at<float>(i, j) = 0;
            }
            else {
                depthImage.at<float>(i, j) = baseline * f / d;
            }
        }
    }
}

inline float SQ(float a){
    return a*a;
}

inline Eigen::Matrix3d vectorToSkewMatrix(const Eigen::Vector3d& w)
{
  Eigen::Matrix3d skewW(3, 3);
  skewW(0, 0) = skewW(1, 1) = skewW(2, 2) = 0;
  skewW(0, 1) = -w(2);
  skewW(1, 0) = w(2);
  skewW(0, 2) = w(1);
  skewW(2, 0) = -w(1);
  skewW(1, 2) = -w(0);
  skewW(2, 1) = w(0);

  return skewW;
}

inline void RtoEulerAngles(Eigen::Matrix3d R, double a[3])
{
    double theta = acos(0.5*(R(0, 0) + R(1, 1) + R(2, 2) - 1.0));
    a[0] = (R(2, 1) - R(1, 2)) / (2.0* sin(theta));
    a[1] = (R(0, 2) - R(2, 0)) / (2.0* sin(theta));
    a[2] = (R(1, 0) - R(0, 1)) / (2.0* sin(theta));
}

inline void R_to_ypr(const Eigen::Matrix3d& R, double angle[3])
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    //Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0)*cos(y) + n(1)*sin(y));
    double r = atan2(a(0)*sin(y) - a(1)*cos(y), -o(0)*sin(y) + o(1)*cos(y));
    angle[0] = y;
    angle[1] = p;
    angle[2] = r;
    //ypr(0) = y;
    //ypr(1) = p;
    //ypr(2) = r;
}

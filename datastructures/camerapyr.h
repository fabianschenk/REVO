/**
* This file is part of REVO.
*
* Copyright (C) 2014-2017 Schenk Fabian <schenk at icg dot tugraz dot at> (Graz University of Technology)
* For more information see <https://github.com/fabianschenk/REVO/>
*
* REVO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* REVO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with REVO. If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Eigen>
#include "../utils/Logging.h"

class ImgPyramidSettings
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImgPyramidSettings(const std::string& settingsFile)
    {
        cv::FileStorage dataF(settingsFile,cv::FileStorage::READ);
        if (!dataF.isOpened())
        {
            I3D_LOG(i3d::error) << "Couldn't open settings file at location: " << settingsFile;
            exit(0);
        }
        //image pyramid settings (camera matrix, resolutions,...)
        cv::read(dataF["cannyThreshold1"],cannyThreshold1,150);
        cv::read(dataF["cannyThreshold2"],cannyThreshold2,100);
        cv::read(dataF["DO_GAUSSIAN_SMOOTHING_BEFORE_CANNY"],DO_GAUSSIAN_SMOOTHING_BEFORE_CANNY,false);
        cv::read(dataF["DEPTH_MIN"],DEPTH_MIN,0.1f);
        cv::read(dataF["DEPTH_MAX"],DEPTH_MAX,5.2f);
        cv::read(dataF["PYR_MIN_LVL"],PYR_MIN_LVL,2);
        cv::read(dataF["PYR_MAX_LVL"],PYR_MAX_LVL,0);
        cv::read(dataF["DO_UNDISTORT"],DO_UNDISTORT,false);

        int height,width;
        cv::read(dataF["height"],height,480);
        cv::read(dataF["width"],width,640);
        this->height = static_cast<size_t>(height);this->width = static_cast<size_t>(width);
        I3D_LOG(i3d::info) << "settingsPyr.DEPTH_MIN: " << DEPTH_MIN << "settingsPyr.DEPTH_MAX" << DEPTH_MAX;
        float fx,fy,cx,cy;
        //Here, we could also use the standard ROS parameters for 640x480 -> fx = fy = 580
        cv::read(dataF["Camera.fx"],fx,(width+height)/2);
        cv::read(dataF["Camera.fy"],fy,fx);
        cv::read(dataF["Camera.cx"],cx,width/2.0f);
        cv::read(dataF["Camera.cy"],cy,height/2.0f);
        K = Eigen::Matrix3f::Identity();
        K(0,0) = fx; K(1,1) = fy; K(0,2) = cx; K(1,2) = cy;
        cv::read(dataF["USE_EDGE_HIST"],USE_EDGE_HIST,true);
        cv::read(dataF["nPercentage"],nPercentage,0.3f);
        cv::read(dataF["USE_PYR_SMOOTH"],USE_PYR_SMOOTH,true);
        dataF.release();
    }

    inline int nLevels() const
    {
        return PYR_MIN_LVL-PYR_MAX_LVL+1;
    }

    int PYR_MIN_LVL,PYR_MAX_LVL;
    //EdgeType edgeType;
    bool ts_float;

    float DEPTH_MIN,DEPTH_MAX;
    uint distFromEdges;
    Eigen::Matrix3f K;
    cv::Mat distCoeff;
    size_t width,height;
    int cannyThreshold1, cannyThreshold2;
    std::string dataFolder;
    bool DO_UNDISTORT;
    bool USE_EDGE_HIST;
    bool USE_PYR_SMOOTH;
    bool DO_GAUSSIAN_SMOOTHING_BEFORE_CANNY;
    float nPercentage;
};
class Camera
{
    public:
        Camera(float fx,float fy,float cx, float cy, size_t width, size_t height):
            fx(fx),fy(fy),cx(cx),cy(cy),width(width),height(height), area(this->height*this->width)
        {

        }
        Camera(float fx,float fy,float cx, float cy, size_t width, size_t height, float scale):
            fx(fx*scale),fy(fy*scale),cx(cx*scale),cy(cy*scale),
            width(static_cast<size_t>(width*scale)),height(static_cast<size_t>(height*scale)), area(this->height*this->width)
        {

        }
        float fx,fy,cx,cy;
        size_t width,height;
        size_t area;
        cv::Size2i returnSize() const
        {
            return cv::Size2i(static_cast<int>(width),static_cast<int>(height));
        }
};

class CameraPyr
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraPyr(const ImgPyramidSettings& settingsPyr)
    {
        I3D_LOG(i3d::info) << settingsPyr.cannyThreshold1 << " " <<settingsPyr.cannyThreshold2 << " " << settingsPyr.K;
        int nLevels = settingsPyr.nLevels();
        if (nLevels <= 0) return;
        const Eigen::Matrix3f baseCam = settingsPyr.K;
        float fx = baseCam(0,0), fy = baseCam(1,1), cx = baseCam(0,2), cy = baseCam(1,2);
        const float width = settingsPyr.width, height = settingsPyr.height;
        if (settingsPyr.DO_UNDISTORT)
        {
            cv::Mat camMatcV = cv::Mat::eye(3,3,CV_64F);
            camMatcV.at<double>(0,0) = fx;camMatcV.at<double>(1,1) = fy;
            camMatcV.at<double>(0,2) = cx;camMatcV.at<double>(1,2) = cy;

            cv::Size imgSize(width,height);
            cv::Mat newCamMat = cv::getOptimalNewCameraMatrix(camMatcV,settingsPyr.distCoeff,imgSize,0,imgSize);
            cv::initUndistortRectifyMap(camMatcV,settingsPyr.distCoeff,cv::Matx33d::eye(),newCamMat,imgSize,CV_16SC2,map1,map2);
            I3D_LOG(i3d::info) << map1 << map2 << map1.size() << map2.size() << newCamMat;
            fx = newCamMat.at<double>(0,0);fy = newCamMat.at<double>(1,1);
            cx = newCamMat.at<double>(0,2);cy = newCamMat.at<double>(1,2);
        }
        I3D_LOG(i3d::info) << fx << " " << fy << " " << settingsPyr.nLevels();
        camPyr.push_back(Camera(fx,fy,cx,cy,width,height));
        for (int lvl = 1; lvl <= nLevels;++lvl)
        {
            const float scale = 1.0f/pow(2,lvl);
            camPyr.push_back(Camera(fx,fy,cx,cy,width,height,scale));
        }
        I3D_LOG(i3d::info) << "camPyr.size(): " << camPyr.size();
        //generate PCL Template
        for (size_t lvl=0; lvl < camPyr.size();++lvl)
        {
            const Camera cam = camPyr[lvl];
            Eigen::Matrix4Xf pcl = Eigen::Matrix4Xf::Zero(4, cam.area);
            for( size_t xx = 0; xx < cam.width ; xx++)
            {
                for( size_t yy = 0; yy < cam.height ; yy++)
                {
                    const float X = (xx-cam.cx) / cam.fx;
                    const float Y = (yy-cam.cy) / cam.fy;
                    const int pclIdx = yy*cam.width+xx;
                    pcl.col(pclIdx) = Eigen::Vector4f( X, Y, 1, 1);
                }
            }
            //now add it to the pyramid
            this->mPclTemplate.push_back(pcl);
        }
    }

    CameraPyr(float fx,float fy,float cx, float cy, size_t width, size_t height, int nLevels)
    {
        if (nLevels <= 0) return;
        camPyr.push_back(Camera(fx,fy,cx,cy,width,height));
        for (int lvl = 1; lvl <= nLevels;++lvl)
        {
            const float scale = 1.0/pow(2,lvl);
            camPyr.push_back(Camera(fx,fy,cx,cy,width,height,scale));
        }
    }

    inline int size() const
    {
        return camPyr.size();
    }

    const Camera& at(int lvl) const
    {
        //I3D_LOG(i3d::info) << "RETURN CAMERA!";
        return camPyr.at(lvl);
    }
    std::vector<Camera> camPyr;
    cv::Mat map1,map2;
    //A pointcloud template only depends on camera paramaters and x,y coordinates.
    //Simply multiply by z to get a full pointcloud
    //std::vector<Eigen::Matrix4Xf, Eigen::aligned_allocator<Eigen::Matrix4Xf>> mPclTemplate;
    std::vector<Eigen::Matrix4Xf> mPclTemplate;
};

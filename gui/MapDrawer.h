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
*
*
* This viewer was partly adapted from ORB-SLAM2 <https://github.com/raulmur/ORB_SLAM2>!
*/

#pragma once
#include <Eigen/StdVector>
#include <Eigen/StdDeque>
#include "../utils/Logging.h"
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <memory>
#include <queue>

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)

namespace  REVOGui
{
class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    class PointCloudBuffered;
    MapDrawer();
    void addKeyframePoseF(const Eigen::Matrix4f& keyFramePose)
    {
        std::unique_lock<std::mutex> lock(this->mFrameMutex);
        this->vpKfs.push_back(keyFramePose.cast<double>());
        mMapNeedsUpdate = true;
    }
    void addOptFramePoseF(const Eigen::Matrix4f& framePose)
    {
        std::unique_lock<std::mutex> lock(this->mFrameOptMutex);
        this->vpOptFrames.push_back(framePose.cast<double>());
        mMapNeedsUpdate = true;
    }
    void addFramePoseF(const Eigen::Matrix4f& framePose)
    {
        std::unique_lock<std::mutex> lock(this->mFrameMutex);
        this->vpFrames.push_back(framePose.cast<double>());
        mMapNeedsUpdate = true;
    }

    void addKeyframePose(const Eigen::Matrix4d& keyFramePose)
    {
        std::unique_lock<std::mutex> lock(this->mFrameMutex);
        this->vpKfs.push_back(keyFramePose);
        mMapNeedsUpdate = true;
    }
    void addFramePose(const Eigen::Matrix4d& framePose)
    {
        std::unique_lock<std::mutex> lock(this->mFrameMutex);
        this->vpFrames.push_back(framePose);
        mMapNeedsUpdate = true;
    }
    void setIntegratingFlag(bool menuIntegrate)
    {
        std::unique_lock<std::mutex> lock(this->mPclMutex);
        mIntegrating = menuIntegrate;
    }

    void addPclAndKfPoseToQueue(const Eigen::MatrixXf &kfPcl,const Eigen::Matrix4f &T_Wkf)
    {

        std::unique_lock<std::mutex> lock(this->mPclMutex);
        if (mIntegrating)
        {
            I3D_LOG(i3d::info) << "Adding: " << kfPcl.cols() << " " << T_Wkf;
            this->vpKfsF.push_back(T_Wkf);
            this->pclKfsClr.push(kfPcl);
            this->pclKfHost.push_back(kfPcl);
            this->nPts += kfPcl.cols();
            I3D_LOG(i3d::debug) << "addPclAndKfPoseToQueue" << kfPcl.cols() << "x" << kfPcl.rows();
        }

    }

    void saveModel()
    {
        std::unique_lock<std::mutex> lock(mModelLock);
        if (modelIsSaved) return;
        std::ofstream pclFile;
        pclFile.open("outputPcl.ply",std::ios_base::out);
        pclFile << "ply\nformat ascii 1.0\nelement vertex "<<nPts<<"\n";
        pclFile << "property float32 x\nproperty float32 y\nproperty float32 z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
        for (size_t i = 0; i < pclKfHost.size();++i)
        {
            const Eigen::MatrixXf ref3dList = pclKfHost[i];
            for (int c = 0;c<ref3dList.cols();++c)
            {
                const Eigen::VectorXf pt = ref3dList.col(c);
                //I3D_LOG(i3d::info) << pt.transpose();
                pclFile << pt[0] << " " << pt[1] << " " << pt[2] << " " << pt[4]*255.0f << " " << pt[5]*255.0f << " " << pt[6]*255.0f << std::endl;
            }
        }

        pclFile.close();
        std::ofstream kfFile;
        kfFile.open("outputKf.ply",std::ios_base::out);
        kfFile << "ply\nformat ascii 1.0\nelement vertex "<<vpKfsF.size()*5<<"\n";
        kfFile << "property float32 x\nproperty float32 y\nproperty float32 z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n";
        kfFile << "element edge "<< vpKfsF.size() * 9 - 1 <<"\nproperty int vertex1\nproperty int vertex2\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
//        element edge 5                        { five edges in object }
//        property int vertex1                  { index to first vertex of edge }
//        property int vertex2                  { index to second vertex }
//        property uchar red                    { start of edge color }
//        property uchar green
//        property uchar blue
//        kfFile << "end_header\n"
        const float w = 0.1;
        const float h = w*0.75;
        const float z = w*0.6;
        const Eigen::Vector3f corner1(w,h,z),corner2(w,-h,z),corner3(-w,-h,z),corner4(-w,h,z);
        for (size_t nCurrCam=0; nCurrCam < vpKfsF.size();++nCurrCam)
        {
            //draw five points for each camera

            const Eigen::Matrix4f currPose = vpKfsF[nCurrCam];

            const Eigen::Matrix3f R = currPose.block<3,3>(0,0);
            const Eigen::Vector3f T = currPose.block<3,1>(0,3);
            //const Eigen::Vector3f camCenter = -R.transpose()*T;
            const Eigen::Vector3f camCenter = T;
            const Eigen::Vector3f pt1 = R*corner1+T,pt2 = R*corner2+T,pt3 = R*corner3+T,pt4 = R*corner4+T;
            //camera center
            kfFile << camCenter[0] << " " << camCenter[1] << " " << camCenter[2] << " " << 0 << " " << 0 << " " << 255 << std::endl;
            //corner points
            kfFile << pt1[0] << " " << pt1[1] << " " << pt1[2] << " " << 0 << " " << 0 << " " << 255 << std::endl;
            kfFile << pt2[0] << " " << pt2[1] << " " << pt2[2] << " " << 0 << " " << 0 << " " << 255 << std::endl;
            kfFile << pt3[0] << " " << pt3[1] << " " << pt3[2] << " " << 0 << " " << 0 << " " << 255 << std::endl;
            kfFile << pt4[0] << " " << pt4[1] << " " << pt4[2] << " " << 0 << " " << 0 << " " << 255 << std::endl;
        }
        for (size_t nCurrCam=0; nCurrCam < vpKfsF.size();++nCurrCam)
        {
            //draw the edges between the points. This should always be the same order. (nCurrCam*5)+1,2,3,4.....
            const int ccIdx = nCurrCam*5, pt1Idx = nCurrCam*5+1, pt2Idx = nCurrCam*5+2, pt3Idx = nCurrCam*5+3, pt4Idx = nCurrCam*5+4;
            kfFile << ccIdx << " " << pt1Idx  << " " << 0 << " " << 0 << " " << 255 << std::endl;
            kfFile << ccIdx << " " << pt2Idx  << " " << 0 << " " << 0 << " " << 255 << std::endl;
            kfFile << ccIdx << " " << pt3Idx  << " " << 0 << " " << 0 << " " << 255 << std::endl;
            kfFile << ccIdx << " " << pt4Idx  << " " << 0 << " " << 0 << " " << 255 << std::endl;

            kfFile << pt1Idx << " " << pt4Idx  << " " << 0 << " " << 0 << " " << 255 << std::endl;
            kfFile << pt1Idx << " " << pt2Idx  << " " << 0 << " " << 0 << " " << 255 << std::endl;
            kfFile << pt2Idx << " " << pt3Idx  << " " << 0 << " " << 0 << " " << 255 << std::endl;
            kfFile << pt3Idx << " " << pt4Idx  << " " << 0 << " " << 0 << " " << 255 << std::endl;
            if (nCurrCam>0) //not the first camera
                kfFile << (nCurrCam-1)*5 << " " << ccIdx <<" "<< 0 << " " << 255 << " " << 0 << std::endl;
        }
        kfFile.close();
        modelIsSaved = true;
    }

    //Read pcls from the queue and then move them to the GPU!
    bool returnLastKfPcl(Eigen::MatrixXf& kfPcl)//, Eigen::Matrix4f& T_w_c)
    {
        //
        if (pclKfsClr.size()>0)
        {
            std::unique_lock<std::mutex> lock(this->mPclMutex);
            kfPcl = pclKfsClr.front();
            pclKfsClr.pop();
            return true;
        }
        return false;
    }
    void DrawKeyFramesCurr();
    void DrawFrameTrajectory();
    void DrawOptFrameTrajectory();
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const Eigen::Matrix4f& Tcw);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> vpKfsF;
private:
    bool mIntegrating;
    bool modelIsSaved;
    bool mMapNeedsUpdate;
    Eigen::MatrixXf edgePcl;
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    cv::Mat mCameraPose;
    bool cameraPoseSet;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> vpKfs;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> vpFrames;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> vpOptFrames;
    std::vector<Eigen::MatrixXf> pclKfs;
    std::queue<Eigen::MatrixXf> pclKfsClr;
    std::vector<Eigen::MatrixXf> pclKfHost;
    int nPts;
    std::mutex mMutexCamera;
    std::mutex mPclMutex;
    std::mutex mFrameMutex,mFrameOptMutex;
    std::mutex mModelLock;
};
}


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

#include "MapDrawer.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <Eigen/LU>

namespace  REVOGui
{
using namespace std;
MapDrawer::MapDrawer()
{
    mKeyFrameSize = 0.05f;
    mKeyFrameLineWidth = 2;
    mGraphLineWidth = 0.9f;
    mPointSize = 1;
    mCameraSize = 0.08f;
    mCameraLineWidth = 3;
    mMapNeedsUpdate = true;
    nPts = 0;
    modelIsSaved = false;
    mIntegrating = true;
}


void MapDrawer::DrawKeyFramesCurr()
{
    if (vpKfs.size()==0) return;
    const float w = 0.1f;
    const float h = w*0.75f;
    const float z = w*0.6f;
    std::unique_lock<std::mutex> lock(this->mFrameMutex);
    //drawnTill++;
    for(size_t i=0; i<vpKfs.size(); i++)
    {
        Eigen::Matrix4f Twc = vpKfs.at(i).cast<float>();//pKF->GetPoseInverse().t();
        //Twc = Twc.inverse().eval();
        glPushMatrix();
        //glMultMatrixf(Twc<GLfloat>(0));
        glMultMatrixf((float*)Twc.data());

        glLineWidth(mKeyFrameLineWidth);
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();
        glPopMatrix();
   }
   mMapNeedsUpdate = false;

}
//Draws a second trajecotry
void MapDrawer::DrawOptFrameTrajectory()
{
    if (vpOptFrames.size()==0) return;
    //drawnTill++;
    glLineWidth(mKeyFrameLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    for(size_t i=0; i<vpOptFrames.size()-1; i++)
    {
        const Eigen::Matrix4f Twc0 = vpOptFrames.at(i).inverse().cast<float>();
        const Eigen::Matrix4f Twc1 = vpOptFrames.at(i+1).inverse().cast<float>();
        const Eigen::Matrix3f R0 = Twc0.block<3,3>(0,0);
        const Eigen::Vector3f T0 = Twc0.block<3,1>(0,3);
        const Eigen::Matrix3f R1 = Twc1.block<3,3>(0,0);
        const Eigen::Vector3f T1 = Twc1.block<3,1>(0,3);
        const Eigen::Vector3f cc0 = -R0.transpose()*T0;
        const Eigen::Vector3f cc1 = -R1.transpose()*T1;
        glVertex3f(cc0[0],cc0[1],cc0[2]);
        glVertex3f(cc1[0],cc1[1],cc1[2]);
    }
    glEnd();
}

//Draws the trajectory of the camera!
void MapDrawer::DrawFrameTrajectory()
{
    if (vpFrames.size()==0) return;
    glLineWidth(mKeyFrameLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    for(size_t i=0; i<vpFrames.size()-1; i++)
    {
        const Eigen::Matrix4f Twc0 = vpFrames.at(i).inverse().cast<float>();//pKF->GetPoseInverse().t();
        const Eigen::Matrix4f Twc1 = vpFrames.at(i+1).inverse().cast<float>();//pKF->GetPoseInverse().t();
        const Eigen::Matrix3f R0 = Twc0.block<3,3>(0,0);
        const Eigen::Vector3f T0 = Twc0.block<3,1>(0,3);
        const Eigen::Matrix3f R1 = Twc1.block<3,3>(0,0);
        const Eigen::Vector3f T1 = Twc1.block<3,1>(0,3);
        const Eigen::Vector3f cc0 = -R0.transpose()*T0;
        const Eigen::Vector3f cc1 = -R1.transpose()*T1;
        glVertex3f(cc0[0],cc0[1],cc0[2]);
        glVertex3f(cc1[0],cc1[1],cc1[2]);
    }
    glEnd();
}

//Draws exactly one camera
void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif
    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


//void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
//{
//    unique_lock<mutex> lock(mMutexCamera);
//    mCameraPose = Tcw.clone();
//}
void MapDrawer::SetCurrentCameraPose(const Eigen::Matrix4f& Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    cv::eigen2cv(Tcw,mCameraPose);
}
void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}
}

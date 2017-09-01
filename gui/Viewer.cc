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

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <opencv2/highgui.hpp>
#include <mutex>
#include <GL/glew.h>
#include <pangolin/gl/glvbo.h>
namespace  REVOGui
{

//Viewer::Viewer(std::shared_ptr<FrameDrawer> pFrameDrawer, std::shared_ptr<MapDrawer> pMapDrawer):
//    /*mpFrameDrawer(pFrameDrawer),*/mpMapDrawer(pMapDrawer),
//    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false), quitRequested(false)
  Viewer::Viewer(std::shared_ptr<MapDrawer> pMapDrawer):
      mpMapDrawer(pMapDrawer), mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false), quitRequested(false)
{
    I3D_LOG(i3d::info) << "Generating Viewer!!";
    mSaveModel = false;
    float fps = 30;//fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;
    mImageWidth = 640;
    mImageHeight = 480;
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }
    mViewpointX = 0;
    mViewpointY = -0.7f;
    mViewpointZ = -1.8f;
    mViewpointF = 500;
    mbFinished = false;
    mbStopped = false;
    I3D_LOG(i3d::info) << "Finished generating Viewer!!";
}

void Viewer::Run()
{
    std::cout << "REVO MAP VIEWER" << std::endl;
    //1024,768
    pangolin::CreateWindowAndBind("REVO Map Viewer",1180,720);
    glewInit();
    glEnable(GL_DEPTH_TEST);
    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowTrajectory("menu.Show Trajectory",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuDrawCurrentCamera("menu.Show Current Camera",true,true);
    pangolin::Var<bool> menuShowPCL("menu.Show PCL",true,true);
    pangolin::Var<bool> menuIntegrate("menu.Integrating",true,false);
    menuIntegrate = true;
    menuShowPCL = true;
    menuShowKeyFrames = true;
    menuShowTrajectory = true;
    //bFollow = true;
    menuFollowCamera = true;
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    bool bFollow = false;
    int it = 0;
    glClearColor(166.0f/255.0f, 200.0f/255.0f, 237.0f/255.0f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    while(1)
    {
        ++it;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        mpMapDrawer->setIntegratingFlag(menuIntegrate);
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        clock_t start = clock();
        if (menuShowKeyFrames)
            mpMapDrawer->DrawKeyFramesCurr();
        Eigen::MatrixXf kfPcl;
        while (mpMapDrawer->returnLastKfPcl(kfPcl))
        {
            if (menuIntegrate)
            {
                const std::shared_ptr<PointCloudBuffered> pclPtr(new PointCloudBuffered(kfPcl,true));
                {
                    I3D_LOG(i3d::info) << "newPcl";
                    std::unique_lock<std::mutex> lock(mMtxPclBuffered);
                    this->mPclKfsBuffered.push_back(pclPtr);
                }
            }
        }
        char buffer[100];
        sprintf(buffer,"%05d",it);
        if (menuShowPCL) DrawPclBuffered();
        if (menuShowTrajectory) mpMapDrawer->DrawFrameTrajectory();
        if (menuDrawCurrentCamera) mpMapDrawer->DrawCurrentCamera(Twc);
        clock_t end = clock();
        I3D_LOG(i3d::detail) << "Time for drawing: " << double(end-start)/CLOCKS_PER_SEC*1000.0f;
        if (pangolin::ShouldQuit()) this->quitRequested=true;
        pangolin::FinishFrame();

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }
        if(CheckFinish())
            break;
        usleep(100000);
    }
    I3D_LOG(i3d::debug) << "Stop";
    if (mSaveModel)
    {
        I3D_LOG(i3d::debug) << "saveModel";
        mpMapDrawer->saveModel();
    }
    SetFinish();
}


void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }
    return false;
}

}

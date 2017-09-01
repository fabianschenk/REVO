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
#include "system.h"
#include "../utils/timer.h"
#include <iomanip>
#include <unistd.h>
REVO::REVO(const std::string& settingsFile, const std::string& dataSettings,int nRuns = 0):
    mSettings(settingsFile,dataSettings,nRuns), camPyr(new CameraPyr(mSettings.settingsPyr))//,windowedOptimization(mSettings.settingsWO)
{
    I3D_LOG(i3d::debug) << "SystemRGBD constructor";
    isFinished = true;
    if (!mSettings.settingsIO.isFinished)
    {
#ifdef WITH_PANGOLIN_VIEWER
    if (mSettings.DO_USE_PANGOLIN_VIEWER)
    {
        //Create Drawers. These are used by the Viewer
        mpMapDrawer = std::shared_ptr<REVOGui::MapDrawer>(new REVOGui::MapDrawer());
        mpViewer = std::shared_ptr<REVOGui::Viewer>(new REVOGui::Viewer(mpMapDrawer));
        mThViewer = std::unique_ptr<std::thread>(new std::thread(&REVOGui::Viewer::Run, mpViewer));
    }
#endif
    if (!camPyr) {I3D_LOG(i3d::error) << "camPyr failed!";}
    I3D_LOG(i3d::detail) << " camPyr.size():" << camPyr->size() << " camPyr.area(0):" << camPyr->at(0).area;
    mIOWrapper = std::shared_ptr<IOWrapperRGBD>(new IOWrapperRGBD(mSettings.settingsIO,mSettings.settingsPyr,camPyr));
    I3D_LOG(i3d::detail) << "after wrapper thread!";
    if (mSettings.DO_OUTPUT_POSES && !mSettings.settingsIO.isFinished)
    {
        const std::string poseFileName("poses_"+mSettings.settingsIO.subDataset+".txt");
        mPoseFile.open((mSettings.outputFolder+poseFileName).c_str(),std::ios_base::out);
        if (!mPoseFile.is_open())
        {
            I3D_LOG(i3d::error) << "Could not open pose file for writing! Exiting!";
            exit(0);
        }
        I3D_LOG(i3d::info) << "Opened pose file: " << (mSettings.outputFolder+poseFileName).c_str();
    }
    isFinished = false;
    }
}

REVO::~REVO()
{
    if (mPoseFile.is_open()) mPoseFile.close();
#ifdef WITH_PANGOLIN_VIEWER
    if(mSettings.DO_USE_PANGOLIN_VIEWER && mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
        pangolin::DestroyWindow("REVO Map Viewer");
    }
#endif
    if (mThIOWrapper) mThIOWrapper->join();
    if (mThViewer) mThViewer->detach();
}
void REVO::writePose(const Eigen::Matrix3f &R, const Eigen::Vector3f &T, const double timeStamp)
{
    const Eigen::Quaternionf Qf(R);
    mPoseFile << std::fixed << timeStamp << " " << std::setprecision(9) << T[0] << " " << T[1] << " " << T[2] << " " <<  Qf.x() << " " << Qf.y() << " " << Qf.z() << " " << Qf.w() << std::endl;
}


//The system and the pose graph
bool REVO::start()
{
    if (isFinished) return false;
    if (!mIOWrapper->isInitSuccess())
    {
        I3D_LOG(i3d::error) << "IO not initialized properly or all datasets computed";
        return false;
    }
    //set nRuns to choose dataset!
    /* System start up*/
    if (mThIOWrapper==NULL)
    {
        mThIOWrapper = std::unique_ptr<std::thread>(new std::thread(&IOWrapperRGBD::generateImgPyramid, mIOWrapper));
    }
    //We need three different frames
    //current frame
    //previous frame
    //reference/key frame
    std::shared_ptr<ImgPyramidRGBD> kfPyr, prevPyr, currPyr;

    Eigen::Matrix4f T_NM1_N = Eigen::Matrix4f::Identity();
    //ImgPyramidRGBD prevPyr(mConfig.pyrConfig,mCamPyr);
    mTracker = std::unique_ptr<TrackerNew>(new TrackerNew(mSettings.settingsTracker,mSettings.settingsPyr));//,camPyr));
    /* System start up end*/
    /*Initializations*/
    uint noFrames = 0, noTrackingLost = 0;
    flagTrackingLost = false;
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f T(0,0,0);
    //Eigen::Matrix4f T_KF_NM1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f startPose = Eigen::Matrix4f::Identity();
    TrackerNew::TrackerStatus trackerStatus = TrackerNew::TRACKER_STATE_UNKNOWN;
    float error = INFINITY;
    bool justAddedNewKeyframe = false;
    int nKeyFrames = 0;
    /*Initializations end*/
    //ImgPyramidRGBD kfPyr(mConfig.pyrConfig,mCamPyr);
    //Eigen::Matrix4f initPose;
    std::vector<std::tuple<Eigen::Matrix4f,bool>,Eigen::aligned_allocator<Eigen::Matrix4f>> baPoseGraph;
#ifdef WITH_PANGOLIN_VIEWER
    while ((mpViewer && !this->mpViewer->quitRequest() && mSettings.DO_USE_PANGOLIN_VIEWER && mIOWrapper->hasMoreImages()) ||
           (mIOWrapper->hasMoreImages() && !mSettings.DO_USE_PANGOLIN_VIEWER))
#else
    while (mIOWrapper->hasMoreImages())
#endif
    {
        //wait for img pyramid
        if (!mIOWrapper->isImgPyramidAvailable())
        {
            I3D_LOG(i3d::detail) << "ImgPyramid not available!";
            usleep(3000);
            continue;
        }

        //Requesting image pyramid
        I3D_LOG(i3d::error) << "Requesting img pyramid";
        if (!mIOWrapper->getOldestPyramid(currPyr))
        {
            I3D_LOG(i3d::error) << "Error getting img pyramid";
            continue;
        }
        I3D_LOG(i3d::error) << "Got img pyramid";
        //Got image pyramid
        currPyr->frameId = noFrames;
        const double tumRefTimestamp = currPyr->returnTimestamp();
        I3D_LOG(i3d::info) << std::fixed << "tumRefTimestamp: " << tumRefTimestamp;
        if (noFrames == 0) //first frame -> keyframe
        {
            kfPyr = currPyr;
            prevPyr = currPyr;
            kfPyr->makeKeyframe();
            kfPyr->setTwf(Eigen::Matrix4f::Identity());
            kfPyr->prepareKfForStorage();
            mPoseGraph.push_back(Pose(Eigen::Matrix4f::Identity(),tumRefTimestamp,kfPyr));
            ++nKeyFrames;
            if (mSettings.DO_OUTPUT_POSES) writePose(startPose.block<3,3>(0,0),startPose.block<3,1>(0,3),tumRefTimestamp);
#ifdef WITH_PANGOLIN_VIEWER
            if (mSettings.DO_USE_PANGOLIN_VIEWER)
            {
                Eigen::MatrixXf pcl;
                kfPyr->generateColoredPcl(0,pcl,mSettings.DO_GENERATE_DENSE_PCL);
                mpMapDrawer->addPclAndKfPoseToQueue(pcl,Eigen::Matrix4f::Identity());
                mpMapDrawer->addKeyframePoseF(Eigen::Matrix4f::Identity());
            }
#endif
            ++noFrames;
            //justAddedNewKeyframe = true;
            justAddedNewKeyframe = true;
            mTracker->addOldPclAndPose(kfPyr->return3DEdges(mTracker->histogramLevel),Eigen::Matrix4f::Identity(),kfPyr->returnTimestamp());
            continue;
        }
        I3D_LOG(i3d::info) << std::fixed << "prevTime: " << prevPyr->returnTimestamp() << " currTime: " << currPyr->returnTimestamp();
        ++noFrames;
        if (mSettings.DO_SHOW_DEBUG_IMAGE)
        {
            int goodCount,badCount;
            reprojectPCLToImg(currPyr->return3DEdges(0),R,T,kfPyr->returnEdges(0),
                              camPyr->at(0).returnSize(),
                              kfPyr->returnK(0),
                              goodCount,badCount,"init");
        }
        clock_t start = clock();
        auto beginTracking = Timer::getTime();
        trackerStatus = mTracker->trackFrames(R,T,error,kfPyr,currPyr);
        clock_t end = clock();
        //positive cases
        Eigen::Matrix4f T_KF_N = transformFromRT(R,T);
        Eigen::Matrix4f currPoseInWorld = kfPyr->getTransKFtoWorld()*T_KF_N;
        //Eigen::Matrix4f currPoseInWorld = relativePoseToWorld(T_KF_N);//globPoseKf.back()*pose_rc;//.inverse();
        //now check the tracking results
        //there are basically three options:
        //- Everything ok
        //- New keyframe needed
        //- Tracking lost
        trackerStatus = mTracker->assessTrackingQuality(currPoseInWorld,currPyr);//(mSettings.CHECK_TRACKING_RESULTS ?  : TrackerNew::TRACKER_STATE_OK);
        //if tracking gets inaccurate, take the previous frame as keyframe and try to optimize again.
        //The idea is that the transformation between consecutive frames is more accurate
        I3D_LOG(i3d::detail) << "posegraph: " << mPoseGraph.back().getCurrToWorld();
        if (trackerStatus == TrackerNew::TRACKER_STATE_NEW_KF && !justAddedNewKeyframe)
        {
            //make previous keyframe!
            //kfPyr.swap(prevPyr);
            kfPyr = prevPyr;
            kfPyr->setTwf(mPoseGraph.back().getCurrToWorld());
            kfPyr->makeKeyframe();
            mPoseGraph.back().setKfFrame(kfPyr);;
            nKeyFrames++;
            mTracker->clearUpPastLists();
            //now, retrack
            R = T_NM1_N.block<3,3>(0,0);
            T = T_NM1_N.block<3,1>(0,3);
            if (mSettings.DO_SHOW_DEBUG_IMAGE)
            {
                int goodCount,badCount;
                reprojectPCLToImg(currPyr->return3DEdges(0),R,T,kfPyr->returnEdges(0),
                                  camPyr->at(0).returnSize(),
                                  kfPyr->returnK(0),
                                  goodCount,badCount,"init with new kf");
            }
            //track again
            mTracker->trackFrames(R,T,error,kfPyr,currPyr);
            T_KF_N = transformFromRT(R,T);
            //T_KF_NM1 = Eigen::Matrix4f::Identity();
            currPoseInWorld = kfPyr->getTransKFtoWorld()*T_KF_N;//relativePoseToWorld(T_KF_N);//kfPyr->getTransKFtoWorld()*T_KF_N;;

            trackerStatus = mTracker->assessTrackingQuality(currPoseInWorld,currPyr);
#ifdef WITH_PANGOLIN_VIEWER
            if (mSettings.DO_USE_PANGOLIN_VIEWER)
            {
                Eigen::MatrixXf pcl;
                kfPyr->generateColoredPcl(0,pcl,mSettings.DO_GENERATE_DENSE_PCL);
                mpMapDrawer->addPclAndKfPoseToQueue(pcl,kfPyr->getTransKFtoWorld());//mKeyframes.back()->getTransKFtoWorld());
                mpMapDrawer->addKeyframePoseF(kfPyr->getTransKFtoWorld());
            }
#endif
            justAddedNewKeyframe = true;
        }
        else
            justAddedNewKeyframe = false;
        auto endTracking = Timer::getTime();
        I3D_LOG(i3d::warning) << "Tracking time: " << double(end-start)/CLOCKS_PER_SEC*1000.0 << " vs " << double(std::chrono::duration_cast<std::chrono::microseconds>(endTracking-beginTracking).count())/1000.0;
        trackingTimes.push_back(double(Timer::getTimeDiffMs(beginTracking,endTracking)));
        if (mSettings.DO_SHOW_DEBUG_IMAGE)
        {
            int goodCount,badCount;
            reprojectPCLToImg(currPyr->return3DEdges(0),R,T,kfPyr->returnEdges(0),
                              camPyr->at(0).returnSize(),
                              kfPyr->returnK(0),
                              goodCount,badCount,"after tracking");
        }

        flagTrackingLost = false;
        //add good frame to pose graph
        mPoseGraph.push_back(Pose(T_KF_N,currPyr->returnTimestamp(),kfPyr));
        mTracker->addOldPclAndPose(currPyr->return3DEdges(mTracker->histogramLevel),currPoseInWorld,currPyr->returnTimestamp());
        //Description:
        //It's easy, when you think about it:
        //[KF] [1] [2] [3] [4]
        //T_KF_1_init = I
        //T_KF_2_init = T_KF_1*T_KF_1
        //T_KF_3_init = T_KF_2*T_1_2 = T_KF_2 * inv(T_KF_1) * T_KF_2
        //relative camera motion between current frame N and the frame before N-1
        T_NM1_N = mPoseGraph.at(mPoseGraph.size()-2).T_N_W() * mPoseGraph.back().T_W_N();
        I3D_LOG(i3d::detail) << "T_kf_N" << T_KF_N << " vs " << T_NM1_N << " " << mPoseGraph.back().T_W_N() << " " << mPoseGraph.back().T_kf_N();
        const Eigen::Matrix4f T_init = mPoseGraph.back().T_kf_N()*T_NM1_N;
        R = T_init.block<3,3>(0,0);
        T = T_init.block<3,1>(0,3);
        Eigen::Matrix4f absPose = mPoseGraph.back().getCurrToWorld();//startPose*currPoseInWorld;
        I3D_LOG(i3d::debug) <<"MY abs pose: " << poseToTUMString(absPose.cast<double>(),tumRefTimestamp);
        //I3D_LOG(i3d::debug) << "TUM abs pose: "<< poseToTUMString(mTumReader->getAbsPose(tumRefTimestamp),tumRefTimestamp);
        if (mSettings.DO_OUTPUT_POSES) writePose(absPose.block<3,3>(0,0),absPose.block<3,1>(0,3),tumRefTimestamp);
#ifdef WITH_PANGOLIN_VIEWER
        if (mSettings.DO_USE_PANGOLIN_VIEWER)
        {
            mpMapDrawer->addFramePoseF(currPoseInWorld);
            mpMapDrawer->SetCurrentCameraPose(currPoseInWorld.inverse());
        }
#endif
        prevPyr = currPyr;
    }
    mIOWrapper->setFinish(true);
#ifdef WITH_PANGOLIN_VIEWER
    while (mpViewer && !this->mpViewer->quitRequest() && mSettings.DO_USE_PANGOLIN_VIEWER)
    {
        usleep(3000);
    }
#endif
    I3D_LOG(i3d::info) << "-----VO Report-----";
    I3D_LOG(i3d::info) << "Frames Tracked: " << mPoseGraph.size();
    I3D_LOG(i3d::info) << "Keyframes Tracked: " << nKeyFrames;
    I3D_LOG(i3d::info) << "Tracking Lost: " << noTrackingLost;
    double meandT = 0.0;
    for (size_t i=0;i < ImgPyramidRGBD::dtTimes.size();++i)
        meandT+=ImgPyramidRGBD::dtTimes[i];

    I3D_LOG(i3d::info) << "Distance Transform: " << meandT/ImgPyramidRGBD::dtTimes.size();
    double sum = std::accumulate(trackingTimes.begin(), trackingTimes.end(), 0.0);
    double mean = sum / trackingTimes.size();
    I3D_LOG(i3d::info) << "Mean Tracking Time: " << mean;
    return true;
}

float REVO::reprojectPCLToImg(const Eigen::MatrixXf& pcl, const Eigen::Matrix3f& R, const Eigen::Vector3f& T,
                        const cv::Mat& img, const cv::Size2i& size,
                        const Eigen::Matrix3f& K, int& goodCount, int& badCount, const std::string& title = "") const
{
    const float fx = K(0,0), fy = K(1,1);
    const float cx = K(0,2), cy = K(1,2);
    float totalCost = 0;
    goodCount = 0;
    badCount = 0;
    cv::Mat imgTmp = img.clone();

    for( int ir=0 ; ir<pcl.cols() ; ir++ )
    {
        const Eigen::VectorXf pt = pcl.col(ir).head<3>();
        Eigen::VectorXf newPt = R*pt+T;
        //std::cout << pt << std::endl;
        newPt[0] = fx * newPt[0]/newPt[2] + cx;
        newPt[1] = fy * newPt[1]/newPt[2] + cy;
        if (newPt[0] >= 0 && newPt[0] < size.width && newPt[1] >= 0 && newPt[1] < size.height)
        {
            imgTmp.at<uint8_t>(floor(newPt[1]),floor(newPt[0])) = 100;
        }
    }
    cv::imshow(title,imgTmp);
    cv::waitKey(0);
    return totalCost;
}

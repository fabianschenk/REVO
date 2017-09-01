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
#include <Eigen/Eigen>

#include <thread>
#include <mutex>
#include "optimizer.h"
#include "../utils/Logging.h"
#include "../datastructures/imgpyramidrgbd.h"
#include <queue>
#include "../utils/timer.h"

class TrackerSettings
{
public:
    TrackerSettings(const std::string& settingsFile)//distThreshold(0.1),angleThreshold(5),distThresholdTrackingLost(0.5), angleThresholdTrackingLost(15)
    {
        cv::FileStorage fs(settingsFile,cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            I3D_LOG(i3d::error) << "Couldn't open settings file at location: " << settingsFile;
            exit(0);
        }
        //image pyramid settings (camera matrix, resolutions,...)
        cv::read(fs["CHECK_INIT_VALUES"],CHECK_INIT_VALUES,true);
        cv::read(fs["nFramesHistogramVoting"],nFramesHistogramVoting,3);
        cv::read(fs["CHECK_TRACKING_RESULTS"],CHECK_TRACKING_RESULTS,true);
        cv::read(fs["USE_EDGE_FILTER"],optimizerSettings.USE_EDGE_FILTER,true);
        cv::read(fs["N_FRAMES_HIST_VOTING"],nFramesHistogramVoting,3);
        fs.release();

    }
    bool CHECK_TRACKING_RESULTS;
    bool CHECK_INIT_VALUES;
    OptimizerSettings optimizerSettings;
    int nFramesHistogramVoting;
};
class TrackerNew
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setReference(const std::shared_ptr<ImgPyramidRGBD> &ref);
    enum TrackerStatus{
        TRACKER_STATE_OK,
        TRACKER_STATE_LOST, //this should switch to relocalization, which is currently not implemented
        TRACKER_STATE_NEW_KF,
        TRACKER_STATE_UNKNOWN
    };
    int histogramLevel;
    //tracks or relocalizes
    TrackerStatus trackFrames(Eigen::Matrix3f &R, Eigen::Vector3f &T, float &error,
                              const std::shared_ptr<ImgPyramidRGBD> &refFrame, const std::shared_ptr<ImgPyramidRGBD> &currFrame);
    TrackerNew(const TrackerSettings& config, const ImgPyramidSettings &pyrConfig);
    ~TrackerNew();
    const Eigen::MatrixXf &returnPcl() const;
    void clearUpPastLists();

    //float performEdgeVoting(const Eigen::Matrix4f &estimatedPose) const;
    TrackerStatus assessTrackingQuality(const Eigen::Matrix4f& estimatedPose, const std::shared_ptr<ImgPyramidRGBD> &mCurrFrame);

    //stores the point cloud for tracking quality assessment
    void addOldPclAndPose(const Eigen::MatrixXf& pcl, const Eigen::Matrix4f& worldPose, const double timeStamp);
    //const void returnColoredPcl(Eigen::MatrixXf& clrPcl, const Eigen::Matrix4f globPose) const;
private:
    float evalCostFunction(const Eigen::Matrix3f &R, const Eigen::Vector3f &T, uint minLvl, const std::shared_ptr<ImgPyramidRGBD> &mCurrFrame, const std::shared_ptr<ImgPyramidRGBD> &mRefFrame) const;
    void checkInitializationValues(Eigen::Matrix3f& R, Eigen::Vector3f& T,
                                   const std::shared_ptr<ImgPyramidRGBD>& mCurrFrame, const std::shared_ptr<ImgPyramidRGBD>& mRefFrame) const;

    //used for voting scheme
    //counting map M, M_i as described in IROS Paper
    cv::Mat M, M_i;
    std::vector<float> histWeights;
    //these are the pcl in the world coordinates for quality assessment
    std::deque<Eigen::MatrixXf> mPastPcl;
    ////that would mean, we only have transform them with the inverse transformation!
    std::deque<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> mPastWorldPoses;
    std::deque<double> mPastTimeStamps;
    std::mutex mTrackerLock;
    const TrackerSettings mSettings;
    const ImgPyramidSettings mPyrConfig; //this is a bit of a hack!
    TrackerStatus mTrackerStatus;
    Optimizer mOptimizer;
    //debugging methods
    void reprojectRefEdgesToCurrentFrame(const cv::Mat& rgbCurr,const Eigen::MatrixXf& _3d, const Eigen::Matrix3f& K,
                                            const Eigen::Matrix3f& R, const Eigen::Vector3f &t, const cv::Mat& edgesCurr,
                                            const std::string title="") const;
};

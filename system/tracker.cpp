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
#include "tracker.h"
#include <iomanip>
#include <opencv2/highgui.hpp>
/// DEBUG FUNCTIONS START
void TrackerNew::reprojectRefEdgesToCurrentFrame(const cv::Mat& rgbCurr,const Eigen::MatrixXf& _3d, const Eigen::Matrix3f& K,
                                        const Eigen::Matrix3f& R, const Eigen::Vector3f &t, const cv::Mat& edgesCurr,
                                        const std::string title) const
{
    const float fx = K(0,0), fy = K(1,1);
    const float cx = K(0,2), cy = K(1,2);
    cv::Mat edgesInCurrRgb,edgesRefCurr;
    if (rgbCurr.channels()>1)
        rgbCurr.copyTo(edgesInCurrRgb);
    else//gray
    {
        std::vector<cv::Mat> channelsRGB;
        channelsRGB.push_back(rgbCurr);
        channelsRGB.push_back(rgbCurr);
        channelsRGB.push_back(rgbCurr);
        cv::merge(channelsRGB,edgesInCurrRgb);
    }
    //edgesCurr.copyTo(edgesRefCurr);
    std::vector<cv::Mat> channels;
    channels.push_back(edgesCurr);
    channels.push_back(edgesCurr);
    channels.push_back(edgesCurr);
    cv::merge(channels,edgesRefCurr);
    for( int ir=0 ; ir<_3d.cols() ; ir++ )
    {
        Eigen::VectorXf pt = _3d.col(ir);
        Eigen::VectorXf res =  R*pt.head<3>()+t;
        res[0] = fx * res[0]/res[2] + cx;
        res[1] = fy * res[1]/res[2] + cy;

        if (res[0] >= 0 && res[0] < rgbCurr.cols && res[1] >= 0 && res[1] < rgbCurr.rows)
        {
            const cv::Vec3b clr = cv::Vec3b(0,255,0);
            edgesInCurrRgb.at<cv::Vec3b>(res[1],res[0]) = clr;
            edgesRefCurr.at<cv::Vec3b>(res[1],res[0]) = clr;
        }
    }
    cv::imshow("edgesInCurrRgb "+title,edgesInCurrRgb);
    cv::imshow("edgesRefCurr "+title,edgesRefCurr);
    I3D_LOG(i3d::info) << "Before write!";
    //cv::imwrite("edgesInCurrRgb_"+title+".png",edgesInCurrRgb);
    //cv::imwrite("edgesRefCurr_"+title+".png",edgesRefCurr);
    cv::waitKey(0);
    return;// edgesInCurrRgb;
}
cv::Mat reprojectRefEdgesToDistTransform(const cv::Mat& dtC,const Eigen::MatrixXf& _3d, const Eigen::Matrix3f& K,
                                        const Eigen::Matrix3f& R, const Eigen::Vector3f &t, const cv::Mat& edgesCurr,
                                        const std::string title = "", bool USE_FORWARD = true)
{
    const float fx = K(0,0), fy = K(1,1);
    const float cx = K(0,2), cy = K(1,2);
    I3D_LOG(i3d::info) << "reprojectRefEdgesToDistTransform!";
    cv::Mat dt;
    cv::normalize(dtC,dt,0.0,255.0,cv::NORM_MINMAX);
    dt.convertTo(dt,CV_8UC1);
    std::vector<cv::Mat> dt3;
    cv::Mat dt_new;
    dt3.push_back(dt);dt3.push_back(dt);dt3.push_back(dt);
    cv::merge(dt3,dt_new);
    for( int ir=0 ; ir<_3d.cols() ; ir++ )
    {
        Eigen::VectorXf pt = _3d.col(ir);
        //std::cout << pt << std::endl;

        //Eigen::VectorXd res = R.transpose()*(pt-t);
        Eigen::VectorXf res;
        if (USE_FORWARD)
            res =  R*pt+t;
        else
            res = R.transpose()*(pt-t);
        res[0] = fx * res[0]/res[2] + cx;
        res[1] = fy * res[1]/res[2] + cy;

        if (res[0] >= 0 && res[0] < dt.cols && res[1] >= 0 && res[1] < dt.rows)
        {
            dt_new.at<cv::Vec3b>(res[1],res[0]) = cv::Vec3b(0,255,0);
        }
    }
    cv::imshow("edgesInDt "+title,dt_new);
    cv::imwrite("edgesInDt.png",dt_new);
    cv::waitKey(0);
    return dt_new;
}
/// DEBUG FUNCTIONS END

//
/**
 * @brief TrackerNew::assessTrackingQuality We project old pcls in the current frame using the estimated pose.
 * If the quality is not good enough, we take the previous one as keyframe and re-estimate.
 * See Schenk and Fraundorfer, IROS 2017 paper, for a more detailed description
 * @param estimatedPose
 * @param mCurrFrame
 * @return
 */
TrackerNew::TrackerStatus TrackerNew::assessTrackingQuality(const Eigen::Matrix4f& estimatedPose,
                                                            const std::shared_ptr<ImgPyramidRGBD>& mCurrFrame)
{
    if (mPastPcl.size()!= mPastWorldPoses.size() || mPastPcl.size() == 0 || !mSettings.CHECK_TRACKING_RESULTS) return TRACKER_STATE_OK;
    const cv::Mat currEdges = mCurrFrame->returnOrigEdges(histogramLevel).clone();
    const cv::Mat currDepth = mCurrFrame->returnDepth(histogramLevel).clone();
    //generate counting maps M_i and M. M = M_1 + ... + M_N
    if (M.empty())
        M = cv::Mat(currEdges.rows,currEdges.cols,CV_8UC1,cv::Scalar(0));
    else
        M.setTo(0);
    if (M_i.empty())
        M_i = cv::Mat(currEdges.rows,currEdges.cols,CV_8UC1,cv::Scalar(0));
    const Camera cam = mCurrFrame->cameraPyr->at(histogramLevel);
    std::vector<int> histogram, overlaps;
    histogram.push_back(0);//number of overlaps with 0,1,2,3 nFrames;
    overlaps.push_back(0);
    //for debugging
    int outOfBounds = 0;
    I3D_LOG(i3d::info) << "mPastPcl.size(): " << mPastPcl.size();
    for (size_t frame = 0; frame <  mSettings.nFramesHistogramVoting && frame < mPastPcl.size(); ++frame)
    {
        histogram.push_back(0);
        overlaps.push_back(0);
        const Eigen::Matrix4f transform = estimatedPose.inverse()*mPastWorldPoses.at(frame);//mPastWorldPoses.at(frame).inverse()*estimatedPose;
        const Eigen::Matrix3f R = transform.block<3,3>(0,0);
        const Eigen::Vector3f T = transform.block<3,1>(0,3);
        const Eigen::MatrixXf _3dPts = mPastPcl.at(frame);
        const size_t pclSize = _3dPts.cols();
        M_i.setTo(0);
        //now perform the projection
        for( size_t ir=0 ; ir < pclSize ; ir++ )
        {
            const Eigen::Vector3f pt = _3dPts.col(ir).head<3>();
            Eigen::Vector3f newPt = R*pt+T;
            newPt[0] = cam.fx * newPt[0]/newPt[2] + cam.cx;
            newPt[1] = cam.fy * newPt[1]/newPt[2] + cam.cy;
            if (newPt[0] >= 0 && newPt[0] < cam.width && newPt[1] >= 0 && newPt[1] < cam.height)
                M_i.at<uint8_t>(floor(newPt[1]),floor(newPt[0])) = 1; //prevent coinciding reprojections
            else
                ++outOfBounds;
        }
        cv::add(M_i,M,M);
    }
    float overlapMeasure = 0.0f;
    I3D_LOG(i3d::info) << "outOfBounds: " << outOfBounds ;
    I3D_LOG(i3d::info) << histogram.size() << " " << overlaps.size() << " " << mCurrFrame->frameId;

    for (int xx = 0; xx < currEdges.cols;++xx)
        for (int yy = 0; yy < currEdges.rows;++yy)
        {
            const float Z = currDepth.at<float>(yy,xx);
            if (mCurrFrame->isPointOkDepth(Z))
            {
                const int val = M.at<uint8_t>(yy,xx);
                histogram.at(val) = histogram.at(val)+1;
                if (currEdges.at<uint8_t>(yy,xx) > 0) overlaps.at(val) = overlaps.at(val) + 1;
            }
        }
    for (size_t histLvl = 0; histLvl < histogram.size();++histLvl)
    {
        if (histLvl>0) overlapMeasure+= (overlaps.at(histLvl)*histWeights.at(histLvl));
        I3D_LOG(i3d::info) << "histLvl: " << histLvl << " total " << histogram.at(histLvl) << " overlap: " << overlaps.at(histLvl)
                           << "percentage: " << float(overlaps.at(histLvl))/histogram.at(histLvl);
    }
    I3D_LOG(i3d::info) << "overlapMeasure: " <<overlapMeasure;
    if (overlapMeasure>=overlaps.at(0) || histogram.size() < 4)// || overlaps.at(histogram.size()-1) > overlaps.at(1)*0.5)
    {
        I3D_LOG(i3d::info) << "No new keyframe!";
        //output reprojections
        cv::imwrite("out/M_"+std::to_string(mCurrFrame->frameId)+"_no.png",M);
        cv::imwrite("out/currEdges"+std::to_string(mCurrFrame->frameId)+"_no.png",currEdges);
        cv::imwrite("out/currDepth"+std::to_string(mCurrFrame->frameId)+"_no.png",currDepth);

        return TRACKER_STATE_OK;
    }

    I3D_LOG(i3d::info) << "New keyframe!";
    //output reprojections
    cv::imwrite("out/M_"+std::to_string(mCurrFrame->frameId)+"_new.png",M);
    cv::imwrite("out/currEdges"+std::to_string(mCurrFrame->frameId)+"_new.png",currEdges);
    cv::imwrite("out/currDepth"+std::to_string(mCurrFrame->frameId)+"_new.png",currDepth);
    return TRACKER_STATE_NEW_KF;
}

/**
 * @brief TrackerNew::addOldPclAndPose Addes old pcl to list for tracking quality assessment
 * @param pcl point cloud to be added
 * @param worldPose
 * @param timeStamp
 */
void TrackerNew::addOldPclAndPose(const Eigen::MatrixXf& pcl, const Eigen::Matrix4f& worldPose, const double timeStamp)
{
    //delete first one if larger than nFramesHistogramVoting
//    if (mPastPcl.size() > mConfig.nFramesHistogramVoting && false)
//    {
//        mPastPcl.pop_front();
//        mPastWorldPoses.pop_front();
//        mPastTimeStamps.pop_front();
//    }
    I3D_LOG(i3d::info) << std::fixed << "Adding: " << timeStamp;
    mPastPcl.push_back(pcl);
    mPastWorldPoses.push_back(worldPose);
    mPastTimeStamps.push_back(timeStamp);

}

TrackerNew::TrackerNew(const TrackerSettings& config, const ImgPyramidSettings& pyrConfig)://, const std::shared_ptr<CameraPyr> camPyr):
    mSettings(config), mPyrConfig(pyrConfig), mTrackerStatus(TRACKER_STATE_UNKNOWN), mOptimizer(config.optimizerSettings)
{
    I3D_LOG(i3d::info) << "Constructing Tracker!";
    histogramLevel = 2;
    //weights for overlap histogram
    histWeights.push_back(0);
    histWeights.push_back(1);
    histWeights.push_back(1.25);
    histWeights.push_back(1.5);
}
TrackerNew::~TrackerNew()
{
    //Free resources
    mPastTimeStamps.clear();
    mPastPcl.clear();
    mPastWorldPoses.clear();

}
/**
 * @brief TrackerNew::clearUpPastLists clears the pcl lists used for tracking quality assessment
 */

void TrackerNew::clearUpPastLists()
{
    while (mPastPcl.size() > mSettings.nFramesHistogramVoting)
    {
        I3D_LOG(i3d::info) << std::fixed << "Deleting: " << mPastTimeStamps.front();
        mPastPcl.pop_front();
        mPastWorldPoses.pop_front();
        mPastTimeStamps.pop_front();
    }
}
/**
 * @brief TrackerNew::checkInitializationValues Evaluates the cost function at at mRefFrame using the 3D PCl from mCurrFrame R * PCL_curr * T
 * @param R Rotation from curr to ref
 * @param T Translation from curr to ref
 * @param mCurrFrame current frame
 * @param mRefFrame reference frame
 */
void TrackerNew::checkInitializationValues(Eigen::Matrix3f &R, Eigen::Vector3f &T,
                                           const std::shared_ptr<ImgPyramidRGBD>& mCurrFrame, const std::shared_ptr<ImgPyramidRGBD>& mRefFrame) const
{
    if(!mSettings.CHECK_INIT_VALUES) return;
    const uint minLvl = mCurrFrame->returnMinLvl();
    clock_t start = clock();
    //check the costs at the lowest level to save speed!
    const float costEye = evalCostFunction(Eigen::Matrix3f::Identity(),Eigen::Vector3f::Zero(),minLvl,mCurrFrame,mRefFrame);
    const float costInit = evalCostFunction(R,T,minLvl,mCurrFrame,mRefFrame);
    clock_t end = clock();
    I3D_LOG(i3d::info) << "checkInitializationValues: " << double(end-start)/CLOCKS_PER_SEC*1000.0f;
    I3D_LOG(i3d::info) << "CostEye:" << costEye << " vs CostInit: " << costInit;
    if (costEye < costInit)
    {
        I3D_LOG(i3d::error) << "DO NOT INIT WITH PREVIOUS TRANSFORM!" << costEye << " " << costInit;
        R.setIdentity();
        T.setZero();
    }
}

/**
 * @brief TrackerNew::trackFrames Estimates the rotation R and translation T from currFrame to refFrame
 * @param R starts with value given and returns new estimate
 * @param T starts with value given and returns new estimate
 * @param error estimated error
 * @param mCurrFrame current frame
 * @param mRefFrame reference frame
 * @return
 */
TrackerNew::TrackerStatus TrackerNew::trackFrames(Eigen::Matrix3f &R, Eigen::Vector3f &T, float& error,
                                                  const std::shared_ptr<ImgPyramidRGBD>& refFrame, const std::shared_ptr<ImgPyramidRGBD>& currFrame)
{
    if (mSettings.optimizerSettings.DO_SHOW_DEBUG_IMAGES)
    {
        cv::imshow("currgray",currFrame->returnGray(0));
        cv::imshow("refgray",refFrame->returnGray(0));
        cv::imshow("curredges",currFrame->returnEdges(0));
        cv::imshow("refedges",refFrame->returnEdges(0));
        const cv::Mat& depth = currFrame->returnDepth(0);
        double min,max;
        cv::minMaxIdx(depth,&min,&max);
        cv::imshow("depth",depth/max);
        reprojectRefEdgesToCurrentFrame(refFrame->returnGray(0),currFrame->return3DEdges(0),currFrame->returnK(0),R,T,refFrame->returnEdges(0),"init");
        reprojectRefEdgesToDistTransform(refFrame->returnDistTransform(0),currFrame->return3DEdges(0),currFrame->returnK(0),R,T,refFrame->returnEdges(0));
        //reprojectRefEdgesToCurrentFrame(mRefFrame->returnDistTransform(0),mCurrFrame->return3DEdges(0),mCurrFrame->returnK(0),R,T,mRefFrame->returnEdges(0),"init");
    }
    I3D_LOG(i3d::info) <<  "Current Frame: " << currFrame->frameId;
    //std::unique_lock<std::mutex> lock(mTrackerLock);
    auto beginInit = Timer::getTime();
    checkInitializationValues(R,T,currFrame,refFrame);
    auto endInit = Timer::getTime();
    I3D_LOG(i3d::info) << "checkInitializationValues: " << Timer::getTimeDiffMiS(beginInit,endInit) << "mis";
    error = INFINITY;
    //std::vector<std::shared_ptr<ImgPyramidRGBD>> testVec;
    //testVec.push_back(currFrame);
    //Eigen::Matrix3d R_d = R.cast<double>();
    //Eigen::Vector3d T_d = T.cast<double>();
    Optimizer::ResidualInfo resInfo;
    //for (int lvl = 0; lvl >= mPyrConfig.maxLevel;--lvl)
    for (int lvl = mPyrConfig.PYR_MIN_LVL; lvl >= mPyrConfig.PYR_MAX_LVL;--lvl)
    {

        I3D_LOG(i3d::warning) << "----LEVEL----" << lvl;// << " curr: " << std::fixed << currFrame->returnTimestamp() << " to " << refFrame->returnTimestamp();
        I3D_LOG(i3d::detail) << "Rinit: " << R;
        I3D_LOG(i3d::detail) << "tinit: " << T.transpose();
        //perform tracking
       // auto startAcc = Timer::getTime();
        //mOptimizer.buildAccelerationStructure(refFrame->returnDistTransform(lvl),lvl);
        //auto endAcc = Timer::getTime();
        //I3D_LOG(i3d::info) << "Building acc structure at lvl: "<<lvl<<": " << Timer::getTimeDiffMs(startAcc,endAcc);
        auto startT = Timer::getTime();
        resInfo.clearAll();
        error = mOptimizer.trackFrames(refFrame,currFrame,R,T,lvl,resInfo);
        auto endT = Timer::getTime();
        I3D_LOG(i3d::info) << "Tracking time for lvl "<<lvl<<": " << Timer::getTimeDiffMiS(startT,endT) << "mis";
    }
    //R = R_d.cast<float>();
    //T = T_d.cast<float>();
    if (mSettings.optimizerSettings.DO_SHOW_DEBUG_IMAGES)
    {
        reprojectRefEdgesToCurrentFrame(refFrame->returnGray(0),currFrame->return3DEdges(0),currFrame->returnK(0),R,T,refFrame->returnEdges(0), "after");
    }
    I3D_LOG(i3d::error) << "FINAL AVG ERROR: " << error << " goodPtsEdge: " << resInfo.goodPtsEdges << "badPtsEdge: " << resInfo.badPtsEdges << " good/bad = "
                        << double(resInfo.goodPtsEdges)/double(resInfo.badPtsEdges) << " good / total"
                        << double(resInfo.goodPtsEdges)/double(resInfo.badPtsEdges+resInfo.goodPtsEdges);
    I3D_LOG(i3d::error) << "Final R: " << R << " final t: " << T;
    if  (double(resInfo.goodPtsEdges)/double(resInfo.badPtsEdges) < 4) return TrackerNew::TRACKER_STATE_NEW_KF;
    return TrackerNew::TRACKER_STATE_OK;
}


//Evaluate the cost at the lowest level to speed up computation
float TrackerNew::evalCostFunction(const Eigen::Matrix3f& R, const Eigen::Vector3f& T, uint minLvl,
                                   const std::shared_ptr<ImgPyramidRGBD>& mCurrFrame, const std::shared_ptr<ImgPyramidRGBD>& mRefFrame) const
{
    const Eigen::Matrix3f K = mCurrFrame->returnK(minLvl);
    const float fx = K(0,0), fy = K(1,1);
    const float cx = K(0,2), cy = K(1,2);
    float totalCost = 0;
    //I3D_LOG(i3d::info) << "K: " << K;
    const Eigen::MatrixXf _3d = mCurrFrame->return3DEdges(minLvl);
    const cv::Size2i size = mCurrFrame->cameraPyr->at(minLvl).returnSize();
    cv::Mat distanceTransform= mRefFrame->returnDistTransform(minLvl);
    double min,max;
    cv::minMaxIdx(distanceTransform,&min,&max);
    cv::imwrite("dist_trans.png",distanceTransform/max*255);
    for( int ir=0 ; ir<_3d.cols() ; ir++ )
    {
        const Eigen::Vector3f pt = _3d.col(ir).head<3>();
        Eigen::Vector3f newPt = R*pt+T;
        int badCount = 0, goodCount=0;
        newPt[0] = fx * newPt[0]/newPt[2] + cx;
        newPt[1] = fy * newPt[1]/newPt[2] + cy;
        //I3D_LOG(i3d::info) << ir << ": " << newPt.transpose();
        if (newPt[0] >= 0 && newPt[0] < size.width && newPt[1] >= 0 && newPt[1] < size.height)
        {
            const float residual = distanceTransform.at<float>(floor(newPt[1]),floor(newPt[0]));
            if (residual > mSettings.optimizerSettings.edgeDistanceLvl[minLvl] && mSettings.optimizerSettings.USE_EDGE_FILTER)
            {
                badCount++;
                continue;
            }
            goodCount++;
            totalCost+=distanceTransform.at<float>(floor(newPt[1]),floor(newPt[0]));
        }
    }
    //exit(0);
    return totalCost;
}

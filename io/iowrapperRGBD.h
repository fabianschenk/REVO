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
#include "../datastructures/imgpyramidrgbd.h"
#include <opencv2/opencv.hpp>
#include <queue>
#include <mutex>
#include <memory>
#include "../utils/timer.h"
#include <fstream>
#ifdef WITH_ORBBEC_ASTRA_PRO
    #ifdef WITH_ORBBEC_FFMPEG
        #include "../orbbec_astra_pro/OrbbecAstraEngineFFMPEG.h"
    #else
        #include "../orbbec_astra_pro/OrbbecAstraEngineUVC.h"
    #endif
#endif
#ifdef WITH_REALSENSE
    #include "realsensesensor.h"
#endif
#ifdef WITH_ORBBEC_ASTRA
    #include "../orbbec_astra_pro/OrbbecAstraOpenNIEngine.h"
#endif
class IOWrapperSettings
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IOWrapperSettings(const std::string& settingsFile, int nRuns = 0)
    {
        I3D_LOG(i3d::info) << "nRuns: " << nRuns;
        isFinished = false;
        cv::FileStorage fs(settingsFile,cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            I3D_LOG(i3d::error) << "Couldn't open settings file at location: " << settingsFile;
            exit(0);
        }
        //image pyramid settings (camera matrix, resolutions,...)
        int inputType; //0 = dataset, 1 = ASTRA PRO, 2 = REAL SENSE
        cv::read(fs["INPUT_TYPE"],inputType,0);
        READ_FROM_ASTRA_PRO = READ_FROM_REALSENSE = READ_FROM_ASTRA = false;
        switch (inputType) {
        case 1: READ_FROM_ASTRA_PRO = true;
                if (nRuns > 0) isFinished = true;
                break;
        case 2: READ_FROM_REALSENSE = true;
                if (nRuns > 0) isFinished = true;
                break;
        case 3: READ_FROM_ASTRA = true;
                if (nRuns > 0) isFinished = true;
                break;
        //dataset
        case 0:
        default:
            if (!fs.isOpened())
            {
                I3D_LOG(i3d::error) << "Couldn't open settings file at location: " << settingsFile;
                exit(0);
            }
            //now read all the settings
            //general settings
            //datasets or sensor?
            cv::FileNode n = fs["Datasets"];                         // Read string sequence - Get node
            if (n.type() == cv::FileNode::SEQ)
            {
                cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
                for (; it != it_end; ++it)
                    datasets.push_back((std::string)*it);
            }
            else
            {
                I3D_LOG(i3d::error) << "Datasets is not a string sequence! Trying to read string!";
                datasets.push_back(fs["Datasets"]);
            }
            I3D_LOG(i3d::info) << "Datasets: " << datasets.size() << "nRuns: "<<nRuns;
            if (nRuns >= int(datasets.size()))
            {
                I3D_LOG(i3d::info) << "Finished all datasets!";
                isFinished = true;
                subDataset = datasets.back();
            }
            else
                subDataset = datasets[nRuns];
            break;
        }
        //Ensure that it stops!
        if ((READ_FROM_ASTRA_DATA || READ_FROM_REALSENSE) && nRuns>0) isFinished = true;
        //TODO: make that work?
        cv::read(fs["READ_INTRINSICS_FROM_SENSOR"],READ_INTRINSICS_FROM_SENSOR,false);
        //only needed to skip auto exposure artifacts
        cv::read(fs["SKIP_FIRST_N_FRAMES"],SKIP_FIRST_N_FRAMES,0);
        cv::read(fs["READ_N_IMAGES"],READ_N_IMAGES,100000); //read at least 100000 images
        //cv::read(fs["DO_ADAPT_CANNY_VALUES"],DO_ADAPT_CANNY_VALUES, true); //tries to guess the Canny values from the first frame!
        cv::read(fs["DO_WAIT_AUTOEXP"],DO_WAIT_AUTOEXP, false); //skips the first 20 frames or so to avoid auto exposure problems
        if (SKIP_FIRST_N_FRAMES < 20 && DO_WAIT_AUTOEXP)
        {
            SKIP_FIRST_N_FRAMES = 20;
            I3D_LOG(i3d::warning) << "Skipping first 20 frames to avoid auto exposure problems!";
        }
        if (!READ_INTRINSICS_FROM_SENSOR)
        {
            cv::read(fs["DEPTH_SCALE_FACTOR"],DEPTH_SCALE_FACTOR,1000.0f);
            cv::read(fs["Camera.width"],imgSize.width,640);
            cv::read(fs["Camera.height"],imgSize.height,480);
        }
        cv::read(fs["DO_RECORD_IMAGES"],DO_OUTPUT_IMAGES,false);
        associateFile = (std::string)fs["ASSOCIATE"];
        if (associateFile.compare("")) associateFile = "associate.txt";
        cv::read(fs["useDepthTimeStamp"],useDepthTimeStamp,true);
        MainFolder = (std::string)fs["MainFolder"];
        I3D_LOG(i3d::info) << "MainFolder: " << MainFolder;
        fs.release();
    }
    bool inline READ_FROM_DATASET() const
    {
        return !(READ_FROM_ASTRA_PRO || READ_FROM_REALSENSE);
    }
    bool READ_INTRINSICS_FROM_SENSOR;
    bool READ_FROM_ASTRA_PRO;
    bool READ_FROM_REALSENSE;
    bool READ_FROM_ASTRA;
    int SKIP_FIRST_N_FRAMES;
    int READ_N_IMAGES;
    float DEPTH_SCALE_FACTOR;
    bool useDepthTimeStamp;
    //bool DO_ADAPT_CANNY_VALUES;
    bool DO_WAIT_AUTOEXP;
    std::vector<std::string> datasets;
    cv::Size2i imgSize;
    std::string MainFolder;
    std::string subDataset;
    std::string associateFile;
    bool DO_OUTPUT_IMAGES;
    bool READ_FROM_ASTRA_DATA;
    bool isFinished;
};

class IOWrapperRGBD
{
private:
//    FileManager mFileReader;
    std::ifstream fileList;
    std::ofstream associateFile;
    IOWrapperSettings mSettings;
    bool mQuitFlag;
    cv::Mat rgb,depth;
    int nFrames;
    //std::queue<ImgPyramidRGBD> mPyrQueue;
    std::queue<std::unique_ptr<ImgPyramidRGBD>> mPyrQueue;
    std::mutex mtx;
    bool mFinish;
    bool mInitSuccess;
    bool mAllImagesRead;

    ImgPyramidSettings mPyrConfig;
    std::shared_ptr<CameraPyr> mCamPyr;
    std::string outputImgDir;
    bool mHasMoreImages;
    void requestQuit();

    //different modalities
    void generateImgPyramidFromFiles();
#ifdef WITH_ORBBEC_ASTRA_PRO
    #ifdef WITH_ORBBEC_FFMPEG
        std::unique_ptr<OrbbecAstraProEngineFFMPEG> orbbecAstraProSensor;
    #else
        std::unique_ptr<OrbbecAstraEngine> orbbecAstraProSensor;
    #endif
#endif
#ifdef WITH_REALSENSE
    std::unique_ptr<RealsenseSensor> realSenseSensor;
#endif
#ifdef WITH_ORBBEC_ASTRA
        std::unique_ptr<OrbbecAstraOpenNIEngine> orbbecAstraSensor;
#endif
    //void adaptCannyValues();
    void generateImgPyramidFromAstraPro();
    void generateImgPyramidFromAstra();
    void generateImgPyramidFromRealSense();
    void writeImages(const cv::Mat& rgb, const cv::Mat depth, const float timestamp);
    bool readNextFrame(cv::Mat& rgb, cv::Mat& depth, double &rgbTimeStamp, double &depthTimeStamp, int skipFrames, double depthScaleFactor);
    int noFrames;
public:
    void generateImgPyramid()
    {
        if (mSettings.READ_FROM_ASTRA_PRO)
            generateImgPyramidFromAstraPro();
        else
        if (mSettings.READ_FROM_REALSENSE)
            generateImgPyramidFromRealSense();
        else
            if (mSettings.READ_FROM_ASTRA)
            {
                generateImgPyramidFromAstra();
            }
            else
                generateImgPyramidFromFiles();
    }
    IOWrapperRGBD(const IOWrapperSettings &settings, const ImgPyramidSettings &mPyrSettings, const std::shared_ptr<CameraPyr>& camPyr);
    //~IOWrapperRGBD();
    inline bool isImgPyramidAvailable()
    {
        //std::unique_lock<std::mutex> lock(this->mtx);
        I3D_LOG(i3d::detail) << "isImgPyramidAvailable";
        return mPyrQueue.size() > 0;
    }
    inline bool hasMoreImages()
    {
        return mHasMoreImages;
    }
    bool getOldestPyramid(ImgPyramidRGBD& pyr);
    bool getOldestPyramid(std::shared_ptr<ImgPyramidRGBD> &pyr);
    void setFinish(bool setFinish);
    inline bool isInitSuccess()
    {
        I3D_LOG(i3d::info) << "mSettings.isFinished" << mSettings.isFinished;
        if (mSettings.isFinished)
        {
            return false;
        }
        return mInitSuccess;
    }
};

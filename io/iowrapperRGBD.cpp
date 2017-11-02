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
#include "iowrapperRGBD.h"
#include <unistd.h>
#include "../utils/timer.h"
#include <boost/filesystem.hpp>

IOWrapperRGBD::IOWrapperRGBD(const IOWrapperSettings& settings, const ImgPyramidSettings& mPyrSettings, const std::shared_ptr<CameraPyr>& camPyr):
                        mSettings(settings), mQuitFlag(false), rgb(settings.imgSize,CV_8UC3),depth(settings.imgSize,CV_32FC1), nFrames(0),
                        mFinish(false), mAllImagesRead(false), mPyrConfig(mPyrSettings), mCamPyr(camPyr), mHasMoreImages(true), noFrames(0)
{
    mInitSuccess = true;
    I3D_LOG(i3d::info) << "camPyr->size(): " << camPyr->size() << " " <<settings.MainFolder+settings.subDataset+"/";
#ifdef WITH_ORBBEC_ASTRA_PRO
    if (mSettings.READ_FROM_ASTRA_PRO)
    {
        //OrbbecAstraEngine astra;
        //bool t = astra.isInitSuccess();
        I3D_LOG(i3d::info) << "After astra normal!";
        //orbbecSensor = std::unique_ptr<OrbbecAstraEngine>(&astra);
#ifdef WITH_ORBBEC_FFMPEG
        orbbecSensor = std::unique_ptr<OrbbecAstraEngineFFMPEG>(new OrbbecAstraEngineFFMPEG());
#else
        orbbecSensor = std::unique_ptr<OrbbecAstraEngine>(new OrbbecAstraEngine());
#endif
        mInitSuccess = orbbecSensor->isInitSuccess();
        I3D_LOG(i3d::info) << "After astra normal!";
    }
    else
        this->orbbecSensor = NULL;
#endif
#ifdef WITH_REALSENSE
    if (mSettings.READ_FROM_REALSENSE)
    {
        realSenseSensor = std::unique_ptr<RealsenseSensor>(new RealsenseSensor());
    }
    else
        this->realSenseSensor = NULL;
#endif
    if (mSettings.READ_FROM_DATASET())
    {
        //OPEN FILES
        fileList.open((mSettings.MainFolder+mSettings.subDataset+"/"+mSettings.associateFile).c_str(),std::ios_base::in);
        I3D_LOG(i3d::info) << "Reading: " << (mSettings.MainFolder+mSettings.subDataset+"/"+mSettings.associateFile);
         if (!fileList.is_open())
         {
             I3D_LOG(i3d::error) <<"Could not open file list";
             mQuitFlag = true;
         }
        assert(fileList.is_open() && "File could not been opened!");
    }
    //if (!mFileReader.isFileOpen()) mQuitFlag = true;
}

void IOWrapperRGBD::generateImgPyramidFromAstra()
{
#ifdef WITH_ORBBEC_ASTRA_PRO
    while (this->mSettings.READ_FROM_ASTRA_PRO && !mFinish)
    {
        if (this->orbbecSensor->getImages(rgb,depth,mSettings.DEPTH_SCALE_FACTOR))
        {
            auto start = Timer::getTime();
            nFrames++;
            I3D_LOG(i3d::info) << "nFrames: " << nFrames;
            //there is a strange orbbec bug, where the first two lines of the "color" image are invalid
            rgb.row(2).copyTo(rgb.row(0));
            rgb.row(2).copyTo(rgb.row(1));
            if (mSettings.DO_OUTPUT_IMAGES) writeImages(rgb,depth,nFrames);
            I3D_LOG(i3d::info) << mSettings.DEPTH_SCALE_FACTOR;
            //mPyrQueue.push(pyr);
            std::unique_ptr<ImgPyramidRGBD> ptrTmp(std::unique_ptr<ImgPyramidRGBD>(new ImgPyramidRGBD(mPyrConfig,mCamPyr,rgb,depth,nFrames)));
            {
                std::unique_lock<std::mutex> lock(this->mtx);
                mPyrQueue.push(std::move(ptrTmp));
                I3D_LOG(i3d::error) << "ImgPyramid queued" << mPyrQueue.size();
            }
            auto end = Timer::getTime();
            I3D_LOG(i3d::info) << "Reading image: " << nFrames << " in " << Timer::getTimeDiffMiS(start,end);
        }
        usleep(500);
    }
#else
    I3D_LOG(i3d::error) << "Not compiled with Orbbec Astra support!";
    exit(0);
#endif
}

//#define WITH_REALSENSE
void IOWrapperRGBD::writeImages(const cv::Mat& rgb, const cv::Mat depth, const float timestamp)
{
    if (!associateFile.is_open())
    {

        auto t = std::time(nullptr);
        auto tm = std::localtime(&t);
        char buffer[80];

        strftime(buffer,80,"%d-%m-%Y-%I-%M-%S",tm);
        //std::string str(buffer);
        outputImgDir = std::string(buffer);
        if (boost::filesystem::create_directory(outputImgDir))
        {
            associateFile.open(outputImgDir+"/associate.txt");
            boost::filesystem::create_directory(outputImgDir+"/rgb");
            boost::filesystem::create_directory(outputImgDir+"/depth");
            //I3D_LOG(i3d::error) << "error generating directories";
        }
    }
    else
    {I3D_LOG(i3d::error) << "Associate file already open! Not generating directories";}

    //we create and save to a folder!
    const std::string timestampStr = std::to_string(timestamp);
    const std::string depthFilename = outputImgDir+"/depth/"+timestampStr+".png";
    const std::string rgbFilename = outputImgDir+"/rgb/"+timestampStr+".png";

    //write to associate.txt
    //1305031471.927651 rgb/1305031471.927651.png 1305031471.924928 depth/1305031471.924928.png
    associateFile << timestampStr << " rgb/"+timestampStr+".png " << timestampStr << " depth/"+timestampStr+".png" << std::endl;
    cv::Mat depth16U(depth.rows,depth.cols,CV_16UC1);
    depth.convertTo(depth16U,CV_16U,1000);
    //save images
    cv::imwrite(depthFilename,depth16U);
    cv::imwrite(rgbFilename,rgb);
}

void IOWrapperRGBD::generateImgPyramidFromRealSense()
{

    #ifdef WITH_REALSENSE
    while (this->mSettings.READ_FROM_REALSENSE && !mFinish)
    {
        if (this->realSenseSensor->getImages(rgb,depth,mSettings.DEPTH_SCALE_FACTOR))
        {
            auto start = Timer::getTime();
            nFrames++;
            //there is a strange orbbec bug, where the first two lines of the "color" image are invalid
            if (mSettings.DO_OUTPUT_IMAGES) writeImages(rgb,depth,nFrames);
            I3D_LOG(i3d::info) << mSettings.DEPTH_SCALE_FACTOR;
            //mPyrQueue.push(pyr);
            std::unique_ptr<ImgPyramidRGBD> ptrTmp(std::unique_ptr<ImgPyramidRGBD>(new ImgPyramidRGBD(mPyrConfig,mCamPyr,rgb,depth,nFrames)));
            {
                std::unique_lock<std::mutex> lock(this->mtx);
                mPyrQueue.push(std::move(ptrTmp));
                I3D_LOG(i3d::error) << "ImgPyramid queued" << mPyrQueue.size();
            }
            auto end = Timer::getTime();
            //if (mPyrQueue.size() > 50) break;
            I3D_LOG(i3d::info) << "Reading image: " << nFrames << " in " << Timer::getTimeDiffMiS(start,end);
        }
        usleep(500);
    }
    #else
        I3D_LOG(i3d::error) << "Not compiled with RealSense support!";
        exit(0);
    #endif
}


void IOWrapperRGBD::requestQuit()
{
       std::unique_lock<std::mutex> lock(this->mtx);
       mQuitFlag = true;
}
void IOWrapperRGBD::generateImgPyramidFromFiles()
{
//    cv::Mat rgb = cv::Mat(480,640,CV_8UC3);
//    cv::Mat depth = cv::Mat(480,640,CV_32FC1);
    double rgbTimeStamp = 0,depthTimeStamp = 0;
    while (readNextFrame(rgb,depth,rgbTimeStamp,depthTimeStamp,mSettings.SKIP_FIRST_N_FRAMES,mSettings.DEPTH_SCALE_FACTOR) && !mFinish)
    {

        I3D_LOG(i3d::info) << "Read next Frame!";
        const double tumRefTimestamp = (mSettings.useDepthTimeStamp ? depthTimeStamp : rgbTimeStamp);
        //clock_t start = clock();
        auto start = Timer::getTime();
        //there is a strange orbbec bug, where the first line of the "color" image is invalid
        if (mSettings.READ_FROM_ASTRA_DATA)
        {
            rgb.row(2).copyTo(rgb.row(0));
            rgb.row(2).copyTo(rgb.row(1));
        }

        nFrames++;
        auto startPush = Timer::getTime();
        I3D_LOG(i3d::info) << "Before img pyramid!";
        std::unique_ptr<ImgPyramidRGBD> pyrPtr(new ImgPyramidRGBD(mPyrConfig,mCamPyr,rgb,depth,tumRefTimestamp));
        auto endPush = Timer::getTime();
        I3D_LOG(i3d::info) << "Creating pyramid: " << Timer::getTimeDiffMiS(startPush,endPush) << " mis." << mSettings.DEPTH_SCALE_FACTOR;
        {
            auto startPush = Timer::getTime();
            std::unique_lock<std::mutex> lock(this->mtx);
            mPyrQueue.push(std::move(pyrPtr));
            auto endPush = Timer::getTime();
            I3D_LOG(i3d::info) << "Waiting for push: " << Timer::getTimeDiffMiS(startPush,endPush) << " mis.";
        }
        auto end = Timer::getTime();
        I3D_LOG(i3d::info) << "Reading image: " << nFrames << " in " << Timer::getTimeDiffMiS(start,end);
        if (nFrames>mSettings.READ_N_IMAGES) break;
        usleep(1000);
    }
    //this->mHasMoreImages = false;
    mAllImagesRead = true;
    while (!mFinish)
    {
        usleep(3000);
    }
}
bool IOWrapperRGBD::readNextFrame(cv::Mat& rgb, cv::Mat& depth, double &rgbTimeStamp, double &depthTimeStamp, int skipFrames, double depthScaleFactor)
{
    I3D_LOG(i3d::info) << "readNextFrame!";
    auto start = Timer::getTime();
    bool fileRead = false;
    std::string currRGBFile, currDepthFile;
    std::string inputLine;
    //std::cout << this->dataFolder << std::endl;
    //read lines
    while((std::getline(fileList,inputLine)))
    {
        //ignore comments
        if (inputLine[0] == '#' || inputLine.empty()) continue;
        noFrames++;
        if (noFrames<=skipFrames) continue;
        std::istringstream is_associate(inputLine);
        is_associate >> rgbTimeStamp >> currRGBFile >> depthTimeStamp >> currDepthFile;
        std::cout << "RGB Files: " << std::fixed << rgbTimeStamp << " filename " << currRGBFile << std::endl;
        std::cout << "Depth Files: " << std::fixed << depthTimeStamp << " filename " << currDepthFile << std::endl;
        fileRead = true;
        break;
    }
    //now read the images
    //I3D_LOG(i3d::info) << "Reading: " << mSettings.MainFolder+mSettings.subDataset+currRGBFile;
    rgb = cv::imread(mSettings.MainFolder+mSettings.subDataset+"/"+currRGBFile);
    depth = cv::imread(mSettings.MainFolder+mSettings.subDataset+"/"+currDepthFile,CV_LOAD_IMAGE_UNCHANGED);
    depth.convertTo(depth,CV_32FC1,1.0f/depthScaleFactor);
    //divide by 5000 to get distance in metres
    //depth = depth/depthScaleFactor;
    auto end = Timer::getTime();
    I3D_LOG(i3d::info) << "readNextFrame " << std::fixed << depthTimeStamp << " Time: " << std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() << "mis";
    return fileRead;
}

void IOWrapperRGBD::setFinish(bool setFinish)
{
    std::unique_lock<std::mutex> lock(this->mtx);
    this->mFinish = true;
}

bool IOWrapperRGBD::getOldestPyramid(std::shared_ptr<ImgPyramidRGBD>& pyr)
{

    I3D_LOG(i3d::error) << "getOldestPyramid = "<<mPyrQueue.size();
    if (mPyrQueue.empty()) return false;
    I3D_LOG(i3d::error) << "mPyrQueue.size() = "<<mPyrQueue.size();
    std::unique_lock<std::mutex> lock(this->mtx);
    pyr = std::move(mPyrQueue.front());
    mPyrQueue.pop();
    if (mPyrQueue.empty() && mAllImagesRead) this->mHasMoreImages = false;
    return pyr != NULL;
}

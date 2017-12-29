/**
* This file is part of REVO.
*
* Copyright (C) 2014-2017
* Schenk Fabian <schenk at icg dot tugraz dot at> (Graz University of Technology),
* Ebner Thomas <thomas dot ebner at cfi dot lbg dot ac dot at> (Graz LBI)
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
* This code was adapted from InfiniTAM's LibUVC Engine <https://github.com/victorprad/InfiniTAM>.
* For the original parts, the original copyright applies:
* "Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM"
*/
//
#include <cstdio>
#include <stdexcept>
#include <iomanip>
#include "OrbbecAstraOpenNIEngine.h"
#include <OpenNI.h>
#include <time.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "../utils/Logging.h"
#include "../utils/timer.h"
#ifndef WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-function"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <libavdevice/avdevice.h>
#include <libswscale/swscale.h>

#ifdef __cplusplus
}
#endif
class EOFListener : public openni::OpenNI::DeviceStateChangedListener
{
    public:
    EOFListener() : eofReached(false){}

    void onDeviceStateChanged(const openni::DeviceInfo*, openni::DeviceState newState)
    {
        if (newState == openni::DEVICE_STATE_EOF)
        {
            printf("OpenNI: Reached end of file\n");
            eofReached = true;
        }
    }

    bool reachedEOF() const { return eofReached; }

    private:
    bool eofReached;
};


class OrbbecAstraOpenNIEngine::PrivateData {
    public:
    PrivateData(void) : streams(NULL) {}
    openni::Device device;
    openni::VideoStream depthStream, colorStream;

    openni::VideoFrameRef depthFrame;
    openni::VideoFrameRef colorFrame;
    openni::VideoStream **streams;

    EOFListener eofListener;
};


static openni::VideoMode findBestMode(const openni::SensorInfo *sensorInfo, int requiredResolutionX = -1, int requiredResolutionY = -1, openni::PixelFormat requiredPixelFormat = (openni::PixelFormat)-1)
{
    const openni::Array<openni::VideoMode> & modes = sensorInfo->getSupportedVideoModes();
    openni::VideoMode bestMode = modes[0];
    for (int m = 0; m < modes.getSize(); ++m) {
        fprintf(stderr, "mode %i: %ix%i, %i %i\n", m, modes[m].getResolutionX(), modes[m].getResolutionY(), modes[m].getFps(), modes[m].getPixelFormat());
        const openni::VideoMode & curMode = modes[m];
        if ((requiredPixelFormat != (openni::PixelFormat)-1)&&(curMode.getPixelFormat() != requiredPixelFormat)) continue;

        bool acceptAsBest = false;
        if ((curMode.getResolutionX() == bestMode.getResolutionX())&&
             (curMode.getFps() > bestMode.getFps())) {
            acceptAsBest = true;
        } else if ((requiredResolutionX <= 0)&&(requiredResolutionY <= 0)) {
            if (curMode.getResolutionX() > bestMode.getResolutionX()) {
                acceptAsBest = true;
            }
        } else {
            int diffX_cur = abs(curMode.getResolutionX()-requiredResolutionX);
            int diffX_best = abs(bestMode.getResolutionX()-requiredResolutionX);
            int diffY_cur = abs(curMode.getResolutionY()-requiredResolutionY);
            int diffY_best = abs(bestMode.getResolutionY()-requiredResolutionY);
            if (requiredResolutionX > 0) {
                if (diffX_cur < diffX_best) {
                    acceptAsBest = true;
                }
                if ((requiredResolutionY > 0)&&(diffX_cur == diffX_best)&&(diffY_cur < diffY_best)) {
                    acceptAsBest = true;
                }
            } else if (requiredResolutionY > 0) {
                if (diffY_cur < diffY_best) {
                    acceptAsBest = true;
                }
            }
        }
        if (acceptAsBest) bestMode = curMode;
    }
    fprintf(stderr, "=> best mode: %ix%i, %i %i\n", bestMode.getResolutionX(), bestMode.getResolutionY(), bestMode.getFps(), bestMode.getPixelFormat());
    return bestMode;
}
OrbbecAstraOpenNIEngine::OrbbecAstraOpenNIEngine()
{
    this->imageSize_d = cv::Size2i(640,480);
    this->imageSize_rgb = cv::Size2i(640,480);
    bool useInternalCalibration = true;


    const char* deviceURI = openni::ANY_DEVICE;

    data = new PrivateData();

    openni::Status rc = openni::STATUS_OK;

    rc = openni::OpenNI::initialize();
    printf("OpenNI: Initialization ... \n%s\n", openni::OpenNI::getExtendedError());

    rc = data->device.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        std::string message("OpenNI: Device open failed!\n");
        message += openni::OpenNI::getExtendedError();
        openni::OpenNI::shutdown();
        delete data;
        data = NULL;
        std::cout << message;
        return;
    }

    // Add listener to handle the EOF event and prevent deadlocks while waiting for more frames.
    openni::OpenNI::addDeviceStateChangedListener(&data->eofListener);

    openni::PlaybackControl *control = data->device.getPlaybackControl();
    if (control != NULL) {
        // this is a file! make sure we get every frame
        control->setSpeed(-1.0f);
        control->setRepeatEnabled(false);
    }

    rc = data->depthStream.create(data->device, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        openni::VideoMode depthMode = findBestMode(data->device.getSensorInfo(openni::SENSOR_DEPTH), imageSize_d.x, imageSize_d.y, openni::PIXEL_FORMAT_DEPTH_1_MM);
        rc = data->depthStream.setVideoMode(depthMode);
        if (rc != openni::STATUS_OK)
        {
            printf("OpenNI: Failed to set depth mode\n");
        }
        data->depthStream.setMirroringEnabled(false);

        rc = data->depthStream.start();

        if (useInternalCalibration) data->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        if (rc != openni::STATUS_OK)
        {
            printf("OpenNI: Couldn't start depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
            data->depthStream.destroy();
        }

        imageSize_d.x = data->depthStream.getVideoMode().getResolutionX();
        imageSize_d.y = data->depthStream.getVideoMode().getResolutionY();

        printf("Initialised OpenNI depth camera with resolution: %d x %d\n", imageSize_d.x, imageSize_d.y);

        depthAvailable = true;
    }
    else
    {
        printf("OpenNI: Couldn't find depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
        depthAvailable = false;
    }

    rc = data->colorStream.create(data->device, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK)
    {

        openni::VideoMode colourMode = findBestMode(data->device.getSensorInfo(openni::SENSOR_COLOR), imageSize_rgb.x, imageSize_rgb.y);
        const openni::Array<openni::VideoMode> & modes = data->device.getSensorInfo(openni::SENSOR_COLOR)->getSupportedVideoModes();
        //colourMode = modes[10];
        rc = data->colorStream.setVideoMode(colourMode);
        if (rc != openni::STATUS_OK)
        {
            printf("OpenNI: Failed to set color mode\n");
        }
        data->colorStream.setMirroringEnabled(false);

        rc = data->colorStream.start();
        if (rc != openni::STATUS_OK)
        {
            printf("OpenNI: Couldn't start colorStream stream:\n%s\n", openni::OpenNI::getExtendedError());
            data->colorStream.destroy();
        }

        imageSize_rgb.x = data->colorStream.getVideoMode().getResolutionX();
        imageSize_rgb.y = data->colorStream.getVideoMode().getResolutionY();

        printf("Initialised OpenNI color camera with resolution: %d x %d\n", imageSize_rgb.x, imageSize_rgb.y);

        colorAvailable = true;
    }
    else
    {
        printf("OpenNI: Couldn't find colorStream stream:\n%s\n", openni::OpenNI::getExtendedError());
        colorAvailable = false;
    }

    if (!depthAvailable)
    {
        openni::OpenNI::shutdown();
        delete data;
        data = NULL;
        std::cout << "OpenNI: No valid streams. Exiting." << std::endl;
        return;
    }

    data->streams = new openni::VideoStream*[2];
    if (depthAvailable) data->streams[0] = &data->depthStream;
    if (colorAvailable) data->streams[1] = &data->colorStream;

    //this part always results in a seg fault! -> && false
    //Might be included in a future release
    if (depthAvailable && false) {
        float h_fov = data->depthStream.getHorizontalFieldOfView();
        float v_fov = data->depthStream.getVerticalFieldOfView();
        openni::CameraSettings* cs = data->depthStream.getCameraSettings();
        I3D_LOG(i3d::info) << "depth cs->getExposure(): " << cs->getExposure() << " cs->getGain: "<<cs->getGain()
                           << "h_fov, v_fov" << h_fov << " " << v_fov;
    }
    if (colorAvailable) {
        float h_fov = data->colorStream.getHorizontalFieldOfView();
        float v_fov = data->colorStream.getVerticalFieldOfView();
        openni::CameraSettings* cs = data->colorStream.getCameraSettings();
        I3D_LOG(i3d::info) << "color cs->getExposure(): " << cs->getExposure() << " cs->getGain: "<<cs->getGain()
                           << "h_fov, v_fov" << h_fov << " " << v_fov;

        I3D_LOG(i3d::info) <<
            (float)imageSize_rgb.x / (2.0f * tan(h_fov/2.0f))  << " " <<
            (float)imageSize_rgb.y / (2.0f * tan(v_fov/2.0f))  << " " <<
            (float)imageSize_rgb.x / 2.0f << " " <<
            (float)imageSize_rgb.y / 2.0f;
    }
    I3D_LOG(i3d::info) << "Finished constructing: OrbbecAstraOpenNIEngine";
}

OrbbecAstraOpenNIEngine::~OrbbecAstraOpenNIEngine()
{
    I3D_LOG(i3d::info) << "DESTRUCTOR called!! OrbbecAstraOpenNIEngine";
	if (data != NULL)
	{
		if (depthAvailable)
		{
			data->depthStream.stop();
			data->depthStream.destroy();
		}

        if (colorAvailable)
        {
            data->colorStream.stop();
            data->colorStream.destroy();
        }
		data->device.close();

        openni::OpenNI::removeDeviceStateChangedListener(&data->eofListener);

        delete[] data->streams;
        delete data;
	}
	openni::OpenNI::shutdown();
}

bool OrbbecAstraOpenNIEngine::getImages(cv::Mat& rgbImage, cv::Mat& rawDepthImage, const float depthScaleFactor)
{
    auto start = Timer::getTime();
    int changedIndex, waitStreamCount;
    if (depthAvailable && colorAvailable) waitStreamCount = 2;
    else waitStreamCount = 1;

    openni::Status rc = openni::OpenNI::waitForAnyStream(data->streams, waitStreamCount, &changedIndex);
    if (rc != openni::STATUS_OK) { printf("OpenNI: Wait failed\n"); return false; }

    if(depthAvailable) data->depthStream.readFrame(&data->depthFrame);
    if(colorAvailable) data->colorStream.readFrame(&data->colorFrame);

    if (depthAvailable && !data->depthFrame.isValid()) return false;
    if (colorAvailable && !data->colorFrame.isValid()) return false;

    //Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
    if (colorAvailable)
    {
        const openni::RGB888Pixel* colorImagePix = (const openni::RGB888Pixel*)data->colorFrame.getData();
        memcpy(rgbImage.data,colorImagePix,sizeof(openni::RGB888Pixel)*rgbImage.total());
//        for (int i = 0; i < rgbImage->noDims.x * rgbImage->noDims.y; i++)
//        {
//            Vector4u newPix; openni::RGB888Pixel oldPix = colorImagePix[i];
//            newPix.x = oldPix.r; newPix.y = oldPix.g; newPix.z = oldPix.b; newPix.w = 255;
//            rgb[i] = newPix;
//        }
    }
    else rgbImage.setTo(cv::Scalar(0,0,0));


    //openni::DepthPixel* depthImagePix;
    if (depthAvailable)
    {
        //int changedIndex;
        //openni::Status rc = openni::OpenNI::waitForAnyStream(data->streams, waitStreamCount, &changedIndex);
        //if (rc != openni::STATUS_OK) { printf("OpenNI: Wait failed\n"); return false; }
        //if (depthAvailable) data->depthStream.readFrame(&data->depthFrame);
        //if (depthAvailable && !data->depthFrame.isValid()) return false;
        //openni::DepthPixel* depthImagePix = (openni::DepthPixel*)data->depthFrame.getData();
        const short* ptIt = static_cast<const short*>(data->depthFrame.getData());
        const short* ptEnd = static_cast<const short*>(data->depthFrame.getData())+rawDepthImage.cols*rawDepthImage.rows;
        int i = 0;
        //convert it to OpenCV float image
        for (;ptIt < ptEnd; ++ptIt,++i)
            rawDepthImage.at<float>(int(i/rawDepthImage.cols),i%rawDepthImage.cols) = (*ptIt)/depthScaleFactor;
    }

    auto end = Timer::getTime();
    I3D_LOG(i3d::info) << "Orbbec total time: " << Timer::getTimeDiffMiS(start,end) <<" Get " << imagesGet << "/" << imagesReadThread;
    return true; /*true*/;
}


//bool OrbbecAstraOpenNIEngine::getImages(cv::Mat& rgbImage, cv::Mat& rawDepthImage, const float depthScaleFactor)
//{
//    auto start = Timer::getTime();
//    if (!isInitSuccess())
//    {
//        I3D_LOG(i3d::error) << "Init not successful!";
//        return false;
//    }
//    int32_t timeout = 30; //ms
//    //lock RGB frame:
//    //std::unique_lock<std::mutex> l(data->newRGBFrameLock);
//    auto startTime = Timer::getTime();

//    if (init_ffmpeg)
//    {
//        std::unique_lock<std::mutex> l(data->newRGBFrameLock);
//        auto rgbTimeStart = Timer::getTime();

//        //wait until new frame
//        while(!data->newRGBFrame)
//        {
//            //I3D_LOG(i3d::info) << "Waiting!! START";
//            auto waitStart = Timer::getTime();
//            data->newRGBFrameCondition.wait(l);
//            //I3D_LOG(i3d::info) << "Waiting!! ENDE" << Timer::getTimeDiffMiS(waitStart,Timer::getTime());
//        }
//        auto rgbTimeEnd = Timer::getTime();
//        //new frame is now in data->rgbFrame
//        //set new_frame to false
//        data->newRGBFrame = false;
//        memcpy(rgbImage.data, data->rgb.data,sizeof(uchar)*data->rgb.total()*3);
//        this->imagesGet++;
//        I3D_LOG(i3d::info) << "Waited for rgb image: " << Timer::getTimeDiffMiS(rgbTimeStart,rgbTimeEnd) << ": " <<  imagesGet << "/" << imagesReadThread;
//        //cv::imwrite("rgb/"+std::to_string(imagesReadThread)+"_"+std::to_string(imagesGet)+"get.png",rgbImage);
//    }
//    auto endTime = Timer::getTime();
//    I3D_LOG(i3d::info) <<  "Time for reading RGB: " << Timer::getTimeDiffMiS(startTime,endTime);
//    openni::DepthPixel* depthImagePix;
//    if (init_openni)
//    {
//        int changedIndex, waitStreamCount = 1;
//        openni::Status rc = openni::OpenNI::waitForAnyStream(data->streams, waitStreamCount, &changedIndex);
//        if (rc != openni::STATUS_OK) { printf("OpenNI: Wait failed\n"); return false; }
//        if (depthAvailable) data->depthStream.readFrame(&data->depthFrame);
//        if (depthAvailable && !data->depthFrame.isValid()) return false;
//        depthImagePix = (openni::DepthPixel*)data->depthFrame.getData();
//    }
//    const short* ptIt = static_cast<const short*>(data->depthFrame.getData());
//    const short* ptEnd = static_cast<const short*>(data->depthFrame.getData())+rawDepthImage.cols*rawDepthImage.rows;
//    int i = 0;
//    //convert it to OpenCV float image
//    for (;ptIt < ptEnd; ++ptIt,++i)
//        rawDepthImage.at<float>(int(i/rawDepthImage.cols),i%rawDepthImage.cols) = (*ptIt)/depthScaleFactor;
//    auto end = Timer::getTime();
//    I3D_LOG(i3d::info) << "Orbbec total time: " << Timer::getTimeDiffMiS(start,end) <<" Get " << imagesGet << "/" << imagesReadThread;
//    return true; /*true*/;
//}

bool OrbbecAstraOpenNIEngine::hasMoreImages(void)
{
    return (data!=NULL);
}

//void OrbbecAstraOpenNIEngine::readFramesThread()
//{
//    int res;
//    int frameFinished;
//    AVPacket packet;
//    SwsContext * img_convert_ctx;
//    img_convert_ctx = sws_getCachedContext(NULL, data->pCodecCtx->width, data->pCodecCtx->height, data->pCodecCtx->pix_fmt,   data->pCodecCtx->width, data->pCodecCtx->height, AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL,NULL);
//    while(!data->exitReadFramesThread)
//    {
//        res = av_read_frame(data->pFormatCtx,&packet);
//        if(res >= 0)
//        {
//            if(packet.stream_index == data->video_stream_index) //correct stream?
//            {
//                avcodec_decode_video2(data->pCodecCtx, data->pFrame, &frameFinished, &packet);

//                if(frameFinished)
//                {
//                    std::unique_lock<std::mutex> l(data->newRGBFrameLock);
//                    sws_scale(img_convert_ctx,
//                               reinterpret_cast<AVPicture*>(data->pFrame)->data,
//                               reinterpret_cast<AVPicture*>(data->pFrame)->linesize, 0, data->pCodecCtx->height,
//                               reinterpret_cast<AVPicture*>(data->pFrameRGB)->data,
//                               reinterpret_cast<AVPicture*>(data->pFrameRGB)->linesize);
//                    char* oldPix = reinterpret_cast<char *>(data->pFrameRGB->data[0]);
//                    memcpy(data->rgb.data,oldPix,data->rgb.total()*sizeof(char)*3);
//                    this->imagesReadThread++;
//                    //cv::imwrite("rgb/"+std::to_string(imagesReadThread)+".png",data->rgb);
//                    data->newRGBFrame = true;
//                    data->newRGBFrameCondition.notify_all();
//                }
//            }
//            av_free_packet(&packet);
//        }
//    }
//    sws_freeContext(img_convert_ctx);
//}

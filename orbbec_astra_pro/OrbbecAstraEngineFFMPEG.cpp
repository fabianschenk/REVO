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
#include "OrbbecAstraEngineFFMPEG.h"
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

class OrbbecAstraProEngineFFMPEG::PrivateData {
    public:
    PrivateData(void)
        : streams(NULL),
          iteration(0)
    {
    }

    ~PrivateData(void)
    {
      //if (rgb) delete[] rgb;
    }

    openni::Device device;
    openni::VideoStream depthStream;//, colorStream;
    openni::VideoFrameRef depthFrame;
    openni::VideoStream **streams;
    int iteration;

    //FFMPEG (RGB Image)
    AVCodecContext  *pCodecCtx = NULL;
    AVFormatContext *pFormatCtx = NULL;
    AVCodec * pCodec = NULL;
    AVInputFormat *iformat = NULL;
    AVFrame *pFrame = NULL, *pFrameRGB = NULL;
    int video_stream_index = -1;

    //readFramesThread data:
    std::thread readRGBFrameThread;
    bool exitReadFramesThread;
    //cv::Mat rgbFrame;
    //Vector4u *rgb = NULL;
    cv::Mat rgb;
    bool newRGBFrame;
    std::mutex newRGBFrameLock;
    std::condition_variable newRGBFrameCondition;
};

bool OrbbecAstraProEngineFFMPEG::initFFMPEG_RGB()
{
    avdevice_register_all();
    avcodec_register_all();

    const std::string requested_video_size = std::to_string(imageSize_rgb.width) + "x" + std::to_string(this->imageSize_rgb.height);
    I3D_LOG(i3d::info) << "requested_video_size: " << requested_video_size;
    const int requested_framerate = 30;
    std::string filenameSrc = this->rgbDeviceURI;
    if(filenameSrc.empty())
    {
#if WIN32
        //TODO: appropriate name unter windows?
      filenameSrc = "video=Astra Pro HD Camera";// video = TODO_FIND_CAMERANAME_WINDOWS";
#else
      filenameSrc = "/dev/video0";
#endif
    }

#if WIN32
    const std::string input_format("dshow");
#else
    const std::string input_format("v4l2");
#endif

    AVDictionary *options = NULL;
    av_dict_set(&options, "framerate", std::to_string(requested_framerate).c_str(), 0);
    av_dict_set(&options, "video_size", requested_video_size.c_str(), 0);

    data->iformat = av_find_input_format(input_format.c_str());
    if(!data->iformat)
    {
        I3D_LOG(i3d::error) << "initUVCRGB(): could not find input format " << input_format;
        return false;
    }

    data->pFormatCtx =  avformat_alloc_context();
    if(!data->pFormatCtx)
    {
        I3D_LOG(i3d::error) << "initUVCRGB(): could not alloc avformat_context";
        return false;
    }

    if(avformat_open_input(&data->pFormatCtx, filenameSrc.c_str(), data->iformat, &options) != 0)
    {
        I3D_LOG(i3d::error) << "initUVCRGB(): could not open input " << filenameSrc;
        return false;
    }
    if(avformat_find_stream_info(data->pFormatCtx, &options) < 0)
    {
        I3D_LOG(i3d::error) << "initUVCRGB(): could not find stream info of " << filenameSrc;
        return false;
    }

    av_dump_format(data->pFormatCtx, 0, filenameSrc.c_str(), 0);
    for(unsigned i = 0; i < data->pFormatCtx->nb_streams; i++)
    {
        if(data->pFormatCtx->streams[i]->codec->coder_type==AVMEDIA_TYPE_VIDEO)
        {
            data->video_stream_index = i;
            break;
        }
    }

    if(data->video_stream_index == -1)
    {
        I3D_LOG(i3d::error) << "initUVCRGB(): couldn't find a stream of type AVMEDIA_TYPE_VIDEO in " << filenameSrc.c_str();
        return false;
    }
    data->pCodecCtx = data->pFormatCtx->streams[data->video_stream_index]->codec;


    data->pCodec =avcodec_find_decoder(data->pCodecCtx->codec_id);
    if(data->pCodec==NULL)
    {
        I3D_LOG(i3d::error) << "initUVCRGB(): couldn't find a decoder for codec " << data->pFormatCtx->streams[data->video_stream_index]->codec->codec_descriptor->long_name;
        return false;
    }

    if(avcodec_open2(data->pCodecCtx, data->pCodec,NULL) < 0)
    {
        I3D_LOG(i3d::error) << "initUVCRGB(): couldn't open decoder for codec " << data->pFormatCtx->streams[data->video_stream_index]->codec->codec_descriptor->long_name;
        return false;
    }

    //set the actual image size:
    this->imageSize_rgb = cv::Size2i(data->pCodecCtx->width, data->pCodecCtx->height);
    I3D_LOG(i3d::info) << "FFMPEG RGB image size = " << data->pCodecCtx->width << "x" << data->pCodecCtx->height;

    data->pFrame    = av_frame_alloc();
    data->pFrameRGB = av_frame_alloc();

    if(!data->pFrame || !data->pFrameRGB)
    {
        I3D_LOG(i3d::error) << "could not alloc frames";
        return false;
    }

    AVPixelFormat  pFormat = AV_PIX_FMT_RGB24;
    size_t numBytes = size_t(avpicture_get_size(pFormat, data->pCodecCtx->width, data->pCodecCtx->height));
    uint8_t *buffer = static_cast<uint8_t *>(av_malloc(numBytes*sizeof(uint8_t)));
    avpicture_fill(reinterpret_cast<AVPicture * >(data->pFrameRGB), buffer, pFormat, data->pCodecCtx->width, data->pCodecCtx->height);
    data->rgb = cv::Mat(imageSize_rgb,CV_8UC3);//new Vector4u[imageSize_rgb.width * imageSize_rgb.height];
    data->newRGBFrame = false;
    data->exitReadFramesThread = false;
    imagesReadThread = 0;
    imagesGet = 0;
    data->readRGBFrameThread = std::thread(&OrbbecAstraProEngineFFMPEG::readFramesThread, this);
    return true;
}
bool OrbbecAstraProEngineFFMPEG::initOpenNIDepth()
{
    openni::Status rc = openni::STATUS_OK;
    cv::Point2i requested_imageSize_d(640,480);
    rc = openni::OpenNI::initialize();
    printf("OpenNI: Initialization ... \n%s\n", openni::OpenNI::getExtendedError());

    rc = data->device.open(openni::ANY_DEVICE);
    if (rc != openni::STATUS_OK)
    {
        std::string message("OpenNI: Device open failed!\n");
        message += openni::OpenNI::getExtendedError();
        openni::OpenNI::shutdown();
        delete data;
        data = NULL;
        std::cout << message;
        return false;
    }

    openni::PlaybackControl *control = data->device.getPlaybackControl();
    if (control != NULL) {
        // this is a file! make sure we get every frame
        control->setSpeed(-1.0f);
        control->setRepeatEnabled(false);
    }

    rc = data->depthStream.create(data->device, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        //openni::VideoMode depthMode = findBestMode(data->device.getSensorInfo(openni::SENSOR_IR), requested_imageSize_d.x, requested_imageSize_d.y, openni::PIXEL_FORMAT_DEPTH_1_MM);
        //rc = data->depthStream.setVideoMode(depthMode);
        if (rc != openni::STATUS_OK)
        {
            printf("OpenNI: Failed to set depth mode\n");
        }
        data->depthStream.setMirroringEnabled(false);

        rc = data->depthStream.start();

        //if (useInternalCalibration)
        data->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        data->device.setDepthColorSyncEnabled(true);

        if (rc != openni::STATUS_OK)
        {
            printf("OpenNI: Couldn't start depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
            data->depthStream.destroy();
        }

        imageSize_d.width = data->depthStream.getVideoMode().getResolutionX();
        imageSize_d.height = data->depthStream.getVideoMode().getResolutionY();

        printf("Initialised OpenNI depth camera with resolution: %d x %d\n", imageSize_d.width, imageSize_d.height);
        depthAvailable = true;
    }
    else
    {
        printf("OpenNI: Couldn't find depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
        depthAvailable = false;
    }

    data->streams = new openni::VideoStream*[2];
    if (depthAvailable) data->streams[0] = &data->depthStream;
    return true;
}

OrbbecAstraProEngineFFMPEG::OrbbecAstraProEngineFFMPEG():
    init_openni(false),init_ffmpeg(false)
{
    this->imageSize_d = cv::Size2i(0,0);
    this->imageSize_rgb = cv::Size2i(640,480);
	data = new PrivateData();
    init_openni = initOpenNIDepth();
    init_ffmpeg = initFFMPEG_RGB();
    I3D_LOG(i3d::info) << "init_ffmpeg: " << init_ffmpeg << " init_openni: " << init_openni;
}

OrbbecAstraProEngineFFMPEG::~OrbbecAstraProEngineFFMPEG()
{
    I3D_LOG(i3d::info) << "DESTRUCTOR called!! OrbbecAstraEngineFFMPEG";
	if (data != NULL)
	{
		if (depthAvailable)
		{
			data->depthStream.stop();
			data->depthStream.destroy();
		}
		data->device.close();

        delete[] data->streams;

        //cleanup stuff of FFMPEG:
        data->exitReadFramesThread = true;
        if(data->readRGBFrameThread.joinable())
            data->readRGBFrameThread.join();

        if(data->pCodecCtx)
            avcodec_close(data->pCodecCtx);

        if(data->pFrame)
            av_free(data->pFrame);

        if(data->pFrameRGB)
            av_free(data->pFrameRGB);

        if(data->pFormatCtx)
            avformat_close_input(&data->pFormatCtx);
        delete data;
	}
	openni::OpenNI::shutdown();
}

bool OrbbecAstraProEngineFFMPEG::getImages(cv::Mat& rgbImage, cv::Mat& rawDepthImage, const float depthScaleFactor)
{
    auto start = Timer::getTime();
    if (!isInitSuccess())
    {
        I3D_LOG(i3d::error) << "Init not successful!";
        return false;
    }
    int32_t timeout = 30; //ms
    //lock RGB frame:
    //std::unique_lock<std::mutex> l(data->newRGBFrameLock);
    auto startTime = Timer::getTime();

    if (init_ffmpeg)
    {
        I3D_LOG(i3d::info) << "init_ffmpeg";
        std::unique_lock<std::mutex> l(data->newRGBFrameLock);
        auto rgbTimeStart = Timer::getTime();
        //wait until new frame
        while(!data->newRGBFrame)
        {
            //I3D_LOG(i3d::info) << "Waiting!! START";
            auto waitStart = Timer::getTime();
            data->newRGBFrameCondition.wait(l);
            //I3D_LOG(i3d::info) << "Waiting!! ENDE" << Timer::getTimeDiffMiS(waitStart,Timer::getTime());
        }
        auto rgbTimeEnd = Timer::getTime();
        //new frame is now in data->rgbFrame
        //set new_frame to false
        data->newRGBFrame = false;
        memcpy(rgbImage.data, data->rgb.data,sizeof(uchar)*data->rgb.total()*3);
        this->imagesGet++;
        //I3D_LOG(i3d::info) << "Waited for rgb image: " << Timer::getTimeDiffMiS(rgbTimeStart,rgbTimeEnd) << ": " <<  imagesGet << "/" << imagesReadThread;
        //cv::imwrite("rgb/"+std::to_string(imagesReadThread)+"_"+std::to_string(imagesGet)+"get.png",rgbImage);
    }
    auto endTime = Timer::getTime();
    I3D_LOG(i3d::info) <<  "Time for reading RGB: " << Timer::getTimeDiffMiS(startTime,endTime);
    openni::DepthPixel* depthImagePix;
    if (init_openni)
    {
        int changedIndex, waitStreamCount = 1;
        openni::Status rc = openni::OpenNI::waitForAnyStream(data->streams, waitStreamCount, &changedIndex);
        if (rc != openni::STATUS_OK) { printf("OpenNI: Wait failed\n"); return false; }
        if (depthAvailable) data->depthStream.readFrame(&data->depthFrame);
        if (depthAvailable && !data->depthFrame.isValid()) return false;
        depthImagePix = (openni::DepthPixel*)data->depthFrame.getData();
    }
    const short* ptIt = static_cast<const short*>(data->depthFrame.getData());
    const short* ptEnd = static_cast<const short*>(data->depthFrame.getData())+rawDepthImage.cols*rawDepthImage.rows;
    int i = 0;
    //convert it to OpenCV float image
    for (;ptIt < ptEnd; ++ptIt,++i)
        rawDepthImage.at<float>(int(i/rawDepthImage.cols),i%rawDepthImage.cols) = (*ptIt)/depthScaleFactor;
    auto end = Timer::getTime();
    I3D_LOG(i3d::info) << "Orbbec total time: " << Timer::getTimeDiffMiS(start,end) <<" Get " << imagesGet << "/" << imagesReadThread;
    return true; /*true*/;
}

bool OrbbecAstraProEngineFFMPEG::hasMoreImages(void)
{
    return (data!=NULL);
}

void OrbbecAstraProEngineFFMPEG::readFramesThread()
{
    int res;
    int frameFinished;
    AVPacket packet;
    SwsContext * img_convert_ctx;
    img_convert_ctx = sws_getCachedContext(NULL, data->pCodecCtx->width, data->pCodecCtx->height, data->pCodecCtx->pix_fmt,   data->pCodecCtx->width, data->pCodecCtx->height, AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL,NULL);
    while(!data->exitReadFramesThread)
    {
        res = av_read_frame(data->pFormatCtx,&packet);
        if(res >= 0)
        {
            if(packet.stream_index == data->video_stream_index) //correct stream?
            {
                avcodec_decode_video2(data->pCodecCtx, data->pFrame, &frameFinished, &packet);

                if(frameFinished)
                {
                    std::unique_lock<std::mutex> l(data->newRGBFrameLock);
                    sws_scale(img_convert_ctx,
                               reinterpret_cast<AVPicture*>(data->pFrame)->data,
                               reinterpret_cast<AVPicture*>(data->pFrame)->linesize, 0, data->pCodecCtx->height,
                               reinterpret_cast<AVPicture*>(data->pFrameRGB)->data,
                               reinterpret_cast<AVPicture*>(data->pFrameRGB)->linesize);
                    char* oldPix = reinterpret_cast<char *>(data->pFrameRGB->data[0]);
                    memcpy(data->rgb.data,oldPix,data->rgb.total()*sizeof(char)*3);
                    this->imagesReadThread++;
                    //cv::imwrite("rgb/"+std::to_string(imagesReadThread)+".png",data->rgb);
                    data->newRGBFrame = true;
                    data->newRGBFrameCondition.notify_all();
                }
            }
            av_free_packet(&packet);
        }
    }
    sws_freeContext(img_convert_ctx);
}

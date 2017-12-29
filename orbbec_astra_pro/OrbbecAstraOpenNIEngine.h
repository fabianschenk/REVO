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
#pragma once

#ifdef WITH_ORBBEC_ASTRA_PRO
//#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#endif
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "OpenNI2")
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <libavdevice/avdevice.h>
#include <libswscale/swscale.h>

#ifdef __cplusplus
}
#endif
/* Orbbec Astra Pro HD Camera ffmpeg modes dshow
[dshow @ 00000000025324a0] DirectShow video device options (from video devices)
[dshow @ 00000000025324a0]  Pin "Capture" (alternative pin name "0")
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=1280x720 fps=5 max s=1280x720 fps=10
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=1280x720 fps=5 max s=1280x720 fps=10
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=640x480 fps=15 max s=640x480 fps=30
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=640x480 fps=15 max s=640x480 fps=30
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=352x288 fps=15 max s=352x288 fps=30
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=352x288 fps=15 max s=352x288 fps=30
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=320x240 fps=15 max s=320x240 fps=30
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=320x240 fps=15 max s=320x240 fps=30
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=176x144 fps=15 max s=176x144 fps=30
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=176x144 fps=15 max s=176x144 fps=30
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=160x120 fps=15 max s=160x120 fps=30
[dshow @ 00000000025324a0]   pixel_format=yuyv422  min s=160x120 fps=15 max s=160x120 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=1280x720 fps=15 max s=1280x720 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=1280x720 fps=15 max s=1280x720 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=640x480 fps=15 max s=640x480 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=640x480 fps=15 max s=640x480 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=352x288 fps=15 max s=352x288 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=352x288 fps=15 max s=352x288 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=320x240 fps=15 max s=320x240 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=320x240 fps=15 max s=320x240 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=176x144 fps=15 max s=176x144 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=176x144 fps=15 max s=176x144 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=160x120 fps=15 max s=160x120 fps=30
[dshow @ 00000000025324a0]   vcodec=mjpeg  min s=160x120 fps=15 max s=160x120 fps=30
*/
// Sensor class for Orbbec Astra Sensor
class OrbbecAstraOpenNIEngine
{
public:
    class PrivateData;
private:
    SwsContext* img_convert_ctx;
    PrivateData *data;
    cv::Point2i imageSize_rgb, imageSize_d;
    bool colorAvailable, depthAvailable;

    std::string rgbDeviceURI, depthDeviceURI;
    /**
     * @brief readFramesThread reads continuously frames from FFMPEG
     *
     * FFMPEG has some internal buffering, which has the consequence to always
     * retrieve the NEXT frame instead of the NEWEST(LATEST) frame from the camera.
     * In case the caller doesn't retrieve frames at the requested framerate, some
     * latency(delay) can be observed.
     * This thread that continuously reads frames from FFMPEG to ensure to always have get the newest frame.
     *
     * @param calibFilename file to calibration parameters of ASTRA Sensor
     * @param depthDeviceURI device URI of depth sensor
     * @param rgbDeviceURI device URI of RGB sensor (e.g. /dev/video0)
     * @param requested_imageSize_rgb requested size of RGB image
     * @param requested_imageSize_rgb requested size of depth image
     */
    void readFramesThread();

    int imagesReadThread, imagesGet;
public:
    OrbbecAstraOpenNIEngine();
    ~OrbbecAstraOpenNIEngine();
    inline bool isInitSuccess()
    {
        return  this->colorAvailable && this->depthAvailable;
    }

    bool hasMoreImages(void);
    bool getImages(cv::Mat &rgbImage, cv::Mat &rawDepthImage, const float depthScaleFactor);
    //void readRGBFrame(cv::Mat &rgbImage);
    inline const cv::Size2i getDepthImageSize(void) const
    {
        return imageSize_d;
    }

    inline const cv::Size2i getRGBImageSize(void) const
    {
        return imageSize_rgb;
    }
};


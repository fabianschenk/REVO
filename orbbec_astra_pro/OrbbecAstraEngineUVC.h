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
// Sensor class for Orbbec AstraPro Sensor
// Depth sensor is accessed via OpenNI2
// Color camera is mounted as standard RGB camera

class OrbbecAstraEngine// : public ImageSourceEngine
{
public:
    class PrivateData;
private:
    //class PrivateData;
    PrivateData *data;
    cv::Point2i imageSize_rgb, imageSize_d;
    bool colorAvailable, depthAvailable;
    bool initUVCRGB();
    bool initOpenNIDepth();
    bool initSuccess;
public:
    OrbbecAstraEngine();
    ~OrbbecAstraEngine();
    inline bool isInitSuccess()
    {
        return initSuccess;
    }

    bool hasMoreImages(void);
    bool getImages(cv::Mat &rgbImage, cv::Mat &rawDepthImage, const float depthScaleFactor);
    cv::Point2i getDepthImageSize(void);
    cv::Point2i getRGBImageSize(void);
};


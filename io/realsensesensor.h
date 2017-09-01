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
#include <opencv2/opencv.hpp>
#include <librealsense/rs.hpp>
#include <memory>
class RealsenseSensor
{
public:
    RealsenseSensor();
    ~RealsenseSensor();

    bool getImages(cv::Mat& rgbImage, cv::Mat& rawDepthImage, const float depthScaleFactorVoid);
private:
    rs::context ctx;
    rs::device* sensor;
    float depthScaleFactor;
    rs::intrinsics depth_intrin,color_intrin;
    rs::extrinsics depth_to_color;
    int nFramesRead;
};


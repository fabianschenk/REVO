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
#include "realsensesensor.h"
#include "../utils/Logging.h"
RealsenseSensor::RealsenseSensor()
{
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0)
    {
        I3D_LOG(i3d::error) << "No Real Sense devices found!!";
        exit(0);
    }

    sensor = ctx.get_device(0);
    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    //rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", sensor->get_name());
    printf("    Serial number: %s\n", sensor->get_serial());
    printf("    Firmware version: %s\n", sensor->get_firmware_version());

    // Configure all streams to run at VGA resolution at 60 frames per second
    sensor->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
    sensor->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 60);
    sensor->start();
    // Retrieve camera parameters for mapping between depth and color
    depth_intrin = sensor->get_stream_intrinsics(rs::stream::depth);
    depth_to_color = sensor->get_extrinsics(rs::stream::depth, rs::stream::color);
    color_intrin = sensor->get_stream_intrinsics(rs::stream::color);
    depthScaleFactor = sensor->get_depth_scale();
    //this should enable outdoor functionality!
    try{
        sensor->set_option(rs::option::color_exposure,sensor->get_option(rs::option::color_enable_auto_exposure));
        sensor->set_option(rs::option::color_backlight_compensation, sensor->get_option(rs::option::color_backlight_compensation));
        sensor->set_option(rs::option::r200_lr_gain,sensor->get_option(rs::option::r200_lr_auto_exposure_enabled));
        sensor->set_option(rs::option::r200_lr_exposure,sensor->get_option(rs::option::r200_lr_auto_exposure_enabled));
        sensor->set_option(rs::option::color_white_balance,sensor->get_option(rs::option::color_enable_auto_white_balance));
    }catch(...)
    {
        I3D_LOG(i3d::error) << "Error setting sensor options!";
    }
    I3D_LOG(i3d::info) << "Please add the following parameters to the sensor config yaml file!";
    I3D_LOG(i3d::info) << "Sensor intrinsics: " << color_intrin.fx << " " << color_intrin.fy << " " << color_intrin.ppx << " " << color_intrin.ppy;
    I3D_LOG(i3d::info) << "Depth intrinsics: " << depth_intrin.fx << " " << depth_intrin.fy << " " << depth_intrin.ppx << " " << depth_intrin.ppy;
    I3D_LOG(i3d::info) << "Size: " << color_intrin.width << "x" <<color_intrin.height << " depthScale: " << depthScaleFactor ;
    std::cout << "Depth coefficients: ";
    for (int d = 0; d < 5; ++d)
        std::cout << depth_intrin.coeffs[d] <<", ";
    std::cout << std::endl;
    std::cout << "Color coefficients: ";
    for (int d = 0; d < 5; ++d)
        std::cout << color_intrin.coeffs[d] <<", ";
    std::cout << std::endl;
    nFramesRead = 0;
}
RealsenseSensor::~RealsenseSensor()
{
    sensor->stop();
}

bool RealsenseSensor::getImages(cv::Mat& rgbImage, cv::Mat& rawDepthImage, const float depthScaleFactorVoid) try
{
    // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
    if (sensor)
    {
        I3D_LOG(i3d::info) << "Waiting for frames";
        sensor->wait_for_frames();
        I3D_LOG(i3d::info) << "Frames received!!";
        nFramesRead++;
        const short * depth_frame = reinterpret_cast<const short *>(sensor->get_frame_data(rs::stream::depth_aligned_to_color));
        I3D_LOG(i3d::info) << "after depth!!";
        const uint8_t * color_image = (const uint8_t *)sensor->get_frame_data(rs::stream::color);
        //Wait for autoexposure to kick in
        if (nFramesRead > 30)
        {
            I3D_LOG(i3d::info) << "after color!!";
            memcpy(rgbImage.data,color_image,rgbImage.cols * rgbImage.rows * 3);
            I3D_LOG(i3d::info) << "after memcpy color!!";
            short* ptIt = (short*)depth_frame;
            const short* ptEnd = depth_frame+rawDepthImage.cols*rawDepthImage.rows;
            int i = 0;
            //convert it to OpenCV float image
            for (;ptIt < ptEnd; ++ptIt,++i)
                rawDepthImage.at<float>(int(i/rawDepthImage.cols),i%rawDepthImage.cols) = (*ptIt)*depthScaleFactor;
            I3D_LOG(i3d::info) << "rawDepth setting!!";

    //        for(int dy=0; dy<depth_intrin.height; ++dy)
    //        {
    //            for(int dx=0; dx<depth_intrin.width; ++dx)
    //            {
    //                // Retrieve the 16-bit depth value and map it into a depth in meters
    //                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
    //                float depth_in_meters = depth_value * scale;

    //                // Skip over pixels with a depth value of zero, which is used to indicate no data
    //                if(depth_value == 0) continue;

    //                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
    //                rs::float2 depth_pixel = {(float)dx, (float)dy};
    //                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
    //                rs::float3 color_point = depth_to_color.transform(depth_point);
    //                rs::float2 color_pixel = color_intrin.project(color_point);

    //                // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
    //                const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
    //                if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
    //                {
    //                    glColor3ub(255, 255, 255);
    //                }
    //                else
    //                {
    //                    glColor3ubv(color_image + (cy * color_intrin.width + cx) * 3);
    //                }

    //                // Emit a vertex at the 3D location of this depth pixel
    //                glVertex3f(depth_point.x, depth_point.y, depth_point.z);
    //            }
    //        }
            return true;
        }
    }
    return false;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}

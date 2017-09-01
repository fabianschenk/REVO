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
//
#include <cstdio>
#include <stdexcept>
#include <iomanip>
#include "OrbbecAstraEngineUVC.h"
//#include "drivers/Include/OpenNI.h"
#include <OpenNI.h>
#include <libuvc/libuvc.h>
#include <time.h>
struct DeviceID {
    int vid, pid;
    bool providesTwoStreams;
    float depthScaleFactor;
} knownDeviceIDs[] = {
    { 0x8086, 0x0a66, true, 1.0f/10000.0f }, // Intel RealSense F200
    { 0x8086, 0x0a80, false, 1.0f/1000.0f },  // Intel RealSense R200
    { 0x2bc5, 0x0403, true, 1.0f/10000.0f },  // match anything, not recommended...
    { 0x2bc5, 0x0501, true, 1.0f/10000.0f },  // match anything, not recommended...
    { 0x2bc5, 0x0193, true, 1.0f/10000.0f }  // match anything, not recommended...
};
class OrbbecAstraEngine::PrivateData {
    public:
    PrivateData(void) : streams(NULL),iteration(0){
        ctx = NULL;
        dev = NULL; devh_d = NULL; devh_rgb = NULL;
        //ctrl_rgb = NULL; ctrl_d = NULL;
        framebuffer_rgb = NULL; framebuffer_depth = NULL;
        got_depth = got_color = 0;
        rgb_stream = NULL;
        frame_desc = NULL;

    }
    uvc_frame_desc_t* frame_desc;
    openni::Device device;
    openni::VideoStream depthStream;//, colorStream;
    openni::VideoFrameRef depthFrame;
    openni::VideoStream **streams;
    int iteration;
    cv::VideoCapture  rgbCameraCap;
    cv::Mat*   rgbFrame;
    openni::RGB888Pixel* m_pTexMap;
    bool rgbIsOpen;
    //UVC
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh_d;
    uvc_device_handle_t *devh_rgb;
    uvc_stream_ctrl_t ctrl_rgb;
    uvc_stream_ctrl_t ctrl_d;
    uvc_stream_handle_t *rgb_stream;
    uvc_frame_t *framebuffer_rgb;
    uvc_frame_t *framebuffer_depth;

    bool hasColour;
    int got_depth, got_color;
};
struct FormatSpec {
    const uvc_frame_desc *frame_desc;
    uint32_t interval;
};
bool formatIdentical(const uint8_t *guidFormat, const char *charFormat)
{
    for (int i = 0; i < 16; ++i) {
        if (*charFormat == 0) return true;
        if (*charFormat != (char)*guidFormat) return false;
        ++charFormat; ++guidFormat;
    }
    return false;
}
void callback_rgb(uvc_frame_t *frame, void *_data)
{
    OrbbecAstraEngine::PrivateData *data = (OrbbecAstraEngine::PrivateData*)_data;
    //std::cout << "callback" << std::endl;

    if (data->framebuffer_rgb == NULL) {
        data->framebuffer_rgb = uvc_allocate_frame(frame->width * frame->height * 3);
    }

    uvc_any2rgb(frame, data->framebuffer_rgb);
    data->got_color = 1;
}
struct FormatSpec findSuitableFormat(uvc_device_handle_t *dev, int requiredResolutionX = -1, int requiredResolutionY = -1, const char *requiredFormat = NULL)
{
    FormatSpec ret;
    ret.frame_desc = NULL; ret.interval = 0;

    const uvc_format_desc_t *formats = uvc_get_format_descs(dev);

    if (requiredFormat != NULL) {
        const uvc_format_desc_t *format_it = formats;
        while (format_it != NULL) {
            printf("%c%c%c%c\n", format_it->guidFormat[0], format_it->guidFormat[1], format_it->guidFormat[2], format_it->guidFormat[3]);
            fprintf(stderr, "%c%c%c%c\n", format_it->guidFormat[0], format_it->guidFormat[1], format_it->guidFormat[2], format_it->guidFormat[3]);
            if (formatIdentical(format_it->guidFormat, requiredFormat)) {
                formats = format_it;
                break;
            } else {
                format_it = format_it->next;
            }
        }
    }

    for (struct uvc_frame_desc *frame_desc = formats->frame_descs; frame_desc != NULL; frame_desc = frame_desc->next) {
        printf("%i %i\n", frame_desc->wWidth, frame_desc->wHeight);
        uint32_t mininterval = frame_desc->intervals[0];
        for (int j = 0; frame_desc->intervals[j] != 0; ++j) {
            if (mininterval > frame_desc->intervals[j]) mininterval = frame_desc->intervals[j];
        }

        bool acceptAsBest = false;
        if (ret.frame_desc == NULL) acceptAsBest = true;
        else if ((frame_desc->wWidth == ret.frame_desc->wWidth) &&
            (frame_desc->wHeight == ret.frame_desc->wHeight) &&
            (mininterval < ret.interval)) {
            acceptAsBest = true;
        } else if ((requiredResolutionX <= 0)&&(requiredResolutionY <= 0)) {
            if (frame_desc->wWidth > ret.frame_desc->wWidth) {
                acceptAsBest = true;
            }
        } else {
            int diffX_cur = abs(frame_desc->wWidth - requiredResolutionX);
            int diffX_best = abs(ret.frame_desc->wWidth - requiredResolutionX);
            int diffY_cur = abs(frame_desc->wHeight - requiredResolutionY);
            int diffY_best = abs(ret.frame_desc->wHeight - requiredResolutionY);
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
        if (acceptAsBest) {
            ret.frame_desc = frame_desc;
            ret.interval = mininterval;
        }
    }

    printf("bestMode: %i %i %i\n", ret.frame_desc->wWidth, ret.frame_desc->wHeight, (int)(10000000.0f*(1.0f/(float)ret.interval)));
    return ret;
}


static openni::VideoMode findBestMode(const openni::SensorInfo *sensorInfo, int requiredResolutionX = -1, int requiredResolutionY = -1, openni::PixelFormat requiredPixelFormat = (openni::PixelFormat)-1)
{
	const openni::Array<openni::VideoMode> & modes = sensorInfo->getSupportedVideoModes();
	openni::VideoMode bestMode = modes[0];
	for (int m = 0; m < modes.getSize(); ++m) {
		//fprintf(stderr, "mode %i: %ix%i, %i %i\n", m, modes[m].getResolutionX(), modes[m].getResolutionY(), modes[m].getFps(), modes[m].getPixelFormat());
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
	//fprintf(stderr, "=> best mode: %ix%i, %i %i\n", bestMode.getResolutionX(), bestMode.getResolutionY(), bestMode.getFps(), bestMode.getPixelFormat());
	return bestMode;
}

//bool OrbbecAstraEngine::initUVCRGB()
//{
//    uvc_error_t res;
//    res = uvc_init(&(data->ctx), NULL);
//    /*if (res < 0) {
//        uvc_perror(res, "LibUVCEngine");
//        break;
//    }*/
//    uvc_device_t **deviceList;
//    uvc_get_device_list((data->ctx),&deviceList);
//    uvc_device_t *dev;
//    int i = 0;
//    printf("Before deviceList\n");
//    uvc_device_descriptor_t *desc;
//    while ((dev = deviceList[i]))
//    {
//        uvc_get_device_descriptor(dev,&desc);
//        printf("pid: %x, vid: %x, manu: %s\n",desc->idProduct,desc->idVendor,desc->manufacturer);
//        if (strstr(desc->manufacturer,"stra"))
//            break;
//        ++i;
//    }
//    printf("After deviceList\n");
//    uvc_free_device_list(deviceList,1);

//    // Step 1: find device
//    //unsigned int deviceID = 0;
//    /*for (; deviceID < sizeof(knownDeviceIDs)/sizeof(struct DeviceID); ++deviceID) {
//        fprintf(stderr, "scanning for devices %x %x\n", knownDeviceIDs[deviceID].vid, knownDeviceIDs[deviceID].pid);
//        res = uvc_find_device(data->ctx, &(data->dev),
//            knownDeviceIDs[deviceID].vid, knownDeviceIDs[deviceID].pid, NULL); /* filter devices: vendor_id, product_id, "serial_num" */
//        //if (res == 0) break;
//    //}

//    if (res < 0) {
//        /* no devices found */
//        uvc_perror(res, "uvc_find_device failed");
//        return false;
//    }
//    uvc_find_device(data->ctx, &(data->dev),desc->idVendor, desc->idProduct, NULL);
//    res = uvc_open2(data->dev, &(data->devh_rgb),0);// cameraID);
//    if (res < 0) {
//        /* no devices found */
//        uvc_perror(res, "uvc_find_device failed");
//        return false;
//    }
//    uvc_print_diag(data->devh_rgb,stderr);
////    uvc_stream_ctrl_t ctrl;
////    /* Try to negotiate a 640x480 30 fps YUYV stream profile */
////    res = uvc_get_stream_ctrl_format_size(
////        data->devh_rgb, &ctrl, /* result stored in ctrl */
////        UVC_FRAME_FORMAT_UNCOMPRESSED, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
////        640, 480, 30 /* width, height, fps */
////    );
////    /* Print out the result */
//////uvc_print_stream_ctrl(&ctrl, stderr);

////    printf("Negotiated YUYV stream\n");
////    fflush(stdout);
////    if (res < 0) {
////           uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
////         } else {
////           /* Start the video stream. The library will call user function cb:
////            *   cb(frame, (void*) 12345)
////            */
////        res = uvc_stream_start(data->rgb_stream,0,0,0);

////           //res = uvc_start_streaming(data->devh_rgb, &ctrl, 0, 12345, 0);
////           if (res < 0) {
////             uvc_perror(res, "start_streaming"); /* unable to start stream */
////           } else {
////             puts("Streaming...");
////             uvc_set_ae_mode(data->devh_rgb, 1); /* e.g., turn on auto exposure */
////             sleep(10); /* stream for 10 seconds */
////             /* End the stream. Blocks until last callback is serviced */
////             uvc_stop_streaming(data->devh_rgb);
////             puts("Done streaming.");
////           }
////         }
////    printf("READ YUYV stream\n");
//    FormatSpec format_rgb;
//    cv::Point2i requested_imageSize_rgb(640,480);
//    format_rgb = findSuitableFormat(data->devh_rgb, requested_imageSize_rgb.x, requested_imageSize_rgb.y);
//    data->hasColour = true;
//    colorAvailable = true;
//    //res = uvc_stream_open_ctrl(data->devh_rgb,&data->rgb_stream,&data->ctrl_rgb);

//    /* Try to negotiate a colour stream */
//        res = uvc_get_stream_ctrl_frame_desc(
//            data->devh_rgb, &(data->ctrl_rgb),
//            format_rgb.frame_desc,
//            format_rgb.interval);
//        if (res < 0) {
//            uvc_perror(res, "get_mode rgb");
//            return false;
//        }
//        res = uvc_stream_open_ctrl(data->devh_rgb,&data->rgb_stream,&data->ctrl_rgb);
//            //res = uvc_get_frame_desc(data->devh_rgb,&data->ctrl_rgb,&data->frame_desc);
//            /*res = uvc_get_stream_ctrl_frame_desc(
//                data->devh_rgb, &(data->ctrl_rgb),
//                format_rgb.frame_desc,
//                format_rgb.interval);*/
//            uvc_set_ae_mode(data->devh_rgb, 1);
//            if (data->framebuffer_rgb == NULL) {
//                std::cout << "Allocating frame" << std::endl;
//                data->framebuffer_rgb = uvc_allocate_frame(640 * 480 * 3);
//            }
//            res = uvc_stream_start(data->rgb_stream,0,0,0);
//            //res = uvc_start_streaming(data->devh_rgb, &(data->ctrl_rgb), callback_rgb, data, 0);
//            if (res < 0) {
//                uvc_perror(res, "start_streaming rgb");
//                return false;
//            }

//            if (format_rgb.frame_desc != NULL) {
//                this->imageSize_rgb = cv::Point2i(format_rgb.frame_desc->wWidth, format_rgb.frame_desc->wHeight);
//            }
//            sleep(1);
//            std::cout << "Width" << format_rgb.frame_desc->wWidth << format_rgb.frame_desc->wHeight << std::endl;
//            return true;

//    return true;
//}


bool OrbbecAstraEngine::initUVCRGB()
{
    uvc_error_t res;
    res = uvc_init(&(data->ctx), NULL);
    /*if (res < 0) {
        uvc_perror(res, "LibUVCEngine");
        break;
    }*/
    uvc_device_t **deviceList;
    uvc_get_device_list((data->ctx),&deviceList);
    uvc_device_t *dev;
    int i = 0;
    printf("Before deviceList\n");
    uvc_device_descriptor_t *desc;
    while ((dev = deviceList[i]))
    {
        uvc_get_device_descriptor(dev,&desc);
        printf("pid: %x, vid: %x, manu: %s\n",desc->idProduct,desc->idVendor,desc->manufacturer);
        if (strstr(desc->manufacturer,"stra"))
            break;
        ++i;
    }
    printf("After deviceList\n");
    uvc_free_device_list(deviceList,1);
    printf("After free deviceList\n");
    if (res < 0) {
        /* no devices found */
        uvc_perror(res, "uvc_find_device failed");
        return false;
    }
    printf("before find deviceList\n");
    uvc_find_device(data->ctx, &(data->dev),desc->idVendor, desc->idProduct, NULL);
    printf("before open2\n");
    res = uvc_open2(data->dev, &(data->devh_rgb),0);
    //res = uvc_open(data->dev, &(data->devh_rgb));
    //uvc_print_diag(data->devh_rgb,stderr);

    if (res < 0) {
        /* no devices found */
        uvc_perror(res, "uvc_find_device failed");
        return false;
    }
    printf("before format_rgb deviceList\n");
    FormatSpec format_rgb;
    cv::Point2i requested_imageSize_rgb(640,480);
    format_rgb = findSuitableFormat(data->devh_rgb, requested_imageSize_rgb.x, requested_imageSize_rgb.y);
    data->hasColour = true;
    colorAvailable = true;

    /* Try to negotiate a colour stream */
    res = uvc_get_stream_ctrl_frame_desc(
        data->devh_rgb, &(data->ctrl_rgb),
        format_rgb.frame_desc,
        format_rgb.interval);
    if (res < 0) {
        uvc_perror(res, "get_mode rgb");
        return false;
    }
    /* Try to negotiate a 640x480 30 fps YUYV stream profile */
//    res = uvc_get_stream_ctrl_format_size(
//                data->devh_rgb, &data->ctrl_rgb, /* result stored in ctrl */
//                UVC_FRAME_FORMAT_UNCOMPRESSED,//UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED     UVC_FRAME_FORMAT_RGB*/
//                requested_imageSize_rgb.y, requested_imageSize_rgb.x, 30 /* width, height, fps */
//                );
    res = uvc_stream_open_ctrl(data->devh_rgb,&data->rgb_stream,&data->ctrl_rgb);
    //res = uvc_get_frame_desc(data->devh_rgb,&data->ctrl_rgb,&data->frame_desc);
    /*res = uvc_get_stream_ctrl_frame_desc(
        data->devh_rgb, &(data->ctrl_rgb),
        format_rgb.frame_desc,
        format_rgb.interval);*/
    //mode	1: manual mode; 2: auto mode; 4: shutter priority mode; 8: aperture priority mode
    //uvc_set_ae_mode(data->devh_rgb, 8);
    //uvc_set_power_line_frequency(data->devh_rgb,50);
    if (data->framebuffer_rgb == NULL) {
        std::cout << "Allocating frame" << std::endl;
        data->framebuffer_rgb = uvc_allocate_frame(640 * 480 * 3);
    }
    res = uvc_stream_start(data->rgb_stream,0,0,0);
    //res = uvc_start_streaming(data->devh_rgb, &(data->ctrl_rgb), callback_rgb, data, 0);
    if (res < 0) {
        uvc_perror(res, "start_streaming rgb");
        return false;
    }

    if (format_rgb.frame_desc != NULL) {
        this->imageSize_rgb = cv::Point2i(format_rgb.frame_desc->wWidth, format_rgb.frame_desc->wHeight);
    }
    sleep(1);
    std::cout << "Width" << format_rgb.frame_desc->wWidth << format_rgb.frame_desc->wHeight << std::endl;
    return true;
}

bool OrbbecAstraEngine::initOpenNIDepth()
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

    data->streams = new openni::VideoStream*[2];
    if (depthAvailable) data->streams[0] = &data->depthStream;
    return true;
}

OrbbecAstraEngine::OrbbecAstraEngine():initSuccess(false)
{
    this->imageSize_d = cv::Point2i(0,0);
    this->imageSize_rgb = cv::Point2i(0,0);
	data = new PrivateData();
    initSuccess = initOpenNIDepth();
    if (initSuccess) initSuccess = initUVCRGB();
}

OrbbecAstraEngine::~OrbbecAstraEngine()
{
    std::cout << "DESTRUCTOR called!!" << std::endl;
	if (data != NULL)
	{
		if (depthAvailable)
		{
			data->depthStream.stop();
			data->depthStream.destroy();
		}
		data->device.close();

        if (data->rgbIsOpen) data ->rgbFrame = NULL;

        if (data->rgb_stream != NULL) {
            uvc_stream_close(data->rgb_stream);
        }
        /* End the stream. Blocks until last callback is serviced */
        if (data->devh_rgb != NULL) {
            uvc_stop_streaming(data->devh_rgb);
        }
        if (data->devh_rgb != NULL) {
            uvc_close(data->devh_rgb);
        }

        if (data->framebuffer_rgb == NULL) {
            uvc_free_frame(data->framebuffer_rgb);
        }
        /* Release the device descriptor */
        if (data->dev != NULL) uvc_unref_device(data->dev);
        if (data->ctx != NULL) uvc_exit(data->ctx);
		delete[] data->streams;
		delete data;
	}
	openni::OpenNI::shutdown();
}

bool OrbbecAstraEngine::getImages(cv::Mat& rgbImage, cv::Mat& rawDepthImage, const float depthScaleFactor)
{
    clock_t start = clock();
    if (!initSuccess)
    {
        std::cout << "Init not successful!";
        return false;
    }
    int32_t timeout = 30; //ms

    std::cout << "Before frame" << std::endl;
    //wait indefinitly
    uvc_frame_t *frame;
    uvc_error_t result = uvc_stream_get_frame( data->rgb_stream, &frame, 0);


    //Read color and depth first and then preprocess!
    if (result != UVC_SUCCESS || frame == NULL)
    {
        std::cout << "could not get uvc frame";
        return false;
    }
    result = uvc_any2rgb(frame, data->framebuffer_rgb);
    if (result != UVC_SUCCESS)
    {
        std::cout << "could not get uvc frame" << std::endl;
        return false;
    }

    int changedIndex, waitStreamCount = 1;
    openni::Status rc = openni::OpenNI::waitForAnyStream(data->streams, waitStreamCount, &changedIndex);
    if (rc != openni::STATUS_OK)
    {
        printf("OpenNI: Wait failed\n");
        return false;
    }

    if (depthAvailable) data->depthStream.readFrame(&data->depthFrame);
    if (depthAvailable && !data->depthFrame.isValid()) return false;

    //data->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    //std::cout << "HERE2" << std::endl;
    //short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);

    //openni::DepthPixel* depthImagePix = (openni::DepthPixel*)data->depthFrame.getData();
    //Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);

    //Process color

    //cv::Mat rgbImage = cv::Mat(frame->height,frame->width,CV_8UC3,data->framebuffer_rgb->data);
    //std::cout << "Here" << std::endl;
    //std::cout << "buffer: " << data->framebuffer_rgb << std::endl;
    //std::cout << "buffer->data: " << data->framebuffer_rgb->data << " " << rgbImage.cols * rgbImage.rows * 3 << std::endl;
    memcpy(rgbImage.data,data->framebuffer_rgb->data,rgbImage.cols * rgbImage.rows * 3);
    //std::cout << "after memcpy" << std::endl;
    //cv::imshow("rgbImage",rgbImage);
    //cv::waitKey(0);
    cv::cvtColor(rgbImage,rgbImage,cv::COLOR_RGB2BGR);
    //memcpy(rawDepthImage.data,data->depthFrame.getData(),rawDepthImage.cols*rawDepthImage.rows * sizeof(short));
    //rawDepthImage.convertTo(rawDepthImage,CV_32FC1,1/depthScaleFactor);
    short* ptIt = (short*)data->depthFrame.getData();
    const short* ptEnd = (short*)(data->depthFrame.getData())+rawDepthImage.cols*rawDepthImage.rows;
    int i = 0;
    //convert it to OpenCV float image
    for (;ptIt < ptEnd; ++ptIt,++i)
        rawDepthImage.at<float>(int(i/rawDepthImage.cols),i%rawDepthImage.cols) = (*ptIt)/depthScaleFactor;
    clock_t end = clock();
    std::cout << "Orbbec Time: " << double(end - start)/CLOCKS_PER_SEC*1000.0f << std::endl;
    return true; /*true*/;
}

bool OrbbecAstraEngine::hasMoreImages(void) { return (data!=NULL); }
cv::Point2i OrbbecAstraEngine::getDepthImageSize(void) { return (data!=NULL)?imageSize_d:cv::Point2i(0,0); }
cv::Point2i OrbbecAstraEngine::getRGBImageSize(void) { return (data!=NULL)?imageSize_rgb:cv::Point2i(0,0); }


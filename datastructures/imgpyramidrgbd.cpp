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
#include "imgpyramidrgbd.h"
#include "../utils/timer.h"
#include <opencv2/highgui.hpp>
std::vector<double> ImgPyramidRGBD::dtTimes;

ImgPyramidRGBD::ImgPyramidRGBD(const ImgPyramidSettings& settings, const std::shared_ptr<CameraPyr> &cameraPyr):
    cameraPyr(cameraPyr),timeStamp(0), mSettings(settings),mIsStored(false)

{
    I3D_LOG(i3d::info) << "ImgPyramidRGBD normal";
}

ImgPyramidRGBD::~ImgPyramidRGBD()
{

    for (int lvl = 0; lvl < optimizationStructure.size();++lvl)
    {
        Eigen::internal::aligned_free((void*) optimizationStructure[lvl]);
        optimizationStructure.clear();
        optimizationStructureBuilt[lvl] = false;
    }
}

ImgPyramidRGBD::ImgPyramidRGBD(const ImgPyramidSettings& settings, const std::shared_ptr<CameraPyr>& cameraPyr, const cv::Mat& fullResRgb,
                               const  cv::Mat& fullResDepth,const double timestamp)
                :cameraPyr(cameraPyr),timeStamp(timestamp),mSettings(settings),mIsStored(false)
{
     I3D_LOG(i3d::detail) << "ImgPyramidRGBD extended";
     auto startPyr = Timer::getTime();
     //Chosen in a way that we always get 32x24 Patches for 3 levels starting from 640x480
     distPatchSizes.push_back(20);distPatchSizes.push_back(10);distPatchSizes.push_back(5);
     rgbFullSize = fullResRgb.clone();
     cv::Mat gray;
     cv::cvtColor(rgbFullSize,gray,CV_BGRA2GRAY);
     cv::Mat depth = fullResDepth.clone();
     auto endScale = Timer::getTime();
     I3D_LOG(i3d::info) << "After downsize: " <<Timer::getTimeDiffMiS(startPyr,endScale);
     if (settings.DO_UNDISTORT)
     {
         cv::Mat gray2;
         cv::remap(gray,gray2,cameraPyr->map1,cameraPyr->map2,CV_INTER_LINEAR);
         cv::Mat depth2;
         cv::remap(depth,depth2,cameraPyr->map1,cameraPyr->map2,CV_INTER_LINEAR);
         gray = gray2;
         depth = depth2;
     }
     auto startLvl = Timer::getTime();
     this->addLevelEdge(gray,depth,cameraPyr->at(0));
     auto endLvl = Timer::getTime();
     I3D_LOG(i3d::info) << "AddLevel 0: " << Timer::getTimeDiffMiS(startLvl,endLvl);
     I3D_LOG(i3d::info) << "Time for creating pyramid after 0: " << Timer::getTimeDiffMiS(startPyr,endLvl);
//     cv::imwrite("./out/depth"+std::to_string(timestamp)+".png",depth);
//     cv::imwrite("./out/gray"+std::to_string(timestamp)+".png",gray);
     //cv::imwrite("./out/edges"+std::to_string(timestamp)+".png",this->edgesPyr[0]);
     //TODO: CONVERT COLOR TO RGB AND ONLY DOWNSIZE
     for (int lvl = 1; lvl < settings.nLevels(); ++lvl)
     {
         auto startLvl = Timer::getTime();
         I3D_LOG(i3d::detail) << "lvl: " << lvl;
         const cv::Size2i currSize = cameraPyr->at(lvl).returnSize();
         //This somehow gives strange edges at the borders!
         cv::Mat downResGray(currSize.height,currSize.width,CV_8UC1);
         cv::pyrDown(gray,downResGray);
         cv::Mat downResDepth(currSize.height,currSize.width,CV_32FC1);
         this->FilterSubsampleWithHoles(downResDepth,depth);
         this->addLevelEdge(downResGray,downResDepth,cameraPyr->at(lvl));
         gray = downResGray;
         depth = downResDepth;
         auto endLvl = Timer::getTime();
         I3D_LOG(i3d::info) << "AddLevel " << lvl << ": " << Timer::getTimeDiffMiS(startLvl,endLvl);
         I3D_LOG(i3d::info) << "Time for creating pyramid after " <<lvl<<": " << Timer::getTimeDiffMiS(startPyr,endLvl);

     }
     auto endPyr = Timer::getTime();
     //if (timestamp == 531) exit(0);
     I3D_LOG(i3d::info) << "Time for creating pyramid: " << Timer::getTimeDiffMiS(startPyr,endPyr);
}
void ImgPyramidRGBD::getEdges(cv::Mat& edges, const cv::Mat& gray)
{
    I3D_LOG(i3d::detail) << mSettings.cannyThreshold1 << " " << mSettings.cannyThreshold2;

    if (mSettings.DO_GAUSSIAN_SMOOTHING_BEFORE_CANNY)
    {
        cv::Mat smoothImg;
        cv::GaussianBlur(gray,smoothImg,cv::Size(7,7),2);
        cv::Canny(smoothImg,edges,mSettings.cannyThreshold1,mSettings.cannyThreshold2,3,true); //...edges,150,100,...
    }
    else
        cv::Canny(gray,edges,mSettings.cannyThreshold1,mSettings.cannyThreshold2,3,true); //...edges,150,100,...
}

void ImgPyramidRGBD::fillInEdges(size_t lvl)
{
    I3D_LOG(i3d::info) << "fillInEdges(int lvl): " << lvl << " " << histPyr.size() << " " << edgesPyr.size();
    const cv::Mat dist = histPyr[lvl]; //get top level
    const cv::Mat topEdges = edgesPyr[lvl-1]; //get top level
    cv::Mat edgesMod = edgesPyr[lvl];
    //cv::imwrite("edges_"+std::to_string(lvl-1)+".png",topEdges);
    //cv::imwrite("edges_"+std::to_string(lvl)+".png",edgesMod);
//    cv::imshow("lvl_0",edgesPyr[0]);
//    cv::waitKey(0);
//    cv::imshow("lvl_1",edgesPyr[1]);
//    cv::waitKey(0);
    const int PATCH_SIZE = this->distPatchSizes[lvl];
    const int PATCH_SIZE_2=PATCH_SIZE*PATCH_SIZE;
    const int PATCH_SIZE_LOW = this->distPatchSizes[lvl-1];
    //I3D_LOG(i3d::info) << "edges: " << edgesMod.size() << "PATCH_SIZE: " << PATCH_SIZE << " LOW: "<<PATCH_SIZE_LOW << dist;
    //cv::imshow("edgesTop: ", topEdges);
//    cv::imshow("edgesPrev: ", edgesMod);
//    cv::imshow("dist",dist*(255.0/PATCH_SIZE_2));
    for (int yy = 0; yy < topEdges.rows;++yy)
        for (int xx = 0; xx < topEdges.cols;++xx)
        {
            if ((yy%2 == 1) && (xx%2==1) && dist.at<u_int8_t>(floor(yy/PATCH_SIZE_LOW),floor(xx/PATCH_SIZE_LOW)) < PATCH_SIZE_2*0.05)
            {
                if (topEdges.at<u_int8_t>(yy,xx)>0)
                    edgesMod.at<u_int8_t>(floor(yy/2),floor(xx/2)) = 255 ;
                //if (topEdges.at<u_int8_t>(yy,xx)>0)
                //edgesMod.at<u_int8_t>(floor(yy/2),floor(xx/2)) += topEdges.at<u_int8_t>(yy,xx);
            }
        }
    //cv::imwrite("edgesMod"+std::to_string(lvl)+".png",edgesMod);
//    cv::imshow("edgesMod", edgesMod);
//    cv::imwrite("dist"+std::to_string(lvl)+".png",dist*(255.0/PATCH_SIZE_2));
//    cv::waitKey(0);
}
float ImgPyramidRGBD::generateDistHistogram(const cv::Mat& edges, int PATCH_SIZE)
{
    cv::Mat dist = cv::Mat(edges.rows/PATCH_SIZE,edges.cols/PATCH_SIZE,CV_8UC1,cv::Scalar(0));
    I3D_LOG(i3d::detail) << "edges: " << edges.size() << "PATCH_SIZE: " << PATCH_SIZE << dist.size();
    for (int yy = 0; yy < edges.rows;++yy)
        for (int xx = 0; xx < edges.cols;++xx)
        {
            if (edges.at<u_int8_t>(yy,xx) > 0)
                dist.at<u_int8_t>(floor(yy/PATCH_SIZE),floor(xx/PATCH_SIZE))++;//= dist.at<u_int8_t>(floor(yy/PATCH_SIZE),floor(xx/PATCH_SIZE))+1;

                //dist.at<u_int8_t>(floor(yy/PATCH_SIZE),floor(xx/PATCH_SIZE))= dist.at<u_int8_t>(floor(yy/PATCH_SIZE),floor(xx/PATCH_SIZE))+1;
        }
    I3D_LOG(i3d::detail) << "After for!";
    const int nDist = cv::countNonZero(dist);
    I3D_LOG(i3d::detail) << "After countNonZero!";
    //int nEdges = cv::countNonZero(edges);
    //I3D_LOG(i3d::info) << "edges: " << edges.size() << " " << nEdges << " percent: " << double(nEdges)/edges.total() << "dist: " <<  nDist << "=> percent " << double(nDist)/(32.0f*24.0f);
    //cv::imshow("topEdges",topEdges);
    //cv::imshow("edgesHist",edges);
    this->histPyr.push_back(dist);
    //int max_dist = PATCH_SIZE*PATCH_SIZE;
    return float(nDist)/float(dist.total());
    //cv::imshow("dist",dist*(255.0/max_dist));
    //cv::imwrite("dist_"+std::to_string(PATCH_SIZE)+".png",dist);
//    cv::imwrite("edge_"+std::to_string(PATCH_SIZE)+".png",edges);
//    cv::waitKey(0);
}
void ImgPyramidRGBD::addLevelEdge(const cv::Mat &gray,const cv::Mat &depth, const Camera& cam)
{
    clock_t start = clock();
    this->grayPyr.push_back(gray);
    this->depthPyr.push_back(depth);
//    cv::imshow("gray",gray);
//    cv::imshow("depth",depth);
//    cv::waitKey(0);

    cv::Mat edges;
    const size_t lvl = static_cast<size_t>(mSettings.PYR_MAX_LVL)+edgesPyr.size();
    cv::Canny(gray,edges,mSettings.cannyThreshold1,mSettings.cannyThreshold2,3,true);
    this->edgesOrigPyr.push_back(edges.clone());
    this->edgesPyr.push_back(edges);
    const float nPercentage = generateDistHistogram(edges,distPatchSizes[lvl]);
    if (mSettings.USE_EDGE_HIST && edgesPyr.size() > 1)
    {
        I3D_LOG(i3d::info) << "mSettings.USE_EDGE_HIST: " << mSettings.USE_EDGE_HIST;

        if (nPercentage < mSettings.nPercentage)
        {
            fillInEdges(lvl);
        }
    }

    //it is safe to assume that only 20 % of pixels are interesting
    Eigen::MatrixXf ref3dList = Eigen::MatrixXf(4,int(cam.area));
    //Eigen::MatrixXf ref3dListDepth = Eigen::MatrixXf(4,int(cam.area/3.0));
    I3D_LOG(i3d::info) << "Allocated " << ref3dList.cols() << " at size " << cam.width << "x"<<cam.height << "edges: " << edges.cols << " "<< edges.rows;
    int linIdx = 0;//, linIdxDepth = 0;
    for( int xx = 0; xx < edges.cols; xx++ )
    {
        for( int yy = 0 ; yy < edges.rows; yy++ )
        {
            const float Z = depth.at<float>(yy,xx);
            if (std::isfinite(Z) && Z > mSettings.DEPTH_MIN && Z < mSettings.DEPTH_MAX)
            {
                const uint8_t edgePx = edges.at<uint8_t>(yy,xx);
                if (edgePx > 0)
                {
                    const float X = Z * (xx-cam.cx) / cam.fx;
                    const float Y = Z * (yy-cam.cy) / cam.fy;
                    ref3dList.col(linIdx) = Eigen::Vector4f(X,Y,Z,1.0f);
                    linIdx++;
                    if (linIdx < 50)
                    {
                        I3D_LOG(i3d::detail) << std::fixed << linIdx << ": " << ref3dList.col(linIdx-1).transpose();
                    }
                }
            }
        }
    }
    I3D_LOG(i3d::info) << "Lvl: " << lvl << "nEdges: " << linIdx;// << modDepth;
    this->edges3DPyr.push_back(ref3dList.leftCols(linIdx));
    clock_t end = clock();
    I3D_LOG(i3d::info) << "addLevel actual time: " << double(end-start)/CLOCKS_PER_SEC*1000.0f;
}
//Compute the distance transform and the acceleration structure!
void ImgPyramidRGBD::makeKeyframe()
{
    //finally, the computation of the distance transform
    auto start = Timer::getTime();
    for (int lvl = 0; lvl < mSettings.nLevels(); ++lvl)
    {
        const Camera cam = cameraPyr->at(lvl);
        //cv::imshow("edgesPyr.at(lvl)",edgesPyr.at(lvl));
        //cv::waitKey(0);
        cv::Mat dt;
        cv::distanceTransform(255-edgesPyr.at(lvl),dt,CV_DIST_L2, CV_DIST_MASK_PRECISE);
        this->dtPyr.push_back(dt);
        I3D_LOG(i3d::info) << "buildOptimizationStructure: " << int(cam.area)*sizeof(Eigen::Vector4f) << "sizeof: "<< sizeof(Eigen::Vector4f);
        Eigen::Vector4f* optStruc = (Eigen::Vector4f*)Eigen::internal::aligned_malloc(int(cam.area)*sizeof(Eigen::Vector4f));
        buildOptimizationStructure(dt,optStruc);
        optimizationStructure.push_back(optStruc);
        optimizationStructureBuilt.push_back(true);
    }
    auto end = Timer::getTime();
    ImgPyramidRGBD::dtTimes.push_back(Timer::getTimeDiffMs(start,end));
    I3D_LOG(i3d::info) << "Computation of DT: " << Timer::getTimeDiffMs(start,end);
}


void ImgPyramidRGBD::buildOptimizationStructure(const cv::Mat& floatImg, Eigen::Vector4f* startPt) const
{
    const int width = floatImg.cols;
    const int height = floatImg.rows;
    const float* img_pt = ((float*)floatImg.data) + width;
    const float* img_pt_max =((float*)floatImg.data) + width*(height-1);
    Eigen::Vector4f* gradxyii_pt = startPt+width;//distGradients + width;

    // in each iteration i need -1,0,p1,mw,pw
    float val_prev = *(img_pt-1);
    float val_center = *img_pt;
    float val_next;
    for(; img_pt < img_pt_max; img_pt++, gradxyii_pt++)
    {
        val_next = *(img_pt+1);
        *((float*)gradxyii_pt) = 0.5f*(val_prev-val_next);
        *(((float*)gradxyii_pt)+1) = 0.5f*(*(img_pt-width)-*(img_pt+width));
        *(((float*)gradxyii_pt)+2) = val_center;
        val_prev = val_center;
        val_center = val_next;
    }
}


void ImgPyramidRGBD::generateColoredPcl(uint lvl, Eigen::MatrixXf& clrPcl, bool densePcl) const
{
    const Camera cam = cameraPyr->at(lvl);
    //we always use xyz1,rgb1
    const int pclSize = int(densePcl ? cam.area : cam.area/5.0);
    clrPcl = Eigen::MatrixXf(8,pclSize);
    //downsize the image!
    cv::Mat rgb;
    if (lvl == 0)
        rgb = rgbFullSize.clone();
    if (lvl == 1)
        cv::pyrDown(rgbFullSize,rgb);
    if (lvl == 2)
    {
        cv::Mat rgbTmp;
        cv::pyrDown(rgbFullSize,rgbTmp);
        cv::pyrDown(rgbTmp,rgb);
    }
    const cv::Mat edges = edgesPyr[lvl];
    const cv::Mat depth = depthPyr[lvl];
    size_t linIdx = 0;
    //Now compute the pcl of the selected points!
    for( int xx=0 ; xx<rgb.cols ; xx++ )
    {
        for( int yy=0 ; yy<rgb.rows ; yy++ )
        {
            const float Z = depth.at<float>(yy,xx);
            if (isPointOkDepth(Z))
            {
                const uint8_t edgePx = edges.at<uint8_t>(yy,xx);
                if (densePcl || edgePx > 0) // dense or valid edge
                {
                    Eigen::VectorXf v(8);
                    const cv::Vec3b clr = rgb.at<cv::Vec3b>(yy,xx);
                    const float X = Z * (xx-cam.cx) / cam.fx;
                    const float Y = Z * (yy-cam.cy) / cam.fy;
                    Eigen::Vector4f p3D(X,Y,Z,1);
                    //const Eigen::Vector4f p3D_w = globTrans*p3D;
                    v << p3D[0], p3D[1], p3D[2], 1, clr[2]/255.0f,clr[1]/255.0f,clr[0]/255.0f,1;
                    clrPcl.col(linIdx) = v;
                    linIdx++;
                }
            }
        }
    }
    I3D_LOG(i3d::info) << clrPcl.cols() << " " << clrPcl.rows();
    clrPcl.conservativeResize(clrPcl.rows(),linIdx);
    //clrPcl = clrPcl.leftCols(linIdx;
}

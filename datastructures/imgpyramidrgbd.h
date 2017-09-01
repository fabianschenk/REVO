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
#include <string>
#include <Eigen/Eigen>
#include "camerapyr.h"
#include <memory>


class ImgPyramidRGBD
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static std::vector<double> dtTimes; //for time measurement
    ImgPyramidRGBD();
    void addLevelEdge(const cv::Mat &gray,const cv::Mat &depth, const Camera& cam);
    void getEdges(cv::Mat& edges, const cv::Mat& gray);
    //generates a dense or sparse (edges only) PCL in the respective camera coordiante system
    void generateColoredPcl(uint lvl, Eigen::MatrixXf& clrPcl, bool densePcl) const;

    ImgPyramidRGBD(const ImgPyramidSettings& mSettings, const std::shared_ptr<CameraPyr> &cameraPyr);
    ImgPyramidRGBD(const ImgPyramidSettings& mSettings, const std::shared_ptr<CameraPyr> &cameraPyr, const cv::Mat& fullResRgb, const  cv::Mat& fullResDepth,
                   const double timestamp);
    ~ImgPyramidRGBD();
    std::shared_ptr<CameraPyr> cameraPyr;
    void makeKeyframe();
    //return methods
    const Eigen::Matrix3f returnK(uint lvl) const
    {
        Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
        if (lvl >= 0 || lvl < cameraPyr->size())
        {
            K(0,0) = cameraPyr->at(lvl).fx;
            K(1,1) = cameraPyr->at(lvl).fy;
            K(0,2) = cameraPyr->at(lvl).cx;
            K(1,2) = cameraPyr->at(lvl).cy;
        }
        return K;
    }
    const cv::Mat& returnDistTransform(uint lvl) const
    {
        assert(lvl>=0 && lvl < dtPyr.size());
        return this->dtPyr.at(lvl);
    }
    const cv::Mat& returnEdges(uint lvl) const
    {
        assert(lvl>=0 && lvl < edgesPyr.size());
        return edgesPyr.at(lvl);
    }
    const cv::Mat& returnOrigEdges(uint lvl) const
    {
        if (mSettings.USE_EDGE_HIST && int(lvl) > mSettings.PYR_MAX_LVL)
        {
            assert(lvl>=0 && lvl < edgesOrigPyr.size());
            return edgesOrigPyr.at(lvl);
        }
        return returnEdges(lvl);
    }
    const Eigen::MatrixXf& return3DEdges(uint lvl) const
    {
        assert(lvl>=0 && lvl < edges3DPyr.size());
        //if (isAccelerationStructureBuilt(lvl)) exit(0);
        return this->edges3DPyr.at(lvl);
    }
    const cv::Mat& returnDepth(uint lvl) const
    {
        assert(lvl>=0 && lvl < depthPyr.size());
        return this->depthPyr.at(lvl);
    }
    const cv::Mat& returnGray(uint lvl) const
    {
        assert(lvl>=0 && lvl < grayPyr.size());
        return this->grayPyr.at(lvl);
    }
    const cv::Mat& returnRgbFullSize() const
    {
        assert(rgbFullSize.total()>0);
        return rgbFullSize;
    }
    inline double returnTimestamp() const
    {
        return timeStamp;
    }
    inline uint returnMaxLvl() const
    {
        return this->mSettings.PYR_MAX_LVL;
    }
    inline uint returnMinLvl() const
    {
        return this->mSettings.PYR_MIN_LVL;
    }
    const Eigen::Vector4f* returnOptimizationStructure(uint lvl) const
    {
        assert(lvl>=0 && lvl < optimizationStructure.size());

        if (optimizationStructureBuilt[lvl])
            return optimizationStructure[lvl];
        I3D_LOG(i3d::error) << "optimizationStructure not built!";
        exit(0);
    }
    //end return methods

    //we might remove one of those and create them on demand!
private:
    Eigen::Matrix4f T_f_w; //transformation world to frame
    Eigen::Matrix4f T_w_f; //transformation frame to world
public:
    int frameId;
    void setTwf(const Eigen::Matrix4f& T)
    {
        T_w_f = T;
        T_f_w = T.inverse();
    }
    void setTwf(const Eigen::Matrix3f& R, const Eigen::Vector3f& T)
    {
        T_w_f = Eigen::Matrix4f::Identity();
        T_w_f.block<3,3>(0,0) = R;
        T_w_f.block<3,1>(0,3) = T;
        T_f_w = T_w_f.inverse();
    }

    void setTfw(const Eigen::Matrix3f& R, const Eigen::Vector3f& T)
    {
        T_f_w = Eigen::Matrix4f::Identity();
        T_f_w.block<3,3>(0,0) = R;
        T_f_w.block<3,1>(0,3) = T;
        T_w_f = T_f_w.inverse();
    }


    const inline Eigen::Matrix4f getTransKFtoWorld() const
    {
        return T_w_f;
    }
    //for the keyframe, we only need the following things:
    // 1. world pose
    // 2. 3D edges
    // 3. optimization structure
    void prepareKfForStorage()
    {
        mIsStored = true;
        //make sure that we have a keyframe
        if (dtPyr.size() == 0 || mIsStored) return;
        //we probably need the distance transform and the 3D edges
        edgesPyr.clear();
        depthPyr.clear();
        grayPyr.clear();
        rgbFullSize.release();
        edgesOrigPyr.clear();
        dtPyr.clear();
        mIsStored = true;
    }
    inline bool isPointOkDepth(const float depthVal) const
    {
        return (std::isfinite(depthVal) && depthVal > mSettings.DEPTH_MIN && depthVal < mSettings.DEPTH_MAX);
    }
private:

    inline bool isPointOkEdgePyr(const float edgePixel, const float depthVal) const
    {
        return (edgePixel > 0 && std::isfinite(depthVal) && depthVal > mSettings.DEPTH_MIN &&
                depthVal < mSettings.DEPTH_MAX);
    }
    void buildOptimizationStructure(const cv::Mat& floatImg, Eigen::Vector4f* startPt) const;

    void fillInEdges(size_t lvl);
    float generateDistHistogram(const cv::Mat& edges, int PATCH_SIZE);

    cv::Mat rgbFullSize;
    std::vector<int> distPatchSizes;

    double timeStamp;
    ImgPyramidSettings mSettings;

    //intensity images are only needed for edge computation and visualization
    std::vector<cv::Mat> grayPyr;
    //depth is only needed for pcl generation
    std::vector<cv::Mat> depthPyr;
    std::vector<cv::Mat> edgesOrigPyr;
    std::vector<cv::Mat> edgesPyr;

    //this is used for "filling in" edges from the top, BMVC 2017
    std::vector<cv::Mat> histPyr;

    //the pcl of edges
    std::vector<Eigen::MatrixXf> edges3DPyr;

    // for keyframes only
    std::vector<cv::Mat> dtPyr; //distance transform

    std::vector<bool> optimizationStructureBuilt;

    //std::vector<std::shared_ptr<Eigen::Vector4f>,Eigen::aligned_allocator<Eigen::Vector4f>> optimizationStructure;
    std::vector<Eigen::Vector4f*> optimizationStructure;

    //end for keyframes only
    bool mIsStored;

    //For depth, we need to deal with the special case, when depth is invalid (is equal to 0).
    //this was partly taken from InfiniTAM (https://github.com/victorprad/InfiniTAM)
    void FilterSubsampleWithHoles(cv::Mat& out, const cv::Mat& in) const
    {
        if (!(out.rows == int(in.rows/2) && out.cols == int(in.cols/2))) return;

        const float *imageData_in = (float*)in.data;
        float *imageData_out = (float*) out.data;

        for (int y = 0; y < out.rows; y++) for (int x = 0; x < out.cols; x++)
            filterSubsampleWithHoles(imageData_out, x, y, out.cols, imageData_in, in.cols);
    }
    inline void filterSubsampleWithHoles(float *imageData_out, const int x, const int y,
                                         const int newWidth, const float *imageData_in, const int origWidth) const
    {
        const int src_pos_x = x * 2, src_pos_y = y * 2;
        float pixel_out = 0.0f, pixel_in, no_good_pixels = 0.0f;

        pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 0) * origWidth];
        if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

        pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 0) * origWidth];
        if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

        pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 1) * origWidth];
        if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

        pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 1) * origWidth];
        if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

        if (no_good_pixels > 0) pixel_out /= no_good_pixels;

        imageData_out[x + y * newWidth] = pixel_out;
    }
};

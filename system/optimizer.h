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
*
* This optimizer is a modified version of LSD-SLAM's SE3 Tracker http://vision.in.tum.de/lsdslam
* "LSD-SLAM: Large-scale direct monocular SLAM, Engel et al., ECCV 2014"
*/
#pragma once
#define ENABLE_SSE
#define SSEE(val,idx) (*(((float*)&val)+idx))
#include <vector>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <memory>
#include "../datastructures/imgpyramidrgbd.h"
#include "../utils/LGSX.h"
#include "../utils/Logging.h"
#include <chrono>

//using namespace lsd_slam;
#define PYRAMID_LEVELS 6
using namespace lsd_slam;
/**
 * This optimizer is a modified version of "LSD-SLAM: Large-scale direct monocular SLAM, Engel et al., ECCV 2014"
 */
class OptimizerSettings
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    inline OptimizerSettings()
    {
        // Set default settings
        if (PYRAMID_LEVELS > 6)
            printf("WARNING: Sim3Tracker(): default settings are intended for a maximum of 6 levels!");

        //Levenberg-Marquardt parameters
        lambdaSuccessFac = 0.5f;
        lambdaFailFac = 2.0f;
        const float stepSizeMinc[6] = {1e-16f, 1e-16f, 1e-16f, 1e-16f, 1e-16f, 1e-16f};
        const int maxIterations[6] = {100, 100, 100, 100, 100, 100};

        //edge filter distance
        const int edgeDistance[6] = {30,20,10,5,5,5};

        for (int level = 0; level < PYRAMID_LEVELS; ++ level)
        {
            lambdaInitial[level] = 0;
            stepSizeMin[level] = stepSizeMinc[level];
            convergenceEps[level] = 0.999f;
            maxItsPerLvl[level] = maxIterations[level];
            edgeDistanceLvl[level] = edgeDistance[level];
        }
        maxIncTry = 10;
        lambdaInitialTestTrack = 0;
        stepSizeMinTestTrack = 1e-3;
        convergenceEpsTestTrack = 0.98;
        maxItsTestTrack = 15;
        //EVO Settings
        huber_edge = 0.3;
        USE_STANDARD_HUBER = true;
        huber_max_weight = 10.0f;
        checkGradients = false;
        DO_SHOW_DEBUG_IMAGES = false;
        USE_EDGE_FILTER = false;
        maxImgSize = cv::Size2i(640,480);
        nPyrLvl = 3;
        nCamsToOptimize = 1;
        DO_SHOW_RESIDUAL_DEBUG_IMAGES = false;
    }

    float lambdaSuccessFac;
    float lambdaFailFac;
    float lambdaInitial[PYRAMID_LEVELS];
    float stepSizeMin[PYRAMID_LEVELS];
    float convergenceEps[PYRAMID_LEVELS];
    int maxItsPerLvl[PYRAMID_LEVELS];
    float edgeDistanceLvl[PYRAMID_LEVELS];
    float scaleFactorR;
    float lambdaInitialTestTrack;
    float stepSizeMinTestTrack;
    float convergenceEpsTestTrack;
    float maxItsTestTrack;
    int maxIncTry;
    float huber_edge;

    bool USE_STANDARD_HUBER; //false = double huber
    float huber_max_weight;
    float filter_depth_threshold;
    bool checkGradients;
    bool DO_SHOW_DEBUG_IMAGES;
    bool USE_EDGE_FILTER;
    cv::Size2i maxImgSize;
    int nCamsToOptimize;
    int nPyrLvl;
    bool DO_SHOW_RESIDUAL_DEBUG_IMAGES;
};

class Optimizer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    class ResidualInfo
    {
    public:
        ResidualInfo()
        {
            clearAll();
        }
        int goodPtsEdges, badPtsEdges, badOutOfBounds;
        float sumErrorUnweighted, sumErrorWeighted, sumSignedRes;
        void clearCountings()
        {
            goodPtsEdges = badPtsEdges = 0;
        }
        void clearErrors()
        {
            sumErrorUnweighted = sumErrorWeighted = sumSignedRes =  0.0f;
        }
        void clearAll()
        {
            clearCountings();
            clearErrors();
        }
    };
private:
    void calculateWarpUpdate(lsd_slam::LGS6 &ls, int goodPoints);

    OptimizerSettings mSettings;
    //This is for the edges
    float* buf_warped_residual;
    float* buf_warped_dx;
    float* buf_warped_dy;
    float* buf_warped_x;
    float* buf_warped_y;
    float* buf_warped_z;
    float* buf_weight_p;
private:
    float calcErrorAndBuffers(const std::shared_ptr<ImgPyramidRGBD> & refFrame, const std::shared_ptr<ImgPyramidRGBD> &currFrame, const Eigen::Matrix3f& R, const Eigen::Vector3f& T, ResidualInfo& resInfo,
                              const uint lvl, bool FILL_BUFFERS=true);
    inline float getWeightOfEvoR(const float r) const
    {
        //residual is always positive, thus no std::abs needed!
        return (r <= mSettings.huber_edge ? 1 : mSettings.huber_edge/r);
    }


public:
    float bundleAdjustment(const std::vector<std::shared_ptr<ImgPyramidRGBD> > &frame, Eigen::Matrix3f &R, Eigen::Vector3f &T, int lvl,
                           const std::shared_ptr<ImgPyramidRGBD>& currFrame);
    Optimizer(const OptimizerSettings &settings);
    ~Optimizer();
    float trackFrames(const std::shared_ptr<ImgPyramidRGBD> &refFrame,const std::shared_ptr<ImgPyramidRGBD> &currFrame,
                                 Eigen::Matrix3f &R, Eigen::Vector3f &T, int lvl, ResidualInfo& resInfo);
    //Bilinear interpolation
    inline Eigen::Vector3f getInterpolatedElement43(const Eigen::Vector4f* const mat, const float x, const float y, const int width) const
    {
        const int ix = static_cast<int>(x);
        const int iy = static_cast<int>(y);
        const float dx = x - ix;
        const float dy = y - iy;
        const float dxdy = dx*dy;
        const Eigen::Vector4f* bp = mat +ix+iy*width;
        return dxdy * *(const Eigen::Vector3f*)(bp+1+width)
                + (dy-dxdy) * *(const Eigen::Vector3f*)(bp+width)
                + (dx-dxdy) * *(const Eigen::Vector3f*)(bp+1)
                + (1-dx-dy+dxdy) * *(const Eigen::Vector3f*)(bp);
    }
};


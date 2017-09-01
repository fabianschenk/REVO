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
#include "../utils/Logging.h"
#include <Eigen/Eigen>
class PointCloudBuffered
{
public:
    PointCloudBuffered(const Eigen::MatrixXf& pcl, bool hasColor = false)
             : nPoints(pcl.cols()),
               mHasColor(hasColor),
               mSize(hasColor ? 8 : 4)
    {
        glGenBuffers(1, &vbo);
        I3D_LOG(i3d::info) << " Binding: " << (nPoints-1)*mSize*sizeof(float);
        glBindBuffer(GL_ARRAY_BUFFER,vbo);
        glBufferData(GL_ARRAY_BUFFER, (nPoints-1)*mSize*sizeof(float), (float*)pcl.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        I3D_LOG(i3d::info) << "After Binding: " << (nPoints-1)*mSize*sizeof(float);
    }
    ~PointCloudBuffered()
    {
        glDeleteBuffers(1, &vbo);
    }
    void drawPoints(const Eigen::Matrix4f& T_w_c)
    {
        GLfloat f = 2.0f;
        glPointSize(f);
        mHasColor = true;
        //the idea is to use the pose
//        glPushMatrix();
//        glMultMatrixf(Twc.ptr<GLfloat>(0));

        Eigen::Matrix4f Twc = T_w_c.cast<float>();//pKF->GetPoseInverse().t();
        //I3D_LOG(i3d::info) << "Twc: " << Twc;
        //Twc = Twc.inverse().eval();
        glPushMatrix();
        //glMultMatrixf(Twc<GLfloat>(0));
        glMultMatrixf((float*)Twc.data());

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        //XYZ1
        glVertexPointer(3, GL_FLOAT, sizeof(float)*8, 0);
        if (mHasColor) glEnableClientState(GL_COLOR_ARRAY);
        //XYZ1,RGB1
        if (mHasColor) glColorPointer(3, GL_FLOAT, sizeof(float)*8, (void *)(sizeof(float) * 4));//);
        glDrawArrays(GL_POINTS,0, nPoints-1);
        if (mHasColor) glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glPopMatrix();
    }
    const int nPoints;
private:

    GLuint vbo;
    bool mHasColor;
    const int mSize; //4 or 8 depending on XYZ1 or XYZ1RGB1

};

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <src/SemanticClassMap/SemanticClassMap.h>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    std::set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());


    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
		glVertex3f(pos.at<float>(0), pos.at<float>(1) , pos.at<float>(2) );

    }

    glEnd();

//************************road
    glPointSize(mPointSize*2);
    glBegin(GL_POINTS);

    float r,g,b;
    r = sematic_color_r[SemanticClass::nRoad]>0?(float)sematic_color_r[SemanticClass::nRoad]/255:0;
    g = sematic_color_g[SemanticClass::nRoad]>0?(float)sematic_color_g[SemanticClass::nRoad]/255:0;
    b = sematic_color_b[SemanticClass::nRoad]>0?(float)sematic_color_b[SemanticClass::nRoad]/255:0;
    glColor3f(r,g,b);
    //std::cout<<"viewer  "<<vpRoadMap.size()<<std::endl;
    ObjectMap& vpRoadMap = mpMap->mSemanticMap->roadMap;
    vpRoadMap.lock();
    //std::cout<<"road  "<<vpRoadMap.size()<<std::endl;
    for(set<MapPoint*>::iterator sit=vpRoadMap.begin(), send=vpRoadMap.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0), pos.at<float>(1) , pos.at<float>(2) );
    }
    vpRoadMap.unlock();
    glEnd();
//**********************car
    glPointSize(mPointSize*2);
    glBegin(GL_POINTS);
    r = sematic_color_r[SemanticClass::nCar]>0?(float)sematic_color_r[SemanticClass::nCar]/255:0;
    g = sematic_color_g[SemanticClass::nCar]>0?(float)sematic_color_g[SemanticClass::nCar]/255:0;
    b = sematic_color_b[SemanticClass::nCar]>0?(float)sematic_color_b[SemanticClass::nCar]/255:0;
    glColor3f(r,g,b);
    ObjectMap& vpCarMap = mpMap->mSemanticMap->carMap;
    vpCarMap.lock();
    //std::cout<<"car  "<<vpCarMap.size()<<std::endl;
    for(set<MapPoint*>::iterator sit=vpCarMap.begin(), send=vpCarMap.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();

        glVertex3f(pos.at<float>(0), pos.at<float>(1) , pos.at<float>(2) );
    }
    vpCarMap.unlock();
    glEnd();
//****************************person
    glPointSize(mPointSize*2);
    glBegin(GL_POINTS);
    r = sematic_color_r[SemanticClass::nPerson]>0?(float)sematic_color_r[SemanticClass::nPerson]/255:0;
    g = sematic_color_g[SemanticClass::nPerson]>0?(float)sematic_color_g[SemanticClass::nPerson]/255:0;
    b = sematic_color_b[SemanticClass::nPerson]>0?(float)sematic_color_b[SemanticClass::nPerson]/255:0;
    glColor3f(r,g,b);
    ObjectMap& vpPersonMap = mpMap->mSemanticMap->personMap;
    vpPersonMap.lock();
    //std::cout<<"person  "<<vpPersonMap.size()<<std::endl;
    for(set<MapPoint*>::iterator sit=vpPersonMap.begin(), send=vpPersonMap.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();

        glVertex3f(pos.at<float>(0), pos.at<float>(1) , pos.at<float>(2) );
    }
    vpPersonMap.unlock();
    glEnd();
//****************************obstacle
    glPointSize(mPointSize*2);
    glBegin(GL_POINTS);
    r = sematic_color_r[SemanticClass::nObstacle]>0?(float)sematic_color_r[SemanticClass::nObstacle]/255:0;
    g = sematic_color_g[SemanticClass::nObstacle]>0?(float)sematic_color_g[SemanticClass::nObstacle]/255:0;
    b = sematic_color_b[SemanticClass::nObstacle]>0?(float)sematic_color_b[SemanticClass::nObstacle]/255:0;
    glColor3f(r,g,b);
    ObjectMap& vpObstacleMap = mpMap->mSemanticMap->obstacleMap;
    vpObstacleMap.lock();
    //std::cout<<"obstacle  "<<vpObstacleMap.size()<<std::endl;
    for(set<MapPoint*>::iterator sit=vpObstacleMap.begin(), send=vpObstacleMap.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();

        glVertex3f(pos.at<float>(0), pos.at<float>(1) , pos.at<float>(2) );
    }
    vpObstacleMap.unlock();
    glEnd();
//****************************parkinglots
    glPointSize(mPointSize*2);
    glBegin(GL_POINTS);
    r = sematic_color_r[SemanticClass::nParkinglots]>0?(float)sematic_color_r[SemanticClass::nParkinglots]/255:0;
    g = sematic_color_g[SemanticClass::nParkinglots]>0?(float)sematic_color_g[SemanticClass::nParkinglots]/255:0;
    b = sematic_color_b[SemanticClass::nParkinglots]>0?(float)sematic_color_b[SemanticClass::nParkinglots]/255:0;
    glColor3f(r,g,b);
    ObjectMap& vpParkinglotMap = mpMap->mSemanticMap->parkinglotMap;
    vpParkinglotMap.lock();
    //std::cout<<"parkinglot  "<<vpParkinglotMap.size()<<std::endl;
    for(set<MapPoint*>::iterator sit=vpParkinglotMap.begin(), send=vpParkinglotMap.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();

        glVertex3f(pos.at<float>(0), pos.at<float>(1) , pos.at<float>(2) );
    }
    vpParkinglotMap.unlock();
    //std::cout<<"viewer OK"<<std::endl;
    glEnd();

}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;


    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            float scale = 5;
            glLineWidth(mKeyFrameLineWidth);
            glColor3f(1.0f,0.0f,0.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w*3,0,0);

            glColor3f(0.0f,1.0f,0.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(0,h*3,0);

            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(0,0,z*3);

            glColor3f(1.0f,0.0f,1.0f);
            glVertex3f(0,0,0);
            glVertex3f(w/scale,h/scale,z/scale);
            glVertex3f(0,0,0);
            glVertex3f(w/scale,-h/scale,z/scale);
            glVertex3f(0,0,0);
            glVertex3f(-w/scale,-h/scale,z/scale);
            glVertex3f(0,0,0);
            glVertex3f(-w/scale,h/scale,z/scale);

            glVertex3f(w/scale,h/scale,z/scale);
            glVertex3f(w/scale,-h/scale,z/scale);

            glVertex3f(-w/scale,h/scale,z/scale);
            glVertex3f(-w/scale,-h/scale,z/scale);

            glVertex3f(-w/scale,h/scale,z/scale);
            glVertex3f(w/scale,h/scale,z/scale);

            glVertex3f(-w/scale,-h/scale,z/scale);
            glVertex3f(w/scale,-h/scale,z/scale);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM

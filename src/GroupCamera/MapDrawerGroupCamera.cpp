#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <src/SemanticClassMap/SemanticClassMap.h>
#include "debug_utils/debug_utils.h"

namespace ORB_SLAM2
{
    void MapDrawer::DrawKeyFramesGroupCamera(const bool bDrawKF, const bool bDrawGraph)
    {
        const float &w = mKeyFrameSize;
        const float h = w * 0.75;
        const float z = w * 0.6;


        const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

        if (bDrawKF)
        {
            for (size_t i = 0; i < vpKFs.size(); i++)
            {
                for (int c = 0; c < vpKFs[i]->Ncameras; c++)
                {
                    KeyFrame *pKF = vpKFs[i];
                    //cv::Mat Twc = pKF->GetPoseInverse().t();
                    cv::Mat Twc = pKF->mvTcwSubcamera[c].inv().t();
                    glPushMatrix();

                    glMultMatrixf(Twc.ptr<GLfloat>(0));

                    float scale = 1;
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                    glVertex3f(0, 0, 0);
                    glVertex3f(w * 3, 0, 0);

                    glColor3f(0.0f, 1.0f, 0.0f);
                    glBegin(GL_LINES);
                    glVertex3f(0, 0, 0);
                    glVertex3f(0, h * 3, 0);

                    glColor3f(0.0f, 0.0f, 1.0f);
                    glBegin(GL_LINES);
                    glVertex3f(0, 0, 0);
                    glVertex3f(0, 0, z * 3);
                    glBegin(GL_LINES);
                    glColor3f(1.0f, 0.0f, 1.0f);
                    glVertex3f(0, 0, 0);
                    glVertex3f(w / scale, h / scale, z / scale);
                    glVertex3f(0, 0, 0);
                    glVertex3f(w / scale, -h / scale, z / scale);
                    glVertex3f(0, 0, 0);
                    glVertex3f(-w / scale, -h / scale, z / scale);
                    glVertex3f(0, 0, 0);
                    glVertex3f(-w / scale, h / scale, z / scale);

                    glVertex3f(w / scale, h / scale, z / scale);
                    glVertex3f(w / scale, -h / scale, z / scale);

                    glVertex3f(-w / scale, h / scale, z / scale);
                    glVertex3f(-w / scale, -h / scale, z / scale);

                    glVertex3f(-w / scale, h / scale, z / scale);
                    glVertex3f(w / scale, h / scale, z / scale);

                    glVertex3f(-w / scale, -h / scale, z / scale);
                    glVertex3f(w / scale, -h / scale, z / scale);
                    glEnd();

                    glPopMatrix();
                }

            }
        }

        if (bDrawGraph)
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            for (size_t i = 0; i < vpKFs.size(); i++)
            {
                // Covisibility Graph
                const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if (!vCovKFs.empty())
                {
                    for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                         vit != vend; vit++)
                    {
                        if ((*vit)->mnId < vpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame *pParent = vpKFs[i]->GetParent();
                if (pParent)
                {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++)
                {
                    if ((*sit)->mnId < vpKFs[i]->mnId)
                        continue;
                    cv::Mat Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
                }
            }

            glEnd();
        }
    }
}
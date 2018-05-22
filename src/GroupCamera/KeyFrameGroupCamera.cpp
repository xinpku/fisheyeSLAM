#include "KeyFrame.h"
#include "ORBmatcher.h"
#include "debug_utils/debug_utils.h"
namespace ORB_SLAM2
{


    //Functions related to groupCamera
    void KeyFrame::UpdateMultiCameraPose()
    {
        for (int i = 0; i < mvTcg.size(); i++)
        {
            mvTcwSubcamera[i] = mvTcg[i] * Tcw;
            mvOwSubcamera[i] = -(mvTcwSubcamera[i].rowRange(0, 3).colRange(0, 3)).t()*(mvTcwSubcamera[i].rowRange(0, 3).col(3));
        }
    }



    std::vector<cv::KeyPoint> KeyFrame::getKeypointUnSubCamera(int c) const
    {
        int start_pos = kp_start_pos[c];
        int end_pose = (c==Ncameras-1?mvKeysUn.size():kp_start_pos[c+1]);
        std::vector<cv::KeyPoint> keypoints;
        keypoints.reserve(end_pose - start_pos);
        for(int i = start_pos;i<end_pose;i++)
        {
            keypoints.push_back(mvKeysUn[i]);
        }
        return keypoints;
    }
    cv::Mat KeyFrame::getTcwSubCamera(int i)
    {
        return mvTcwSubcamera[i].clone();
    }
    cv::Mat KeyFrame::GetCameraCenterSubCamera(int i)
    {
        return mvOwSubcamera[i].clone();
    }
    cv::Mat KeyFrame::GetRotationSubCamera(int i)
    {
        unique_lock<mutex> lock(mMutexPose);
        return mvTcwSubcamera[i].rowRange(0, 3).colRange(0, 3).clone();
    }
    cv::Mat KeyFrame::GetTranslationSubCamera(int i)
    {
        unique_lock<mutex> lock(mMutexPose);
        return mvTcwSubcamera[i].rowRange(0, 3).col(3).clone();
    }


    float KeyFrame::ComputeSceneMedianDepthGroupCamera(const int q)
    {
        vector<MapPoint*> vpMapPoints;
        std::vector<cv::Mat> Tcw_(Ncameras);
        {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = mvpMapPoints;
            for(int c = 0;c<Ncameras;c++)
            {
                Tcw_[c] = getTcwSubCamera(c);
            }
        }

        vector<float> vDepths;
        vDepths.reserve(N);
        std::vector<cv::Mat> Rcw2(Ncameras);
        std::vector<float> zcw(Ncameras);
        for(int c = 0;c<Ncameras;c++)
        {
            Rcw2[c] = Tcw_[c].row(2).colRange(0,3);
            Rcw2[c] = Rcw2[c].t();
            zcw[c] = Tcw_[c].at<float>(2,3);
        }


        for(int i=0; i<N; i++)
        {
            if(mvpMapPoints[i])
            {
                MapPoint* pMP = mvpMapPoints[i];
                cv::Mat x3Dw = pMP->GetWorldPos();

                int cameraID = mvCamera_Id_KeysUn[i];

                float z = Rcw2[cameraID].dot(x3Dw)+zcw[cameraID];
                vDepths.push_back(z);
            }
        }

        sort(vDepths.begin(),vDepths.end());

        return vDepths[(vDepths.size()-1)/q];
    }
    void KeyFrame::UpdateConnectionsGroupCamera()
    {
        map<KeyFrame*,int> KFcounter;

        vector<MapPoint*> vpMP;

        {
            unique_lock<mutex> lockMPs(mMutexFeatures);
            vpMP = mvpMapPoints;
        }

        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;

            if(!pMP)
                continue;

            if(pMP->isBad()||pMP->dealed_by_current_frame)
                continue;

            pMP->dealed_by_current_frame = true;

            map<KeyFrame*,size_t> observations = pMP->GetObservations();

            for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
            {
                if(mit->first->mnId==mnId)
                    continue;
                KFcounter[mit->first]++;
            }
        }


        for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
        {
            MapPoint *pMP = *vit;

            if (!pMP)
                continue;
            pMP->dealed_by_current_frame = false;
        }
        // This should not happen
        if(KFcounter.empty())
            return;

        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        int nmax=0;
        KeyFrame* pKFmax=NULL;
        int th = 15;

        vector<pair<int,KeyFrame*> > vPairs;
        vPairs.reserve(KFcounter.size());
        for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
        {
            if(mit->second>nmax)
            {
                nmax=mit->second;
                pKFmax=mit->first;
            }
            if(mit->second>=th)
            {
                vPairs.push_back(make_pair(mit->second,mit->first));
                (mit->first)->AddConnection(this,mit->second);
            }
        }

        if(vPairs.empty())
        {
            vPairs.push_back(make_pair(nmax,pKFmax));
            pKFmax->AddConnection(this,nmax);
        }

        sort(vPairs.begin(),vPairs.end());
        list<KeyFrame*> lKFs;
        list<int> lWs;
        for(size_t i=0; i<vPairs.size();i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            unique_lock<mutex> lockCon(mMutexConnections);

            // mspConnectedKeyFrames = spConnectedKeyFrames;
            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

            if(mbFirstConnection && mnId!=0)
            {
                mpParent = mvpOrderedConnectedKeyFrames.front();
                mpParent->AddChild(this);
                mbFirstConnection = false;
            }

        }
    }

    vector<size_t> KeyFrame::GetFeaturesInAreaSubCamera(const float &x, const float &y, const float &r,int cameraID) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
        if(nMinCellX>=mnGridCols)
            return vIndices;

        const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
        if(nMaxCellX<0)
            return vIndices;

        const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
        if(nMinCellY>=mnGridRows)
            return vIndices;

        const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
        if(nMaxCellY<0)
            return vIndices;

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    if(mvCamera_Id_KeysUn[vCell[j]]!=cameraID)
                        continue;
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }


    void KeyFrame::ComputeStereoGroupCamera(
            std::vector<std::pair<int,int>> relatedCamera,
            std::vector<cv::Mat> F_relatedCameras)
    {

        printON;
        ORBmatcher matcher(0.6,false);

        const float &fx1 = fx;
        const float &fy1 = fy;
        const float &cx1 = cx;
        const float &cy1 = cy;
        const float &invfx1 = invfx;
        const float &invfy1 = invfy;

        const float ratioFactor = 1.5f*mfScaleFactor;

        int nnew=0;
        int Ncameras = Ncameras;

        const float &fx2 = fx;
        const float &fy2 = fy;
        const float &cx2 = cx;
        const float &cy2 = cy;
        const float &invfx2 = invfx;
        const float &invfy2 = invfy;

        for(int i = 0;i<relatedCamera.size();i++)
        {
            int view1 = relatedCamera[i].first;
            int view2 = relatedCamera[i].second;
            // Compute Fundamental Matrix
            cv::Mat F12SubCamera = F_relatedCameras[i];

            // Search matches that fullfil epipolar constraint
            vector<pair<size_t, size_t> > vMatchedIndices;

            //TODO implement the matching function, check the reconstruction logical. Add the stereo matching to keyframe creation and initilization

            matcher.SearchForTriangulationGroupCamera(this, this, view1, view2, F12SubCamera, vMatchedIndices,
                                                      false);

            cv::Mat Rcw1 = GetRotationSubCamera(view1);
            cv::Mat Rwc1 = Rcw1.t();
            cv::Mat tcw1 = GetTranslationSubCamera(view1);
            cv::Mat Tcw1(3, 4, CV_32F);
            Rcw1.copyTo(Tcw1.colRange(0, 3));
            tcw1.copyTo(Tcw1.col(3));
            cv::Mat Ow1 = GetCameraCenterSubCamera(view1);

            cv::Mat Rcw2 = GetRotationSubCamera(view2);
            cv::Mat Rwc2 = Rcw2.t();
            cv::Mat tcw2 = GetTranslationSubCamera(view2);
            cv::Mat Tcw2(3, 4, CV_32F);
            Rcw2.copyTo(Tcw2.colRange(0, 3));
            tcw2.copyTo(Tcw2.col(3));
            cv::Mat Ow2 = GetCameraCenterSubCamera(view2);
            cv::Mat vBaseline = Ow2-Ow1;
            const float baseline = cv::norm(vBaseline);


            // Triangulate each match
            const int nmatches = vMatchedIndices.size();
            print_value(nmatches);
            int new_local = 0;

            std::vector<int> crash_reason;
            for (int ikp = 0; ikp < nmatches; ikp++)
            {
                crash_reason.push_back(0);
                const int &idx1 = vMatchedIndices[ikp].first;
                const int &idx2 = vMatchedIndices[ikp].second;

                const cv::KeyPoint &kp1 = mvKeysUn[idx1];
                const float kp1_ur = mvuRight[idx1];
                bool bStereo1 = kp1_ur >= 0;

                const cv::KeyPoint &kp2 = mvKeysUn[idx2];
                const float kp2_ur = mvuRight[idx2];
                bool bStereo2 = kp2_ur >= 0;

                // Check parallax between rays
                cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
                cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2) * invfx2, (kp2.pt.y - cy2) * invfy2, 1.0);

                cv::Mat ray1 = Rwc1 * xn1;
                cv::Mat ray2 = Rwc2 * xn2;

                float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));


                /*if(cosParallaxRays>0.9998&&(abs(kp1.pt.x-kp2.pt.x)>2||abs(kp1.pt.y-kp2.pt.y)>2))
                    cosParallaxRays = 0.9995;*/
                /*print_string("'''''''''''''''''")
                print_vect_cv(cv::Mat(kp1.pt),c==0);
                print_vect_cv(cv::Mat(kp2.pt),c==0);
                print_vect_cv(ray1,c==0);
                print_vect_cv(ray2,c==0);
                print_value(cosParallaxRays,c==0);*/

                float cosParallaxStereo = cosParallaxRays + 1;
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;

                if (bStereo1)
                    cosParallaxStereo1 = cos(2 * atan2(mb / 2, mvDepth[idx1]));
                else if (bStereo2)
                    cosParallaxStereo2 = cos(2 * atan2(mb / 2, mvDepth[idx2]));

                cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

                /*print_string(",,,,,,,,,,,,,,,,,,,,,,,,,,,,,,")
                print_value(ikp,c==0)
                print_value(kp1.pt,c==0)
                print_value(kp2.pt,c==0)
                print_value(cosParallaxRays,c==0)
                print_value(cosParallaxStereo,c==0)
                print_value(bStereo1,c==0)
                print_value(bStereo2,c==0)
                print_value(cosParallaxStereo1,c==0)
                print_value(cosParallaxStereo2,c==0)*/
                crash_reason.back() = 1;

                cv::Mat x3D;
                if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
                    (bStereo1 || bStereo2 || cosParallaxRays < 0.9998))
                {
                    // Linear Triangulation Method
                    cv::Mat A(4, 4, CV_32F);
                    A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
                    A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
                    A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
                    A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

                    cv::Mat w, u, vt;
                    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

                    x3D = vt.row(3).t();
                    //print_vect_cv(x3D,c==0)
                    if (x3D.at<float>(3) == 0)
                        continue;

                    // Euclidean coordinates
                    x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);

                } else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2)
                {
                    x3D = UnprojectStereo(idx1);
                } else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1)
                {
                    x3D = UnprojectStereo(idx2);
                } else
                    continue; //No stereo and very low parallax

                cv::Mat x3Dt = x3D.t();

                crash_reason.back() = 2;
                //Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
                if (z1 <= 0)
                    continue;

                crash_reason.back() = 3;

                float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
                if (z2 <= 0)
                    continue;

                //Check reprojection error in first keyframe
                const float &sigmaSquare1 = mvLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
                const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
                const float invz1 = 1.0 / z1;

                crash_reason.back() = 4;
                if (!bStereo1)
                {
                    float u1 = fx1 * x1 * invz1 + cx1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    //print_value(errX1*errX1+errY1*errY1,c==0)
                    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                        continue;
                } else
                {
                    float u1 = fx1 * x1 * invz1 + cx1;
                    float u1_r = u1 - mbf * invz1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    float errX1_r = u1_r - kp1_ur;
                    //print_value(errX1*errX1+errY1*errY1,c==0)
                    if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) > 7.8 * sigmaSquare1)
                        continue;
                }

                crash_reason.back() = 5;

                //Check reprojection error in second keyframe
                const float sigmaSquare2 = mvLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
                const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
                const float invz2 = 1.0 / z2;
                if (!bStereo2)
                {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    //print_value(errX2*errX2+errY2*errY2,c==0)
                    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                        continue;
                } else
                {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float u2_r = u2 - mbf * invz2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    float errX2_r = u2_r - kp2_ur;
                    //print_value(errX2*errX2+errY2*errY2,c==0)
                    if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) > 7.8 * sigmaSquare2)
                        continue;
                }

                //Check scale consistency
                cv::Mat normal1 = x3D - Ow1;
                float dist1 = cv::norm(normal1);

                cv::Mat normal2 = x3D - Ow2;
                float dist2 = cv::norm(normal2);

                crash_reason.back() = 6;
                //print_value(dist1,c==0)
                //print_value(dist2,c==0)
                if (dist1 == 0 || dist2 == 0)
                    continue;

                const float ratioDist = dist2 / dist1;
                const float ratioOctave =
                        mvScaleFactors[kp1.octave] / mvScaleFactors[kp2.octave];

                crash_reason.back() = 7;
                /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                    continue;*/
                //print_value(ratioDist,c==0)
                //print_value(ratioOctave,c==0)
                if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
                    continue;

                // Triangulation is succesfull
                MapPoint *pMP = new MapPoint(x3D, this, mpMap);
                pMP->created_by_kf1 = view1;
                print_vect_cv(x3D, new_local < 10);
                new_local++;
                //wx-debug we simply set the semantic class of the map point the same as current keyframe. But it should be a fusion processing according the probability of every class type.
                KeyFrame *semantic_source;
                int semantic_idx;
                if (mvSemanticProbability[idx1] > mvSemanticProbability[idx2])
                {
                    semantic_source = this;
                    semantic_idx = idx1;
                } else
                {
                    semantic_source = this;
                    semantic_idx = idx2;
                }

                pMP->mSemanticClass = semantic_source->mvSemanticClass[semantic_idx];
                pMP->mSemanticProb = semantic_source->mvSemanticProbability[semantic_idx];

                pMP->AddObservation(this, idx1);
                pMP->AddObservation(this, idx2);

                this->AddMapPoint(pMP, idx1);
                this->AddMapPoint(pMP, idx2);

                pMP->ComputeDistinctiveDescriptors();

                pMP->UpdateNormalAndDepth();

                mpMap->AddMapPoint(pMP);
                //mlpRecentAddedMapPoints.push_back(pMP);
                crash_reason.back() = 0;
                nnew++;
            }
        }
        print_value(nnew);
    }






}
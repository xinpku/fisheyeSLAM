#include "ORBmatcher.h"


namespace ORB_SLAM2
{
    //********************************************
    //Functions related to groupCamera

    int ORBmatcher::SearchByProjectionGroupCamera(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3)
    {
        int nmatches=0;

        const bool bFactor = th!=1.0;

        for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++)
        {
            MapPoint *pMP = vpMapPoints[iMP];
            if (!pMP->mbTrackInView)
                continue;

            if (pMP->isBad())
                continue;

            for(int c = 0;c<F.Ncameras;c++)
            {
                const int &nPredictedLevel = pMP->mvnTrackScaleLevel[c];

                // The size of the window will depend on the viewing direction
                float r = RadiusByViewingCos(pMP->mvTrackViewCos[c]);

                if (bFactor)
                    r *= th;

                const vector<size_t> vIndices =
                        F.GetFeaturesInArea(pMP->mvTrackProjX[c], pMP->mvTrackProjY[c], r * F.mvScaleFactors[nPredictedLevel],
                                            nPredictedLevel - 1, nPredictedLevel);

                if (vIndices.empty())
                    continue;

                const cv::Mat MPdescriptor = pMP->GetDescriptor();

                int bestDist = 256;
                int bestLevel = -1;
                int bestDist2 = 256;
                int bestLevel2 = -1;
                int bestIdx = -1;
                // Get best and second matches with near keypoints
                for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++)
                {
                    const size_t idx = *vit;

                    if (F.mvpMapPoints[idx])
                        if (F.mvpMapPoints[idx]->Observations() > 0)
                            continue;

                    if (F.mvuRight[idx] > 0)
                    {
                        const float er = fabs(pMP->mTrackProjXR - F.mvuRight[idx]);
                        if (er > r * F.mvScaleFactors[nPredictedLevel])
                            continue;
                    }

                    const cv::Mat &d = F.mDescriptors.row(idx);

                    const int dist = DescriptorDistance(MPdescriptor, d);

                    if (dist < bestDist)
                    {
                        bestDist2 = bestDist;
                        bestDist = dist;
                        bestLevel2 = bestLevel;
                        bestLevel = F.mvKeysUn[idx].octave;
                        bestIdx = idx;
                    } else if (dist < bestDist2)
                    {
                        bestLevel2 = F.mvKeysUn[idx].octave;
                        bestDist2 = dist;
                    }
                }

                // Apply ratio to second match (only if best and second are in the same scale level)
                if (bestDist <= TH_HIGH)
                {
                    if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                        continue;

                    F.mvpMapPoints[bestIdx] = pMP;
                    if(!pMP->dealed_by_current_frame)
                    {
                        nmatches++;
                        pMP->dealed_by_current_frame = true;
                    }
                }

            }

        }

        for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++)
        {
            if (vpMapPoints[iMP])
                vpMapPoints[iMP]->dealed_by_current_frame = false;
        }

        return nmatches;
    }





    int ORBmatcher::SearchByProjectionGroupCamera(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
    {
        int nmatches = 0;

        // Rotation Histogram (to check rotation consistency)
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i<HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;


        const int& Ncameras = CurrentFrame.Ncameras;


        const bool bForward =  !bMono;//暂时没有用处，预留给之后可能的立体视觉实现
        const bool bBackward = !bMono;//暂时没有用处，预留给之后可能的立体视觉实现

        for (int i = 0; i<LastFrame.N; i++)
        {
            MapPoint* pMP = LastFrame.mvpMapPoints[i];

            if (pMP)
            {
                if (!LastFrame.mvbOutlier[i] && !pMP->dealed_by_current_frame)//如果已经被处理过了就不再投影
                {
                    pMP->dealed_by_current_frame = true;//将已经处理过的标志置位
                    bool counted_once = false;//To deal with the situation that a map point is observed by multiple view point in a frame
                    for (int c = 0; c < Ncameras; c++)//分别按照各个子相机投影，然后检索可能的对应，并将对应关系加入到index向量，之后从index向量中搜索最好的匹配
                    {
                        // Project
                        const cv::Mat& Rcw = CurrentFrame.mvRcwSubcamera[c];
                        const cv::Mat& tcw = CurrentFrame.mvtcwSubcamera[c];
                        cv::Mat x3Dw = pMP->GetWorldPos();
                        cv::Mat x3Dc = Rcw*x3Dw + tcw;

                        const float xc = x3Dc.at<float>(0);
                        const float yc = x3Dc.at<float>(1);
                        const float invzc = 1.0 / x3Dc.at<float>(2);

                        if (invzc < 0)
                            continue;

                        float u = CurrentFrame.fx*xc*invzc + CurrentFrame.cx;
                        float v = CurrentFrame.fy*yc*invzc + CurrentFrame.cy;

                        if (u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                            continue;
                        if (v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                            continue;

                        int nLastOctave = LastFrame.mvKeys[i].octave;

                        // Search in a window. Size depends on scale
                        float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];
                        vector<size_t> vIndices2;
                        if (bForward)
                            vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave);
                        else if (bBackward)
                            vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, 0, nLastOctave);
                        else
                            vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave - 1, nLastOctave + 1);

                        if (vIndices2.empty())
                            continue;

                        const cv::Mat dMP = pMP->GetDescriptor();

                        int bestDist = 256;
                        int bestIdx2 = -1;

                        for (vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end(); vit != vend; vit++)
                        {
                            const size_t i2 = *vit;
                            if (CurrentFrame.mvpMapPoints[i2])
                                if (CurrentFrame.mvpMapPoints[i2]->Observations() > 0)
                                    continue;

                            if (CurrentFrame.mvuRight[i2] > 0)
                            {
                                const float ur = u - CurrentFrame.mbf*invzc;
                                const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                                if (er > radius)
                                    continue;
                            }

                            const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                            const int dist = DescriptorDistance(dMP, d);

                            if (dist < bestDist)
                            {
                                bestDist = dist;
                                bestIdx2 = i2;
                            }
                        }

                        if (bestDist <= TH_HIGH)
                        {
                            CurrentFrame.mvpMapPoints[bestIdx2] = pMP;

                            if(!counted_once)//A map point only counted once
                            {
                                nmatches++;
                                counted_once = true;
                            }

                            if (mbCheckOrientation)
                            {
                                float rot = LastFrame.mvKeysUn[i].angle - CurrentFrame.mvKeysUn[bestIdx2].angle;
                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = round(rot*factor);
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                assert(bin >= 0 && bin < HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdx2);
                            }
                        }
                    }
                }
            }
        }
        //清空所有处理记录
        for (int i = 0; i < LastFrame.N; i++)
        {
            MapPoint* pMP = LastFrame.mvpMapPoints[i];

            if (pMP)
                pMP->dealed_by_current_frame = false;
        }
        //Apply rotation consistency
        if (mbCheckOrientation)
        {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i<HISTO_LENGTH; i++)
            {
                if (i != ind1 && i != ind2 && i != ind3)
                {
                    for (size_t j = 0, jend = rotHist[i].size(); j<jend; j++)
                    {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]] = static_cast<MapPoint*>(NULL);
                        nmatches--;
                    }
                }
            }
        }

        return nmatches;
    }






    int ORBmatcher::SearchForTriangulationGroupCamera(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<std::vector<cv::Mat>> vF,
                                                      vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo)
    {
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
        const int Ncamera = pKF1->mvTcwSubcamera.size();

        float** ex_subcamera = (float**)new float[Ncamera*Ncamera];
        float** ey_subcamera = (float**)new float[Ncamera*Ncamera];

        for (int i = 0; i < Ncamera; i++)
        {
            for (int j = 0; j < Ncamera; j++)
            {
                //Compute epipole in second image
                cv::Mat Cw = pKF1->GetCameraCenterSubCamera(i);
                cv::Mat R2w = pKF2->GetRotationSubCamera(j);
                cv::Mat t2w = pKF2->GetTranslationSubCamera(j);
                cv::Mat C2 = R2w*Cw + t2w;
                const float invz = 1.0f / C2.at<float>(2);
                ex_subcamera[i][j] = pKF2->fx*C2.at<float>(0)*invz + pKF2->cx;
                ey_subcamera[i][j] = pKF2->fy*C2.at<float>(1)*invz + pKF2->cy;
            }
        }


        // Find matches between not tracked keypoints
        // Matching speed-up by ORB Vocabulary
        // Compare only ORB that share the same node

        int nmatches = 0;
        vector<bool> vbMatched2(pKF2->N, false);
        vector<int> vMatches12(pKF1->N, -1);

        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i<HISTO_LENGTH; i++)
            rotHist[i].reserve(500);

        const float factor = 1.0f / HISTO_LENGTH;

        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        while (f1it != f1end && f2it != f2end)
        {
            if (f1it->first == f2it->first)
            {
                for (size_t i1 = 0, iend1 = f1it->second.size(); i1<iend1; i1++)
                {
                    const size_t idx1 = f1it->second[i1];

                    MapPoint* pMP1 = pKF1->GetMapPoint(idx1);

                    // If there is already a MapPoint skip
                    if (pMP1)
                        continue;

                    const bool bStereo1 = pKF1->mvuRight[idx1] >= 0;

                    if (bOnlyStereo)
                        if (!bStereo1)
                            continue;

                    const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];
                    const int kp1_camera_id = pKF1->mvCamera_Id_KeysUn[idx1];
                    const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

                    int bestDist = TH_LOW;
                    int bestIdx2 = -1;

                    for (size_t i2 = 0, iend2 = f2it->second.size(); i2<iend2; i2++)
                    {
                        size_t idx2 = f2it->second[i2];

                        MapPoint* pMP2 = pKF2->GetMapPoint(idx2);

                        // If we have already matched or there is a MapPoint skip
                        if (vbMatched2[idx2] || pMP2)
                            continue;

                        const bool bStereo2 = pKF2->mvuRight[idx2] >= 0;

                        if (bOnlyStereo)
                            if (!bStereo2)
                                continue;

                        const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);

                        const int dist = DescriptorDistance(d1, d2);

                        if (dist>TH_LOW || dist>bestDist)
                            continue;

                        const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
                        const int kp2_camera_id = pKF2->mvCamera_Id_KeysUn[idx2];
                        if (!bStereo1 && !bStereo2)
                        {
                            const float distex = ex_subcamera[kp1_camera_id][kp2_camera_id] - kp2.pt.x;
                            const float distey = ey_subcamera[kp1_camera_id][kp2_camera_id] - kp2.pt.y;
                            if (distex*distex + distey*distey<100 * pKF2->mvScaleFactors[kp2.octave])
                                continue;
                        }

                        if (CheckDistEpipolarLine(kp1, kp2, vF[kp1_camera_id][kp2_camera_id], pKF2))
                        {
                            bestIdx2 = idx2;
                            bestDist = dist;
                        }
                    }

                    if (bestIdx2 >= 0)
                    {
                        const cv::KeyPoint &kp2 = pKF2->mvKeysUn[bestIdx2];
                        vMatches12[idx1] = bestIdx2;
                        nmatches++;

                        if (mbCheckOrientation)
                        {
                            float rot = kp1.angle - kp2.angle;
                            if (rot<0.0)
                                rot += 360.0f;
                            int bin = round(rot*factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                    }
                }

                f1it++;
                f2it++;
            }
            else if (f1it->first < f2it->first)
            {
                f1it = vFeatVec1.lower_bound(f2it->first);
            }
            else
            {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        if (mbCheckOrientation)
        {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i<HISTO_LENGTH; i++)
            {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j<jend; j++)
                {
                    vMatches12[rotHist[i][j]] = -1;
                    nmatches--;
                }
            }

        }

        vMatchedPairs.clear();
        vMatchedPairs.reserve(nmatches);

        for (size_t i = 0, iend = vMatches12.size(); i<iend; i++)
        {
            if (vMatches12[i]<0)
                continue;
            vMatchedPairs.push_back(make_pair(i, vMatches12[i]));
        }

        return nmatches;
    }



    int ORBmatcher::SearchForTriangulationGroupCamera(KeyFrame *pKF1, KeyFrame *pKF2, int subCameraIDKF1,int subCameraIDKF2, cv::Mat vF12subcamera,
                                                      vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo)
    {
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

        //Compute epipole in second image
        cv::Mat Cw = pKF1->GetCameraCenterSubCamera(subCameraIDKF1);
        cv::Mat R2w = pKF2->GetRotationSubCamera(subCameraIDKF2);
        cv::Mat t2w = pKF2->GetTranslationSubCamera(subCameraIDKF2);
        cv::Mat C2 = R2w*Cw+t2w;
        const float invz = 1.0f/C2.at<float>(2);
        const float ex =pKF2->fx*C2.at<float>(0)*invz+pKF2->cx;
        const float ey =pKF2->fy*C2.at<float>(1)*invz+pKF2->cy;

        // Find matches between not tracked keypoints
        // Matching speed-up by ORB Vocabulary
        // Compare only ORB that share the same node

        int nmatches=0;
        vector<bool> vbMatched2(pKF2->N,false);
        vector<int> vMatches12(pKF1->N,-1);

        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);

        const float factor = 1.0f/HISTO_LENGTH;

        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        int subCamBeginPosKF1 = pKF1->kp_start_pos[subCameraIDKF1];
        int subCamEndPosKF1 = (subCameraIDKF1==pKF1->Ncameras-1?pKF1->mvKeysUn.size():pKF1->kp_start_pos[subCameraIDKF1+1]);
        int subCamBeginPosKF2 = pKF2->kp_start_pos[subCameraIDKF2];
        int subCamEndPosKF2 = (subCameraIDKF2==pKF2->Ncameras-1?pKF2->mvKeysUn.size():pKF2->kp_start_pos[subCameraIDKF2+1]);


        while(f1it!=f1end && f2it!=f2end)
        {
            if(f1it->first == f2it->first)
            {
                for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
                {
                    const size_t idx1 = f1it->second[i1];

                    if(idx1<subCamBeginPosKF1||idx1>=subCamEndPosKF1)//Skip the points that don't belong to the sub camera
                        continue;

                    MapPoint* pMP1 = pKF1->GetMapPoint(idx1);

                    // If there is already a MapPoint skip
                    if(pMP1)
                        continue;

                    const bool bStereo1 = pKF1->mvuRight[idx1]>=0;

                    if(bOnlyStereo)
                        if(!bStereo1)
                            continue;

                    const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];

                    const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

                    int bestDist = TH_LOW;
                    int bestIdx2 = -1;

                    for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                    {
                        size_t idx2 = f2it->second[i2];

                        if(idx2<subCamBeginPosKF2||idx2>=subCamEndPosKF2)//Skip the points that don't belong to the sub camera
                            continue;

                        MapPoint* pMP2 = pKF2->GetMapPoint(idx2);

                        // If we have already matched or there is a MapPoint skip
                        if(vbMatched2[idx2] || pMP2)
                            continue;

                        const bool bStereo2 = pKF2->mvuRight[idx2]>=0;

                        if(bOnlyStereo)
                            if(!bStereo2)
                                continue;

                        const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);

                        const int dist = DescriptorDistance(d1,d2);

                        if(dist>TH_LOW || dist>bestDist)
                            continue;

                        const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

                        if(!bStereo1 && !bStereo2)
                        {
                            const float distex = ex-kp2.pt.x;
                            const float distey = ey-kp2.pt.y;
                            if(distex*distex+distey*distey<100*pKF2->mvScaleFactors[kp2.octave])
                                continue;
                        }

                        if(CheckDistEpipolarLine(kp1,kp2,vF12subcamera,pKF2))
                        {
                            bestIdx2 = idx2;
                            bestDist = dist;
                        }
                    }

                    if(bestIdx2>=0)
                    {
                        const cv::KeyPoint &kp2 = pKF2->mvKeysUn[bestIdx2];
                        vMatches12[idx1]=bestIdx2;
                        nmatches++;

                        if(mbCheckOrientation)
                        {
                            float rot = kp1.angle-kp2.angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                    }
                }

                f1it++;
                f2it++;
            }
            else if(f1it->first < f2it->first)
            {
                f1it = vFeatVec1.lower_bound(f2it->first);
            }
            else
            {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    vMatches12[rotHist[i][j]]=-1;
                    nmatches--;
                }
            }

        }

        vMatchedPairs.clear();
        vMatchedPairs.reserve(nmatches);

        for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
        {
            if(vMatches12[i]<0)
                continue;
            vMatchedPairs.push_back(make_pair(i,vMatches12[i]));
        }

        return nmatches;
    }


}
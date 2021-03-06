#include "ORBmatcher.h"
#include "debug_utils/debug_utils.h"

namespace ORB_SLAM2
{
    //********************************************
    //Functions related to groupCamera

    int ORBmatcher::SearchByProjectionGroupCamera(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th)
    {
        int nmatches = 0;
        printON

        const bool bFactor = th != 1.0;

        for (int c = 0; c < F.Ncameras; c++)
        {

            for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++)
            {
                MapPoint *pMP = vpMapPoints[iMP];
                if (!pMP->mvbTrackInView[c])
                    continue;

                if (pMP->isBad())
                    continue;



                const int &nPredictedLevel = pMP->mvnTrackScaleLevel[c];

                // The size of the window will depend on the viewing direction
                float r = RadiusByViewingCos(pMP->mvTrackViewCos[c]);

                if (bFactor)
                    r *= th;
                const vector<size_t> vIndices =
                        F.GetFeaturesInAreaSubCamera(pMP->mvTrackProjX[c], pMP->mvTrackProjY[c],
                                                     r * F.mvScaleFactors[nPredictedLevel],
                                                     nPredictedLevel - 1, nPredictedLevel, c);

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
                    }
                    else if (dist < bestDist2)
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

                    nmatches++;
                }
            }

        print_value(nmatches);
        }

        return nmatches;
    }





    int ORBmatcher::SearchByProjectionGroupCamera(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
    {
        int nmatches = 0;

        const int& Ncameras = CurrentFrame.Ncameras;
        printON;
        for (int c = 0; c < Ncameras; c++)//分别按照各个子相机投影，然后检索可能的对应，并将对应关系加入到index向量，之后从index向量中搜索最好的匹配
        {
            // Rotation Histogram (to check rotation consistency)
            vector<int> rotHist[HISTO_LENGTH];
            for (int i = 0; i<HISTO_LENGTH; i++)
                rotHist[i].reserve(500);
            const float factor = 1.0f / HISTO_LENGTH;


            // Project
            const cv::Mat& Rcw = CurrentFrame.mvRcwSubcamera[c];
            const cv::Mat& tcw = CurrentFrame.mvtcwSubcamera[c];

            const cv::Mat twc = -Rcw.t()*tcw;

            const cv::Mat Rlw = LastFrame.mvTcwSubcamera[c].rowRange(0,3).colRange(0,3);
            const cv::Mat tlw = LastFrame.mvTcwSubcamera[c].rowRange(0,3).col(3);

            const cv::Mat tlc = Rlw*twc+tlw;

            const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
            const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;


            for (int i = 0; i<LastFrame.N; i++)
            {
                MapPoint* pMP = LastFrame.mvpMapPoints[i];

                if(LastFrame.mvCamera_Id_KeysUn[i]!=c)//Because it's the last frame, we estimated that the correspondences should belong to the same sub camera.
                    continue;


                if (pMP)
                {
                    if (!LastFrame.mvbOutlier[i])//如果已经被处理过了就不再投影
                    {

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
                            vIndices2 = CurrentFrame.GetFeaturesInAreaSubCamera(u, v, radius, nLastOctave,-1,c);
                        else if (bBackward)
                            vIndices2 = CurrentFrame.GetFeaturesInAreaSubCamera(u, v, radius, 0, nLastOctave,c);
                        else
                            vIndices2 = CurrentFrame.GetFeaturesInAreaSubCamera(u, v, radius, nLastOctave - 1, nLastOctave + 1,c);

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


                            nmatches++;

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
            print_value(nmatches);
        }

        return nmatches;
    }




    int ORBmatcher::SearchByBoWGroupCamera(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
    {
        const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

        vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));

        const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

        int nmatches=0;

        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LENGTH;

        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
        DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
        DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
        DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

        while(KFit != KFend && Fit != Fend)
        {
            if(KFit->first == Fit->first)
            {
                const vector<unsigned int> vIndicesKF = KFit->second;
                const vector<unsigned int> vIndicesF = Fit->second;

                for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
                {
                    const unsigned int realIdxKF = vIndicesKF[iKF];

                    MapPoint* pMP = vpMapPointsKF[realIdxKF];

                    if(!pMP)
                        continue;

                    if(pMP->isBad())
                        continue;

                    const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);

                    int bestDist1=256;
                    int bestIdxF =-1 ;
                    int bestDist2=256;

                    for(size_t iF=0; iF<vIndicesF.size(); iF++)
                    {
                        const unsigned int realIdxF = vIndicesF[iF];

                        if(vpMapPointMatches[realIdxF])
                            continue;

                        const cv::Mat &dF = F.mDescriptors.row(realIdxF);

                        const int dist =  DescriptorDistance(dKF,dF);

                        if(dist<bestDist1)
                        {
                            bestDist2=bestDist1;
                            bestDist1=dist;
                            bestIdxF=realIdxF;
                        }
                        else if(dist<bestDist2)
                        {
                            bestDist2=dist;
                        }
                    }

                    if(bestDist1<=TH_LOW)
                    {
                        if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                        {
                            vpMapPointMatches[bestIdxF]=pMP;

                            const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];

                            if(mbCheckOrientation)
                            {
                                float rot = kp.angle-F.mvKeys[bestIdxF].angle;
                                if(rot<0.0)
                                    rot+=360.0f;
                                int bin = round(rot*factor);
                                if(bin==HISTO_LENGTH)
                                    bin=0;
                                assert(bin>=0 && bin<HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdxF);
                            }
                            nmatches++;
                        }
                    }

                }

                KFit++;
                Fit++;
            }
            else if(KFit->first < Fit->first)
            {
                KFit = vFeatVecKF.lower_bound(Fit->first);
            }
            else
            {
                Fit = F.mFeatVec.lower_bound(KFit->first);
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
                    vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
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
        printON
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
        print_value(nmatches);
        return nmatches;
    }


    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int ORBmatcher::FuseGroupCamera(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th)
    {
        int nFused=0;
        const int nMPs = vpMapPoints.size();


        for(int c = 0;c<pKF->Ncameras;c++)
        {
            cv::Mat Rcw = pKF->GetRotationSubCamera(c);
            cv::Mat tcw = pKF->GetTranslationSubCamera(c);

            const float &fx = pKF->fx;
            const float &fy = pKF->fy;
            const float &cx = pKF->cx;
            const float &cy = pKF->cy;
            const float &bf = pKF->mbf;

            cv::Mat Ow = pKF->GetCameraCenterSubCamera(c);


            for(int i=0; i<nMPs; i++)
            {
                MapPoint* pMP = vpMapPoints[i];

                if(!pMP)
                    continue;

                if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
                    continue;

                cv::Mat p3Dw = pMP->GetWorldPos();
                cv::Mat p3Dc = Rcw*p3Dw + tcw;

                // Depth must be positive
                if(p3Dc.at<float>(2)<0.0f)
                    continue;

                const float invz = 1/p3Dc.at<float>(2);
                const float x = p3Dc.at<float>(0)*invz;
                const float y = p3Dc.at<float>(1)*invz;

                const float u = fx*x+cx;
                const float v = fy*y+cy;

                // Point must be inside the image
                if(!pKF->IsInImage(u,v))
                    continue;

                const float ur = u-bf*invz;

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();
                cv::Mat PO = p3Dw-Ow;
                const float dist3D = cv::norm(PO);

                // Depth must be inside the scale pyramid of the image
                if(dist3D<minDistance || dist3D>maxDistance )
                    continue;

                // Viewing angle must be less than 60 deg
                cv::Mat Pn = pMP->GetNormal();

                if(PO.dot(Pn)<0.5*dist3D)
                    continue;

                int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

                // Search in a radius
                const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

                const vector<size_t> vIndices = pKF->GetFeaturesInAreaSubCamera(u,v,radius,c);

                if(vIndices.empty())
                    continue;

                // Match to the most similar keypoint in the radius

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx = -1;
                for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
                {
                    const size_t idx = *vit;

                    const cv::KeyPoint &kp = pKF->mvKeysUn[idx];

                    const int &kpLevel= kp.octave;

                    if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                        continue;

                    if(pKF->mvuRight[idx]>=0)
                    {
                        // Check reprojection error in stereo
                        const float &kpx = kp.pt.x;
                        const float &kpy = kp.pt.y;
                        const float &kpr = pKF->mvuRight[idx];
                        const float ex = u-kpx;
                        const float ey = v-kpy;
                        const float er = ur-kpr;
                        const float e2 = ex*ex+ey*ey+er*er;

                        if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8)
                            continue;
                    }
                    else
                    {
                        const float &kpx = kp.pt.x;
                        const float &kpy = kp.pt.y;
                        const float ex = u-kpx;
                        const float ey = v-kpy;
                        const float e2 = ex*ex+ey*ey;

                        if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                            continue;
                    }

                    const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                    const int dist = DescriptorDistance(dMP,dKF);

                    if(dist<bestDist)
                    {
                        bestDist = dist;
                        bestIdx = idx;
                    }
                }

                // If there is already a MapPoint replace otherwise add new measurement
                if(bestDist<=TH_LOW)
                {
                    MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
                    if(pMPinKF)
                    {
                        if(!pMPinKF->isBad())
                        {
                            if(pMPinKF->Observations()>pMP->Observations())
                                pMP->Replace(pMPinKF);
                            else
                                pMPinKF->Replace(pMP);
                        }
                    }
                    else
                    {
                        pMP->AddObservation(pKF,bestIdx);
                        pMP->UpdateSemanticInfo(pKF->mvSemanticClass[bestIdx],pKF->mvSemanticProbability[bestIdx]);
                        pKF->AddMapPoint(pMP,bestIdx);
                    }
                    nFused++;
                }
            }
        }


        return nFused;
    }





    //****************************functions about loop closure
    int ORBmatcher::SearchBySim3GroupCamera(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint*> &vpMatches12,
                                            const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th)
    {
        const float &fx = pKF1->fx;
        const float &fy = pKF1->fy;
        const float &cx = pKF1->cx;
        const float &cy = pKF1->cy;

        // Camera 1 from world
        cv::Mat R1w = pKF1->GetRotation();
        cv::Mat t1w = pKF1->GetTranslation();

        //Camera 2 from world
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();

        //Transformation between cameras
        cv::Mat sR12 = s12*R12;
        cv::Mat sR21 = (1.0/s12)*R12.t();
        cv::Mat t21 = -sR21*t12;

        const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
        const int N1 = vpMapPoints1.size();

        const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
        const int N2 = vpMapPoints2.size();

        vector<bool> vbAlreadyMatched1(N1,false);
        vector<bool> vbAlreadyMatched2(N2,false);

        for(int i=0; i<N1; i++)
        {
            MapPoint* pMP = vpMatches12[i];
            if(pMP)
            {
                vbAlreadyMatched1[i]=true;
                int idx2 = pMP->GetIndexInKeyFrame(pKF2);
                if(idx2>=0 && idx2<N2)
                    vbAlreadyMatched2[idx2]=true;
            }
        }

        vector<int> vnMatch1(N1,-1);
        vector<int> vnMatch2(N2,-1);

        // Transform from KF1 to KF2 and search
        for(int i1=0; i1<N1; i1++)
        {
            MapPoint* pMP = vpMapPoints1[i1];

            if(!pMP || vbAlreadyMatched1[i1])
                continue;

            if(pMP->isBad())
                continue;

            vector<size_t> vIndices;
            int nPredictedLevel;
            for(int c = 0;c<pKF2->Ncameras;c++)
            {
                cv::Mat p3Dw = pMP->GetWorldPos();
                cv::Mat p3Dc1 = R1w * p3Dw + t1w;
                cv::Mat p3Dc2 = sR21 * p3Dc1 + t21;
                p3Dc2 = pKF2->mvTcg[c].rowRange(0, 3).colRange(0, 3) * p3Dc2 + pKF2->mvTcg[c].col(3).rowRange(0, 3);
                //TODO wx-impl transform p3Dc2 to subcamera.
                // Depth must be positive
                if (p3Dc2.at<float>(2) < 0.0)
                    continue;

                const float invz = 1.0 / p3Dc2.at<float>(2);
                const float x = p3Dc2.at<float>(0) * invz;
                const float y = p3Dc2.at<float>(1) * invz;

                const float u = fx * x + cx;
                const float v = fy * y + cy;

                // Point must be inside the image
                if (!pKF2->IsInImage(u, v))
                    continue;

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();
                const float dist3D = cv::norm(p3Dc2);

                // Depth must be inside the scale invariance region
                if (dist3D < minDistance || dist3D > maxDistance)
                    continue;

                // Compute predicted octave
                nPredictedLevel = pMP->PredictScale(dist3D, pKF2);

                // Search in a radius
                const float radius = th * pKF2->mvScaleFactors[nPredictedLevel];

                auto indices_subcamera = pKF2->GetFeaturesInAreaSubCamera(u, v, radius, c);
                vIndices.insert(vIndices.end(),indices_subcamera.begin(),indices_subcamera.end());
            }


            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++)
            {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

                if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if (bestDist <= TH_HIGH)
            {
                vnMatch1[i1] = bestIdx;
            }

        }

        // Transform from KF2 to KF2 and search
        for(int i2=0; i2<N2; i2++)
        {
            MapPoint* pMP = vpMapPoints2[i2];

            if(!pMP || vbAlreadyMatched2[i2])
                continue;

            if(pMP->isBad())
                continue;

            vector<size_t> vIndices;
            int nPredictedLevel;
            for(int c = 0;c<pKF1->Ncameras;c++)
            {
                cv::Mat p3Dw = pMP->GetWorldPos();
                cv::Mat p3Dc2 = R2w * p3Dw + t2w;
                cv::Mat p3Dc1 = sR12 * p3Dc2 + t12;
                p3Dc1 = pKF1->mvTcg[c].rowRange(0, 3).colRange(0, 3) * p3Dc1 + pKF1->mvTcg[c].col(3).rowRange(0, 3);

                // Depth must be positive
                if (p3Dc1.at<float>(2) < 0.0)
                    continue;

                const float invz = 1.0 / p3Dc1.at<float>(2);
                const float x = p3Dc1.at<float>(0) * invz;
                const float y = p3Dc1.at<float>(1) * invz;

                const float u = fx * x + cx;
                const float v = fy * y + cy;

                // Point must be inside the image
                if (!pKF1->IsInImage(u, v))
                    continue;

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();
                const float dist3D = cv::norm(p3Dc1);

                // Depth must be inside the scale pyramid of the image
                if (dist3D < minDistance || dist3D > maxDistance)
                    continue;

                // Compute predicted octave
                const int nPredictedLevel = pMP->PredictScale(dist3D, pKF1);

                // Search in a radius of 2.5*sigma(ScaleLevel)
                const float radius = th * pKF1->mvScaleFactors[nPredictedLevel];

                auto indices_subcamera = pKF1->GetFeaturesInAreaSubCamera(u, v, radius, c);
                vIndices.insert(vIndices.end(),indices_subcamera.begin(),indices_subcamera.end());


            }
            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

                if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_HIGH)
            {
                vnMatch2[i2]=bestIdx;
            }
        }

        // Check agreement
        int nFound = 0;

        for(int i1=0; i1<N1; i1++)
        {
            int idx2 = vnMatch1[i1];

            if(idx2>=0)
            {
                int idx1 = vnMatch2[idx2];
                if(idx1==i1)
                {
                    vpMatches12[i1] = vpMapPoints2[idx2];
                    nFound++;
                }
            }
        }

        return nFound;
    }

//TODO wx-impl  next work.
    int ORBmatcher::SearchByProjectionGroupCamera(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)
    {

        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Decompose Scw
        cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
        const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
        cv::Mat Rcw = sRcw/scw;
        cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
        cv::Mat Ow = -Rcw.t()*tcw;

        // Set of MapPoints already found in the KeyFrame
        set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
        spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

        int nmatches=0;

        // For each Candidate MapPoint Project and Match
        for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
        {
            MapPoint* pMP = vpPoints[iMP];

            // Discard Bad MapPoints and already found
            if(pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // Get 3D Coords.
            cv::Mat p3Dw = pMP->GetWorldPos();

            // Transform into Camera Coords.
            cv::Mat p3Dc = Rcw*p3Dw+tcw;

            // Depth must be positive
            if(p3Dc.at<float>(2)<0.0)
                continue;

            // Project into Image
            const float invz = 1/p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0)*invz;
            const float y = p3Dc.at<float>(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // Point must be inside the image
            if(!pKF->IsInImage(u,v))
                continue;

            // Depth must be inside the scale invariance region of the point
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = p3Dw-Ow;
            const float dist = cv::norm(PO);

            if(dist<minDistance || dist>maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            cv::Mat Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist)
                continue;

            int nPredictedLevel = pMP->PredictScale(dist,pKF);

            // Search in a radius
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit;
                if(vpMatched[idx])
                    continue;

                const int &kpLevel= pKF->mvKeysUn[idx].octave;

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_LOW)
            {
                vpMatched[bestIdx]=pMP;
                nmatches++;
            }

        }

        return nmatches;
    }



}
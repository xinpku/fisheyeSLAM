#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>


namespace ORB_SLAM2
{
    void LocalMapping::CreateNewMapPointsGroupCamera()
    {
        // Retrieve neighbor keyframes in covisibility graph
        //std::cout<<"Mapping CreateNewMapPoints "<<std::endl;
        int nn = 10;
        if(mbMonocular)
            nn=20;
        const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

        ORBmatcher matcher(0.6,false);

        const float &fx1 = mpCurrentKeyFrame->fx;
        const float &fy1 = mpCurrentKeyFrame->fy;
        const float &cx1 = mpCurrentKeyFrame->cx;
        const float &cy1 = mpCurrentKeyFrame->cy;
        const float &invfx1 = mpCurrentKeyFrame->invfx;
        const float &invfy1 = mpCurrentKeyFrame->invfy;

        const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

        int nnew=0;

        int Ncameras = mpCurrentKeyFrame->Ncameras;

        // Search matches with epipolar restriction and triangulate
        for(size_t i=0; i<vpNeighKFs.size(); i++)
        {
            if(i>0 && CheckNewKeyFrames())
                return;

            KeyFrame* pKF2 = vpNeighKFs[i];

            // Check first that baseline is not too short
            cv::Mat Ow2 = pKF2->GetCameraCenter();
            cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
            cv::Mat vBaseline = Ow2-Ow1;
            const float baseline = cv::norm(vBaseline);

            if(!mbMonocular)
            {
                if(baseline<pKF2->mb)
                    continue;
            }
            else
            {
                const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
                const float ratioBaselineDepth = baseline/medianDepthKF2;

                if(ratioBaselineDepth<0.01)
                    continue;
            }


            const float &fx2 = pKF2->fx;
            const float &fy2 = pKF2->fy;
            const float &cx2 = pKF2->cx;
            const float &cy2 = pKF2->cy;
            const float &invfx2 = pKF2->invfx;
            const float &invfy2 = pKF2->invfy;

            for(int c = 0;c<Ncameras;c++)
            {
                // Compute Fundamental Matrix
                cv::Mat F12SubCamera = ComputeF12SubCamera(mpCurrentKeyFrame,pKF2,c);

                // Search matches that fullfil epipolar constraint
                vector<pair<size_t,size_t> > vMatchedIndices;
                matcher.SearchForTriangulationGroupCamera(mpCurrentKeyFrame,pKF2,c,c,F12SubCamera,vMatchedIndices,false);

                cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotationSubCamera(c);
                cv::Mat Rwc1 = Rcw1.t();
                cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslationSubCamera(c);
                cv::Mat Tcw1(3,4,CV_32F);
                Rcw1.copyTo(Tcw1.colRange(0,3));
                tcw1.copyTo(Tcw1.col(3));


                cv::Mat Rcw2 = pKF2->GetRotationSubCamera(c);
                cv::Mat Rwc2 = Rcw2.t();
                cv::Mat tcw2 = pKF2->GetTranslationSubCamera(c);
                cv::Mat Tcw2(3,4,CV_32F);
                Rcw2.copyTo(Tcw2.colRange(0,3));
                tcw2.copyTo(Tcw2.col(3));

                // Triangulate each match
                const int nmatches = vMatchedIndices.size();
                for(int ikp=0; ikp<nmatches; ikp++)
                {
                    const int &idx1 = vMatchedIndices[ikp].first;
                    const int &idx2 = vMatchedIndices[ikp].second;

                    const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
                    const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
                    bool bStereo1 = kp1_ur>=0;

                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
                    const float kp2_ur = pKF2->mvuRight[idx2];
                    bool bStereo2 = kp2_ur>=0;

                    // Check parallax between rays
                    cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
                    cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

                    cv::Mat ray1 = Rwc1*xn1;
                    cv::Mat ray2 = Rwc2*xn2;
                    const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

                    float cosParallaxStereo = cosParallaxRays+1;
                    float cosParallaxStereo1 = cosParallaxStereo;
                    float cosParallaxStereo2 = cosParallaxStereo;

                    if(bStereo1)
                        cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
                    else if(bStereo2)
                        cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

                    cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

                    cv::Mat x3D;
                    if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
                    {
                        // Linear Triangulation Method
                        cv::Mat A(4,4,CV_32F);
                        A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                        A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                        A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                        A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                        cv::Mat w,u,vt;
                        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                        x3D = vt.row(3).t();

                        if(x3D.at<float>(3)==0)
                            continue;

                        // Euclidean coordinates
                        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

                    }
                    else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
                    {
                        x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
                    }
                    else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
                    {
                        x3D = pKF2->UnprojectStereo(idx2);
                    }
                    else
                        continue; //No stereo and very low parallax

                    cv::Mat x3Dt = x3D.t();

                    //Check triangulation in front of cameras
                    float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
                    if(z1<=0)
                        continue;

                    float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
                    if(z2<=0)
                        continue;

                    //Check reprojection error in first keyframe
                    const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                    const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
                    const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
                    const float invz1 = 1.0/z1;

                    if(!bStereo1)
                    {
                        float u1 = fx1*x1*invz1+cx1;
                        float v1 = fy1*y1*invz1+cy1;
                        float errX1 = u1 - kp1.pt.x;
                        float errY1 = v1 - kp1.pt.y;
                        if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                            continue;
                    }
                    else
                    {
                        float u1 = fx1*x1*invz1+cx1;
                        float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                        float v1 = fy1*y1*invz1+cy1;
                        float errX1 = u1 - kp1.pt.x;
                        float errY1 = v1 - kp1.pt.y;
                        float errX1_r = u1_r - kp1_ur;
                        if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                            continue;
                    }

                    //Check reprojection error in second keyframe
                    const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
                    const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
                    const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
                    const float invz2 = 1.0/z2;
                    if(!bStereo2)
                    {
                        float u2 = fx2*x2*invz2+cx2;
                        float v2 = fy2*y2*invz2+cy2;
                        float errX2 = u2 - kp2.pt.x;
                        float errY2 = v2 - kp2.pt.y;
                        if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                            continue;
                    }
                    else
                    {
                        float u2 = fx2*x2*invz2+cx2;
                        float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                        float v2 = fy2*y2*invz2+cy2;
                        float errX2 = u2 - kp2.pt.x;
                        float errY2 = v2 - kp2.pt.y;
                        float errX2_r = u2_r - kp2_ur;
                        if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                            continue;
                    }

                    //Check scale consistency
                    cv::Mat normal1 = x3D-Ow1;
                    float dist1 = cv::norm(normal1);

                    cv::Mat normal2 = x3D-Ow2;
                    float dist2 = cv::norm(normal2);

                    if(dist1==0 || dist2==0)
                        continue;

                    const float ratioDist = dist2/dist1;
                    const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

                    /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                        continue;*/
                    if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                        continue;

                    // Triangulation is succesfull
                    MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);
                    //wx-debug we simply set the semantic class of the map point the same as current keyframe. But it should be a fusion processing according the probability of every class type.
                    KeyFrame* semantic_source;
                    int semantic_idx;
                    if(mpCurrentKeyFrame->mvSemanticProbability[idx1]>pKF2->mvSemanticProbability[idx2])
                    {
                        semantic_source = mpCurrentKeyFrame;
                        semantic_idx = idx1;
                    }
                    else
                    {
                        semantic_source = pKF2;
                        semantic_idx = idx2;
                    }

                    pMP->mSemanticClass = semantic_source->mvSemanticClass[semantic_idx];
                    pMP->mSemanticProb = semantic_source->mvSemanticProbability[semantic_idx];

                    pMP->AddObservation(mpCurrentKeyFrame,idx1);
                    pMP->AddObservation(pKF2,idx2);

                    mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
                    pKF2->AddMapPoint(pMP,idx2);

                    pMP->ComputeDistinctiveDescriptors();

                    pMP->UpdateNormalAndDepth();

                    mpMap->AddMapPoint(pMP);
                    mlpRecentAddedMapPoints.push_back(pMP);

                    nnew++;
                }
            }

        }
    }


    cv::Mat LocalMapping::ComputeF12SubCamera(KeyFrame *&pKF1, KeyFrame *&pKF2,int subCameraID)
    {
        cv::Mat R1w = pKF1->GetRotationSubCamera(subCameraID);
        cv::Mat t1w = pKF1->GetTranslationSubCamera(subCameraID);
        cv::Mat R2w = pKF2->GetRotationSubCamera(subCameraID);
        cv::Mat t2w = pKF2->GetTranslationSubCamera(subCameraID);

        cv::Mat R12 = R1w*R2w.t();
        cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

        cv::Mat t12x = SkewSymmetricMatrix(t12);

        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;


        return K1.t().inv()*t12x*R12*K2.inv();
    }


    void LocalMapping::ProcessNewKeyFrameGroupCamera()
    {
        {
            unique_lock<mutex> lock(mMutexNewKFs);
            mpCurrentKeyFrame = mlNewKeyFrames.front();
            mlNewKeyFrames.pop_front();
        }

        // Compute Bags of Words structures
        mpCurrentKeyFrame->ComputeBoW();

        // Associate MapPoints to the new keyframe and update normal and descriptor
        const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            MapPoint* pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                    {
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        pMP->UpdateSemanticInfo(mpCurrentKeyFrame->mvSemanticClass[i],mpCurrentKeyFrame->mvSemanticProbability[i]);
                        pMP->UpdateNormalAndDepth();
                        pMP->ComputeDistinctiveDescriptors();
                    }
                    else // this can only happen for new stereo points inserted by the Tracking
                    {
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }

        // Update links in the Covisibility Graph
        mpCurrentKeyFrame->UpdateConnectionsGroupCamera();

        // Insert Keyframe in Map
        mpMap->AddKeyFrame(mpCurrentKeyFrame);
    }

    void LocalMapping::SearchInNeighborsGroupCamera()
    {
        // Retrieve neighbor keyframes
        int nn = 10;
        if(mbMonocular)
            nn=20;
        const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
        vector<KeyFrame*> vpTargetKFs;
        for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;
            if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

            // Extend to some second neighbors
            const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
            for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
            {
                KeyFrame* pKFi2 = *vit2;
                if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                    continue;
                vpTargetKFs.push_back(pKFi2);
            }
        }


        // Search matches by projection from current KF in target KFs
        ORBmatcher matcher;
        vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            matcher.FuseGroupCamera(pKFi,vpMapPointMatches);
        }

        // Search matches by projection from target KFs in current KF
        vector<MapPoint*> vpFuseCandidates;
        vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

        for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
        {
            KeyFrame* pKFi = *vitKF;

            vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

            for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
            {
                MapPoint* pMP = *vitMP;
                if(!pMP)
                    continue;
                if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                    continue;
                pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
                vpFuseCandidates.push_back(pMP);
            }
        }

        matcher.FuseGroupCamera(mpCurrentKeyFrame,vpFuseCandidates);


        // Update points
        vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
        {
            MapPoint* pMP=vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    pMP->ComputeDistinctiveDescriptors();
                    pMP->UpdateNormalAndDepth();
                }
            }
        }

        // Update connections in covisibility graph
        mpCurrentKeyFrame->UpdateConnectionsGroupCamera();
    }


    void LocalMapping::RunGroupCamera()
    {

        mbFinished = false;

        while(1)
        {
            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(false);

            // Check if there are keyframes in the queue
            if(CheckNewKeyFrames())
            {
                // BoW conversion and insertion in Map
                ProcessNewKeyFrameGroupCamera();

                // Check recent MapPoints
                MapPointCulling();

                // Triangulate new MapPoints
                CreateNewMapPointsGroupCamera();

                if(!CheckNewKeyFrames())
                {
                    // Find more matches in neighbor keyframes and fuse point duplications
                    SearchInNeighborsGroupCamera();
                }

                mbAbortBA = false;

                if(!CheckNewKeyFrames() && !stopRequested())
                {
                    // Local BA
                    if(mpMap->KeyFramesInMap()>2)
                        Optimizer::LocalBundleAdjustmentGroupCamera(mpCurrentKeyFrame,&mbAbortBA, mpMap);

                    // Check redundant local Keyframes
                    KeyFrameCulling();
                }

                mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            }
            else if(Stop())
            {
                // Safe area to stop
                while(isStopped() && !CheckFinish())
                {
                    usleep(3000);
                }
                if(CheckFinish())
                    break;
            }

            ResetIfRequested();

            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(true);

            if(CheckFinish())
                break;

            usleep(3000);
        }

        SetFinish();
    }



}
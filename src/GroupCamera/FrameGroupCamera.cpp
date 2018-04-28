#include "Frame.h"
#include "debug_utils/debug_utils.h"
#include "ORBmatcher.h"

namespace ORB_SLAM2
{
    //*********************************************
    //The functions related to the groupCamera

    Frame::Frame(const std::vector<cv::Mat> &imGrays, std::vector<FisheyeCorrector> &correctors,const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,std::vector<cv::Mat>& Tcg)
            :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
             mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;


            mnMinX = 0.0f;
            mnMaxX = cx*2;
            mnMinY = 0.0f;
            mnMaxY = cy*2;

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            mbInitialComputations = false;
        }

        Ncameras = Tcg.size();

        kp_start_pos.resize(Ncameras);
        mvTcg.resize(Tcg.size());
        mvTgc.resize(Tcg.size());
        mvTcwSubcamera.resize(Tcg.size());
        mvOwSubcamera.resize(Tcg.size());
        mvRcwSubcamera.resize(Tcg.size());
        mvtcwSubcamera.resize(Tcg.size());
        for (int i = 0; i < Tcg.size(); i++)
        {
            Tcg[i].copyTo(mvTcg[i]);
            mvTgc[i] = Tcg[i].inv();
        }
        // ORB extraction
        ExtractORBGroupCamera(imGrays, correctors);


        N = mvKeys.size();

        if (mvKeys.empty())
            return;

        //UndistortKeyPoints();

        // Set no stereo information
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        mvpMapPoints = vector<MapPoint*>(N, static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N, false);



        mb = mbf / fx;

        AssignFeaturesToGrid();
    }


    void Frame::ExtractORBGroupCamera(const std::vector<cv::Mat> &imGrays, std::vector<FisheyeCorrector> &correctors)
    {
        for (int i = 0; i < imGrays.size(); i++)
        {
            kp_start_pos[i] = mvCamera_Id_KeysUn.size();
            for(int v = 0;v<correctors.size();v++)
            {
                cv::Mat im_view;
                correctors[v].correct(imGrays[i],im_view);
                std::vector<cv::KeyPoint> mvKeys_temp,mvfisheye_keys_current,mvundist_keys_current;
                cv::Mat descriptors_temp;
                (*mpORBextractorLeft)(im_view, cv::Mat(), mvKeys_temp, descriptors_temp);

                for(auto& p:mvKeys_temp)
                    p.class_id = i;
                correctors[v].mapToOriginalImage(mvKeys_temp, mvfisheye_keys_current);
                correctors[v].mapFromCorrectedImageToCenterImagePlane(mvKeys_temp, mvundist_keys_current, cx, cy, fx);



                mvKeys.insert(mvKeys.end(),mvfisheye_keys_current.begin(),mvfisheye_keys_current.end());
                mvKeysUn.insert(mvKeysUn.end(), mvundist_keys_current.begin(),mvundist_keys_current.end());
                mDescriptors.push_back(descriptors_temp);

                std::vector<int> camera_id(mvKeys_temp.size(),i);
                mvCamera_Id_KeysUn.insert(mvCamera_Id_KeysUn.end(),camera_id.begin(),camera_id.end());

/*                cv::Mat part;
		cv::drawKeypoints(im_view, mvKeys_temp, part);
		std::stringstream sst;
		sst << "part" << v;
		cv::namedWindow(sst.str(), 0);
        std::cout<<sst.str()<<"  "<<im_view.size()<<std::endl;
		cv::imshow(sst.str(),part);
                cv::waitKey(10);*/
            }

        }


        mvSemanticClass = std::vector<SemanticClass>(mvKeys.size(),SemanticClass::nBackground);
        mvSemanticProbability =  std::vector<uchar>(mvKeys.size(),100);
    }

    void Frame::UpdateMultiCameraPose()
    {
        for (int i = 0; i < mvTcg.size(); i++)
        {
            mvTcwSubcamera[i] = mvTcg[i] * mTcw;
            mvOwSubcamera[i] = -(mvTcwSubcamera[i].rowRange(0, 3).colRange(0, 3)).t()*(mvTcwSubcamera[i].rowRange(0, 3).col(3));
            mvTcwSubcamera[i].rowRange(0, 3).colRange(0, 3).copyTo(mvRcwSubcamera[i]);
            mvTcwSubcamera[i].rowRange(0, 3).col(3).copyTo(mvtcwSubcamera[i]);
        }
    }

    bool Frame::isInFrustumGroupCamera(MapPoint *pMP, float viewingCosLimit)
    {
        bool state = false;
        pMP->mvbTrackInView.resize(Ncameras);
        pMP->mvTrackProjX.resize(Ncameras);
        pMP->mvTrackProjXR.resize(Ncameras);
        pMP->mvTrackProjY.resize(Ncameras);
        pMP->mvnTrackScaleLevel.resize(Ncameras);
        pMP->mvTrackViewCos.resize(Ncameras);
        for(int c = 0 ;c<Ncameras;c++)
        {
            pMP->mvbTrackInView[c] = false;

            // 3D in absolute coordinates
            cv::Mat P = pMP->GetWorldPos();

            const cv::Mat Pc = mvRcwSubcamera[c]*P+mvtcwSubcamera[c];
            const float &PcX = Pc.at<float>(0);
            const float &PcY= Pc.at<float>(1);
            const float &PcZ = Pc.at<float>(2);

            // Check positive depth
            if(PcZ<0.0f)
            {
                continue;
            }

            // Project in image and check it is not outside
            const float invz = 1.0f/PcZ;
            const float u=fx*PcX*invz+cx;
            const float v=fy*PcY*invz+cy;

            if(u<mnMinX || u>mnMaxX)
            {
                continue;
            }
            if(v<mnMinY || v>mnMaxY)
            {
                continue;
            }

            // Check distance is in the scale invariance region of the MapPoint
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const cv::Mat PO = P-mvOwSubcamera[c];
            const float dist = cv::norm(PO);

            if(dist<minDistance || dist>maxDistance)
            {
                continue;
            }

            // Check viewing angle
            cv::Mat Pn = pMP->GetNormal();

            const float viewCos = PO.dot(Pn)/dist;

            if(viewCos<viewingCosLimit)
            {
                continue;
            }

            // Predict scale in the image
            const int nPredictedLevel = pMP->PredictScale(dist,this);

            // Data used by the tracking
            pMP->mvbTrackInView[c] = true;
            pMP->mvTrackProjX[c] = u;
            pMP->mvTrackProjXR[c] = u - mbf*invz;
            pMP->mvTrackProjY[c] = v;
            pMP->mvnTrackScaleLevel[c]= nPredictedLevel;
            pMP->mvTrackViewCos[c] = viewCos;
            state = true;
        }

        return state;
    }


    std::vector<cv::KeyPoint> Frame::getKeypointUnSubCamera(int c) const
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


    vector<size_t> Frame::GetFeaturesInAreaSubCamera(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel,int cameraID) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
        if(nMinCellX>=FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
        if(nMaxCellX<0)
            return vIndices;

        const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
        if(nMinCellY>=FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
        if(nMaxCellY<0)
            return vIndices;

        const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);


        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                if(vCell.empty())
                    continue;

                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {

                    if(mvCamera_Id_KeysUn[vCell[j]]!=cameraID)
                        continue;
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    if(bCheckLevels)
                    {
                        if(kpUn.octave<minLevel)
                            continue;
                        if(maxLevel>=0)
                            if(kpUn.octave>maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }
        //print_value(vIndices.size());
        return vIndices;
    }



    void Frame::ComputeStereoGroupCamera()
    {
        mvuRight = vector<float>(N,-1.0f);
        mvDepth = vector<float>(N,-1.0f);

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        for(int c = 0;c<Ncameras;c++)
        {
            // Compute Fundamental Matrix
            cv::Mat F12SubCamera = ComputeF12SubCamera(mpCurrentKeyFrame, pKF2, c);
            print_string("((((((((((((((((((((((((((((((((((")
            print_value(c)
            // Search matches that fullfil epipolar constraint
            vector<pair<size_t, size_t> > vMatchedIndices;
            matcher.SearchForTriangulationGroupCamera(mpCurrentKeyFrame, pKF2, c, c, F12SubCamera, vMatchedIndices,
                                                      false);

            cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotationSubCamera(c);
            cv::Mat Rwc1 = Rcw1.t();
            cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslationSubCamera(c);
            cv::Mat Tcw1(3, 4, CV_32F);
            Rcw1.copyTo(Tcw1.colRange(0, 3));
            tcw1.copyTo(Tcw1.col(3));


            cv::Mat Rcw2 = pKF2->GetRotationSubCamera(c);
            cv::Mat Rwc2 = Rcw2.t();
            cv::Mat tcw2 = pKF2->GetTranslationSubCamera(c);
            cv::Mat Tcw2(3, 4, CV_32F);
            Rcw2.copyTo(Tcw2.colRange(0, 3));
            tcw2.copyTo(Tcw2.col(3));



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

                const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
                const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
                bool bStereo1 = kp1_ur >= 0;

                const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
                const float kp2_ur = pKF2->mvuRight[idx2];
                bool bStereo2 = kp2_ur >= 0;

                // Check parallax between rays
                cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
                cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2) * invfx2, (kp2.pt.y - cy2) * invfy2, 1.0);

                cv::Mat ray1 = Rwc1 * xn1;
                cv::Mat ray2 = Rwc2 * xn2;

                float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

                //TODO: wx-deug-test\
                    Only for the mapping of front camera
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
                    cosParallaxStereo1 = cos(2 * atan2(mpCurrentKeyFrame->mb / 2, mpCurrentKeyFrame->mvDepth[idx1]));
                else if (bStereo2)
                    cosParallaxStereo2 = cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

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
                    x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
                } else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1)
                {
                    x3D = pKF2->UnprojectStereo(idx2);
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
                const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
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
                    float u1_r = u1 - mpCurrentKeyFrame->mbf * invz1;
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
                const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
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
                    float u2_r = u2 - mpCurrentKeyFrame->mbf * invz2;
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
                        mpCurrentKeyFrame->mvScaleFactors[kp1.octave] / pKF2->mvScaleFactors[kp2.octave];

                crash_reason.back() = 7;
                /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                    continue;*/
                //print_value(ratioDist,c==0)
                //print_value(ratioOctave,c==0)
                if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
                    continue;

                // Triangulation is succesfull
                MapPoint *pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);
                pMP->created_by_kf1 = c;
                print_vect_cv(x3D, new_local < 10);
                new_local++;
                //wx-debug we simply set the semantic class of the map point the same as current keyframe. But it should be a fusion processing according the probability of every class type.
                KeyFrame *semantic_source;
                int semantic_idx;
                if (mpCurrentKeyFrame->mvSemanticProbability[idx1] > pKF2->mvSemanticProbability[idx2])
                {
                    semantic_source = mpCurrentKeyFrame;
                    semantic_idx = idx1;
                } else
                {
                    semantic_source = pKF2;
                    semantic_idx = idx2;
                }

                pMP->mSemanticClass = semantic_source->mvSemanticClass[semantic_idx];
                pMP->mSemanticProb = semantic_source->mvSemanticProbability[semantic_idx];

                pMP->AddObservation(mpCurrentKeyFrame, idx1);
                pMP->AddObservation(pKF2, idx2);

                mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
                pKF2->AddMapPoint(pMP, idx2);

                pMP->ComputeDistinctiveDescriptors();

                pMP->UpdateNormalAndDepth();

                mpMap->AddMapPoint(pMP);
                mlpRecentAddedMapPoints.push_back(pMP);
                crash_reason.back() = 0;
                nnew++;
            }
        }
    }



}
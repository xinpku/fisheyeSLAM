#include "Frame.h"
#include "debug_utils/debug_utils.h"

namespace ORB_SLAM2
{
    //*********************************************
    //The functions related to the groupCamera

    Frame::Frame(const std::vector<cv::Mat> &imGrays, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,std::vector<cv::Mat>& Tcg)
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
        ExtractORBGroupCamera(imGrays);


        N = mvKeys.size();

        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        // Set no stereo information
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        mvpMapPoints = vector<MapPoint*>(N, static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imGrays[0]);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        AssignFeaturesToGrid();
    }


    void Frame::ExtractORBGroupCamera(const std::vector<cv::Mat> &imGrays)
    {
        for (int i = 0; i < imGrays.size(); i++)
        {
            std::vector<cv::KeyPoint> mvKeys_temp;
            cv::Mat descriptors_temp;
            (*mpORBextractorLeft)(imGrays[i], cv::Mat(), mvKeys_temp, descriptors_temp);

            for(auto& p:mvKeys_temp)
                p.class_id = i;

            mvKeys.insert(mvKeys.end(),mvKeys_temp.begin(),mvKeys_temp.end());
            print_value(mvKeys.size())
            std::vector<int> camera_id(mvKeys_temp.size(),i);
            kp_start_pos[i] = mvCamera_Id_KeysUn.size();
            mvCamera_Id_KeysUn.insert(mvCamera_Id_KeysUn.end(),camera_id.begin(),camera_id.end());
            mDescriptors.push_back(descriptors_temp);
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

}
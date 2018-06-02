#include "Frame.h"
#include "debug_utils/debug_utils.h"
#include "ORBmatcher.h"
#include <map>
#include <iostream>

namespace ORB_SLAM2
{
    //*********************************************
    //The functions related to the groupCamera

    Frame::Frame(const std::vector<cv::Mat> &imGrays, std::vector<FisheyeCorrector,Eigen::aligned_allocator<FisheyeCorrector>> &correctors,const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, std::vector<cv::Mat> &K, cv::Mat &distCoef, const float &bf, const float &thDepth,std::vector<cv::Mat>& Tcg)
            :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
             mTimeStamp(timeStamp),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
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
        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            mvK.resize(Ncameras);
            mvfx.resize(Ncameras);
            mvfy.resize(Ncameras);
            mvcx.resize(Ncameras);
            mvcy.resize(Ncameras);
            mvInvfx.resize(Ncameras);
            mvInvfy.resize(Ncameras);
            for(int c =0;c<Ncameras;c++)
            {
                mvK[c] = K[c].clone();
                mvfx[c] = K[c].at<float>(0,0);
                mvfy[c] = K[c].at<float>(1,1);
                mvcx[c] = K[c].at<float>(0,2);
                mvcy[c] = K[c].at<float>(1,2);
                mvInvfx[c] = 1.0f/mvfx[c];
                mvInvfy[c] = 1.0f/mvfy[c];
            }


            mnMinX = 0.0f;
            mnMaxX = *(std::max_element(mvcx.begin(),mvcx.end()))*2;
            mnMinY = 0.0f;
            mnMaxY = *(std::max_element(mvcy.begin(),mvcy.end()))*2;

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            mbInitialComputations = false;
        }



        kp_start_pos.resize(Ncameras);
        mvTcg.resize(Tcg.size());
        mvTgc.resize(Tcg.size());
        mvTcwSubcamera.resize(Tcg.size());
        mvOwSubcamera.resize(Tcg.size());
        mvRcwSubcamera.resize(Tcg.size());
        mvtcwSubcamera.resize(Tcg.size());
        mvK.resize(Ncameras);
        for (int i = 0; i < Tcg.size(); i++)
        {
            Tcg[i].copyTo(mvTcg[i]);
            mvTgc[i] = Tcg[i].inv();
            mvK[i] = K[i].clone();
        }
        // ORB extraction
        ExtractORBGroupCamera(imGrays, correctors);



        N = mvKeys.size();
        keypoints_stat.push_back(N);
        if (mvKeys.empty())
            return;

        //UndistortKeyPoints();

        // Set no stereo information
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        mvpMapPoints = vector<MapPoint*>(N, static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N, false);



        //mb = mbf / fx;

        AssignFeaturesToGrid();
    }


    void Frame::ExtractORBGroupCamera(const std::vector<cv::Mat> &imGrays, std::vector<FisheyeCorrector,Eigen::aligned_allocator<FisheyeCorrector>> &correctors)
    {
        for (int i = 0; i < imGrays.size(); i++)
        {
            kp_start_pos[i] = mvCamera_Id_KeysUn.size();
            /*for(int v = 0;v<correctors.size();v++)
            {*/
                cv::Mat im_view;
                correctors[i].correct(imGrays[i],im_view);
                std::vector<cv::KeyPoint> mvKeys_temp,mvfisheye_keys_current,mvundist_keys_current;
                cv::Mat descriptors_temp;
                (*mpORBextractorLeft)(im_view, cv::Mat(), mvKeys_temp, descriptors_temp);

                for(auto& p:mvKeys_temp)
                    p.class_id = i;
                correctors[i].mapToOriginalImage(mvKeys_temp, mvfisheye_keys_current);
                //correctors[i].mapFromCorrectedImageToCenterImagePlane(mvKeys_temp, mvundist_keys_current, cx, cy, fx);
                mvundist_keys_current = mvKeys_temp;


                mvKeys.insert(mvKeys.end(),mvfisheye_keys_current.begin(),mvfisheye_keys_current.end());
                mvKeysUn.insert(mvKeysUn.end(), mvundist_keys_current.begin(),mvundist_keys_current.end());
                mDescriptors.push_back(descriptors_temp);

                std::vector<int> camera_id(mvKeys_temp.size(),i);
                mvCamera_Id_KeysUn.insert(mvCamera_Id_KeysUn.end(),camera_id.begin(),camera_id.end());

                cv::Mat part;
		cv::drawKeypoints(im_view, mvKeys_temp, part);
		std::stringstream sst;
		sst << "part" << i;
		cv::namedWindow(sst.str(), 0);
        std::cout<<sst.str()<<"  "<<im_view.size()<<std::endl;
		cv::imshow(sst.str(),part);
                cv::waitKey(10);
            //}

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
            const float u=mvfx[c]*PcX*invz+mvcx[c];
            const float v=mvfy[c]*PcY*invz+mvcy[c];

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
#include "KeyFrame.h"


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
        std::vector<cv::Mat> Tcw_;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = mvpMapPoints;
            for(int c = 0;c<Ncameras;c++)
            {
                Tcw_[c] = getTcwSubCamera(c).clone();
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
}
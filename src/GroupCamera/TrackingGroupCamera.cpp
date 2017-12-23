#include "Tracking.h"
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>

namespace ORB_SLAM2
{
    //****************************************
    //Functions related to the groupCamera

    void Tracking::SearchLocalPointsGroupCamera()
    {
        // Do not search map points already matched
        for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
            {
                if(pMP->isBad())
                {
                    *vit = static_cast<MapPoint*>(NULL);
                }
                else
                {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    pMP->mbTrackInView = false;
                }
            }
        }

        int nToMatch=0;

        // Project points in frame and check its visibility
        for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if(pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            if(mCurrentFrame.isInFrustumGroupCamera(pMP,0.5))
            {
                pMP->IncreaseVisible();
                nToMatch++;
            }
        }

        if(nToMatch>0)
        {
            ORBmatcher matcher(0.8);
            int th = 1;
            if(mSensor==System::RGBD)
                th=3;
            // If the camera has been relocalised recently, perform a coarser search
            if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
                th=5;
            matcher.SearchByProjectionGroupCamera(mCurrentFrame,mvpLocalMapPoints,th);
        }
    }







    cv::Mat Tracking::GrabImageMultiCamera(const std::vector<cv::Mat> &ims, const double &timestamp)
    {
        for (int i = 0; i < ims.size(); i++)
        {
            if (ims[i].channels() == 3)
            {
                if (mbRGB)
                    cvtColor(ims[i], ims[i], CV_RGB2GRAY);
                else
                    cvtColor(ims[i], ims[i], CV_BGR2GRAY);
            }
            else if (ims[i].channels() == 4)
            {
                if (mbRGB)
                    cvtColor(ims[i], ims[i], CV_RGBA2GRAY);
                else
                    cvtColor(ims[i], ims[i], CV_BGRA2GRAY);
            }
        }
        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            mCurrentFrame = Frame(ims, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mvTcg);
        else
            mCurrentFrame = Frame(ims, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mvTcg);

        MontageImagesKeypoints(ims, mImGray, mCurrentFrame);

        Track();

        return mCurrentFrame.mTcw.clone();
    }




    void Tracking::TrackGroupCamera()
    {
        if(mState==NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }

        mLastProcessedState=mState;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        if(mState==NOT_INITIALIZED)
        {
            if(mSensor==System::STEREO || mSensor==System::RGBD)
                StereoInitialization();
            else
                MonocularInitialization();

            mpFrameDrawer->Update(this);

            if(mState!=OK)
                return;
        }
        else
        {
            // System is initialized. Track Frame.
            bool bOK;

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            if(!mbOnlyTracking)
            {
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.

                if(mState==OK)
                {
                    // Local Mapping might have changed some MapPoints tracked in last frame
                    CheckReplacedInLastFrame();

                    if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                    {
                        bOK = TrackReferenceKeyFrameGroupCamera();
                    }
                    else
                    {
                        bOK = TrackWithMotionModelGroupCamera();
                        if(!bOK)
                            bOK = TrackReferenceKeyFrameGroupCamera();
                    }
                }
                else
                {
                    bOK = Relocalization();
                }
            }
            else
            {
                // Only Tracking: Local Mapping is deactivated

                if(mState==LOST)
                {
                    bOK = Relocalization();
                }
                else
                {
                    if(!mbVO)
                    {
                        // In last frame we tracked enough MapPoints in the map

                        if(!mVelocity.empty())
                        {
                            bOK = TrackWithMotionModelGroupCamera();
                        }
                        else
                        {
                            bOK = TrackReferenceKeyFrameGroupCamera();
                        }
                        if (!bOK)
                        {
                            std::cout << "trackWithMotionModel fail" << std::endl;
                        }

                    }
                    else
                    {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.

                        bool bOKMM = false;
                        bool bOKReloc = false;
                        vector<MapPoint*> vpMPsMM;
                        vector<bool> vbOutMM;
                        cv::Mat TcwMM;
                        if(!mVelocity.empty())
                        {
                            bOKMM = TrackWithMotionModelGroupCamera();
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.mTcw.clone();
                        }
                        bOKReloc = Relocalization();

                        if(bOKMM && !bOKReloc)
                        {
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if(mbVO)
                            {
                                for(int i =0; i<mCurrentFrame.N; i++)
                                {
                                    if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                    {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        }
                        else if(bOKReloc)
                        {
                            mbVO = false;
                        }

                        bOK = bOKReloc || bOKMM;
                    }
                }
            }

            mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // If we have an initial estimation of the camera pose and matching. Track the local map.
            if(!mbOnlyTracking)
            {
                //std::cout << "tracking statue reference frame: " << bOK << std::endl;
                /*if (!bOK)
                {
                    std::cout << "trackWithReference fail" << std::endl;
                    std::cout<<mCurrentFrame.mTcw<<std::endl;
                }*/

                if(mState==OK)
                    bOK |= TrackLocalMapGroupCamera();

                if (!bOK)
                    std::cout << "trackWithLocalMap fail" << std::endl;
            }
            else
            {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therfore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                if(bOK && !mbVO)
                    bOK = TrackLocalMapGroupCamera();
            }

            if(bOK)
                mState = OK;
            else
            {
                mState=LOST;
                std::cout<<"lost"<<std::endl;
            }



            // Update drawer
            mpFrameDrawer->Update(this);

            // If tracking were good, check if we insert a keyframe
            if(bOK)
            {
                // Update motion model
                if(!mLastFrame.mTcw.empty())
                {
                    cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                    mVelocity = mCurrentFrame.mTcw*LastTwc;
                }
                else
                    mVelocity = cv::Mat();

                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

                // Clean temporal point matches
                for(int i=0; i<mCurrentFrame.N; i++)
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                    if(pMP)
                        if(pMP->Observations()<1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                        }
                }

                // Delete temporal MapPoints
                for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
                {
                    MapPoint* pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();

                // Check if we need to insert a new keyframe
                if(NeedNewKeyFrame())
                    CreateNewKeyFrame();

                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame.
                for(int i=0; i<mCurrentFrame.N;i++)
                {
                    if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                }
            }

            // Reset if the camera get lost soon after initialization
            if(mState==LOST)
            {
                if(mpMap->KeyFramesInMap()<=5)
                {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }

            if(!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            mLastFrame = Frame(mCurrentFrame);
        }

        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if(!mCurrentFrame.mTcw.empty())
        {
            cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            if(!mlRelativeFramePoses.empty())
            {
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                mlFrameTimes.push_back(mlFrameTimes.back());
                mlbLost.push_back(mState==LOST);
            }

        }

    }


    bool Tracking::TrackWithMotionModelGroupCamera()
    {
        ORBmatcher matcher(0.9,true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points
        UpdateLastFrame();

        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        // Project points seen in previous frame
        int th;
        if(mSensor!=System::STEREO)
            th=15;
        else
            th=7;
        int nmatches = matcher.SearchByProjectionGroupCamera(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

        // If few matches, uses a wider window search
        if(nmatches<20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
            nmatches = matcher.SearchByProjectionGroupCamera(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
        }

        if(nmatches<20)
            return false;

        // Optimize frame pose with all matches
        Optimizer::PoseOptimizationGroupCamera(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

        if(mbOnlyTracking)
        {
            mbVO = nmatchesMap<10;
            return nmatches>20;
        }

        return nmatchesMap>=10;
    }
    bool Tracking::TrackReferenceKeyFrameGroupCamera()
    {
        // Compute Bag of Words vector
        mCurrentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7,true);
        vector<MapPoint*> vpMapPointMatches;

        int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
        //std::cout << "TrackReferenceKeyFrame SearchByBoW nmatches: " << nmatches << std::endl;
        //mCurrentFrame.SetPose(mLastFrame.mTcw);//edit-by-wx 2016-12-09 If tracking on reference frame is lost, we still want to try to track on local map. Then the pose must be set.

        mCurrentFrame.SetPose(mLastFrame.mTcw);
        if(nmatches<15)//wx-2016-12-09 original value is 15
            return false;

        mCurrentFrame.mvpMapPoints = vpMapPointMatches;


        Optimizer::PoseOptimizationGroupCamera(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }
        //std::cout << "TrackReferenceKeyFrame after optimization" << nmatchesMap << std::endl;
        return nmatchesMap>=10;
    }


    bool Tracking::TrackLocalMapGroupCamera()
    {
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.

        UpdateLocalMap();

        SearchLocalPointsGroupCamera();

        // Optimize Pose
        Optimizer::PoseOptimizationGroupCamera(&mCurrentFrame);
        //std::cout << "result " <<  << std::endl;
        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if(!mbOnlyTracking)
                    {
                        if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                            mnMatchesInliers++;
                    }
                    else
                        mnMatchesInliers++;
                }
                else if(mSensor==System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)//original value is 50
            return false;

        if(mnMatchesInliers<15)//wx-parameter-adjust 2016-12-31 original value is 30
            return false;
        else
            return true;
    }

}
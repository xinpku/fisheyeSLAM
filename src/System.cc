/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>



namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
	//try to load from the binary file
	bool bVocLoad = mpVocabulary->loadFromBinFile(strVocFile + ".bin");
	if (!bVocLoad)
	{
		cerr << "Cannot find binary file for vocabulary. " << endl;
		cerr << "Falied to open at: " << strVocFile + ".bin" << endl;
		cerr << "Trying to open the text file. " << endl;
		bool bVocLoad2 = mpVocabulary->loadFromTextFile(strVocFile);
		if (!bVocLoad2)
		{
			cerr << "Wrong path to vocabulary. " << endl;
			cerr << "Falied to open at: " << strVocFile << endl;
			exit(-1);
		}
		cerr << "Saving the vocabulary to binary for the next time to " << strVocFile + ".bin" << endl;
		mpVocabulary->saveToBinFile(strVocFile + ".bin");
	}

    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);
    mpSemanticMap = new SemanticMap();
    mpMap->mSemanticMap = mpSemanticMap;
    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}


cv::Mat System::TrackFisheye(const cv::Mat &imFisheyeGray, const cv::Mat &object_class,
                             const double &timestamp, std::vector<FisheyeCorrector> &correctors)
{
	if (mSensor != FISHEYE)
	{
		cerr << "ERROR: you called TrackStereo but input sensor was not set to FISHEYE." << endl;
		exit(-1);
	}

	// Check mode change
	{
		unique_lock<mutex> lock(mMutexMode);
		if (mbActivateLocalizationMode)
		{
			mpLocalMapper->RequestStop();

			// Wait until Local Mapping has effectively stopped
			while (!mpLocalMapper->isStopped())
			{
				usleep(1000);
			}

			mpTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		if (mbDeactivateLocalizationMode)
		{
			mpTracker->InformOnlyTracking(false);
			mpLocalMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}

	// Check reset
	{
		unique_lock<mutex> lock(mMutexReset);
		if (mbReset)
		{
			mpTracker->Reset();
			mbReset = false;
		}
	}

    std::vector<cv::Mat> imgs;
    for (int view = 0; view < correctors.size(); view++)
    {
        cv::Mat current_view;
        correctors[view].correct(imFisheyeGray,current_view);
        imgs.push_back(current_view);
        /*std::stringstream sst;
        sst << "view" << view;
        cv::imshow(sst.str(), current_view);*/
    }


	cv::Mat Tcw = mpTracker->GrabImageFisheye(imFisheyeGray, imgs, object_class, timestamp, correctors);

	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = mpTracker->mState;
	mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
	mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
	return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
    out.resize(plain.rows);

    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
    }
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
        mpViewer->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()  ||
          !mpViewer->isFinished()      || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }
		
        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryVtx(const string &filename)
{
	
	//cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

	vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
	if (vpKFs.size() == 0)
		return;
	sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

	// Transform all keyframes so that the first keyframe is at the origin.
	// After a loop closure the first keyframe might not be at the origin.
	cv::Mat Two = vpKFs[0]->GetPoseInverse();

	ofstream f;
	f.open(filename.c_str());
	f << fixed;

	// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
	// We need to get first the keyframe pose and then concatenate the relative transformation.
	// Frames not localized (tracking failure) are not saved.

	// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
	// which is true when tracking failed (lbL).
	list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
	list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
	list<bool>::iterator lbL = mpTracker->mlbLost.begin();
	for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
		lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++)
	{
		if (*lbL)
			continue;

		KeyFrame* pKF = *lRit;

		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

		// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
		while (pKF->isBad())
		{
			Trw = Trw*pKF->mTcp;
			pKF = pKF->GetParent();
		}

		Trw = Trw*pKF->GetPose()*Two;

		cv::Mat Tcw = (*lit)*Trw;
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

		vector<float> q = Converter::toQuaternion(Rwc);

		f << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2)  << endl;
	}
	f.close();
	//cout << endl << "trajectory saved!" << endl;
}
int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

void System::SaveMapClouds(const string &filename)
{
	std::ofstream cloud_file(filename);
	const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();

	for (int i = 0; i<vpMPs.size(); i++)
	{
		cv::Mat pos = vpMPs[i]->GetWorldPos();
		cloud_file << pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2) << std::endl;
	}
}

#define WEIGHT_THRESH  150
    std::vector<KeyFrame*> selectFrame(std::vector<KeyFrame*> vpKFs)
    {
        std::sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);  //vpKFs has already been sorted
        std::set<KeyFrame*> spSavedKFs;

//////////////    sparsely chose keyframes from all the keyframes ////////////////
//	std::vector<KeyFrame*>::iterator it_i = vpKFs.begin(),it_j,it_end;
        size_t it_i = 0 ,it_j = 0, it_end = vpKFs.size();
        KeyFrame* pKFi = vpKFs[it_i++],
                *pKFj;
        spSavedKFs.insert(pKFi);
        while( it_i < it_end )
        {
            pKFj = vpKFs[it_i]; //*it_i;
            if (pKFi->isBad()) {
                if (pKFj != nullptr)
                    spSavedKFs.insert(pKFj);
                pKFi = pKFj;
                it_i++;
                if (it_i == it_end)
                    break;
                continue;
            }

            std::vector<KeyFrame *> vpKFi_j;
            int w_max = pKFi->GetWeight(pKFj);
            int w_thresh = std::max(w_max / 4, WEIGHT_THRESH);
            if (w_max >= WEIGHT_THRESH)
                vpKFi_j.push_back(pKFj);
            it_j = it_i;
            it_j++;
            if (it_j == it_end) {
                if (pKFj != nullptr)
                    spSavedKFs.insert(pKFj);
                it_i = it_j;
                //break;
                continue;
            }
            while (it_j < it_end && pKFi->GetWeight(vpKFs[it_j]) > w_thresh) {
                vpKFi_j.push_back(vpKFs[it_j++]);
            }

//		cout << "prev KF id: " << pKFi->mnId << " , current: " << pKFj->mnId
//		     << ", forward KFs: " << vpKFi_j.size() << endl;

            // delete nearest KFs which have strong overlap with current one in spKFs
            if (vpKFi_j.empty() || w_max < WEIGHT_THRESH)
            {
                if (pKFj != nullptr)
                    spSavedKFs.insert(pKFj);
                vpKFi_j.clear();
                pKFi = pKFj;
                if (++it_i == it_end)
                    break;
            }
            else
            {
                unsigned j = vpKFi_j.size();
                pKFj = vpKFi_j[j - 1];
                spSavedKFs.insert(pKFj);
                for (unsigned k = 0; k < j - 1; k++) {
                    std::set<MapPoint *> spMapPoint = vpKFi_j[k]->GetMapPoints();
                    std::set<MapPoint *>::iterator sit = spMapPoint.begin();
                    for (; sit != spMapPoint.end(); sit++) {
                        vpKFi_j[k]->EraseMapPointMatch(*sit);
                    }
                    pKFi->EraseConnection(vpKFi_j[k]);
                    pKFi->EraseChild(vpKFi_j[k]);
                    //mpKeyFrameDatabase->erase(vpKFi_j[k]);
                }
                pKFi->UpdateConnections();
                it_i = it_j;
                pKFi = pKFj;
                it_i++;
            }
        }

        return std::vector<KeyFrame*>(spSavedKFs.begin(),spSavedKFs.end());

    }


    void System::SaveKeyframes(const std::vector<KeyFrame*> keyframes_list,const string &filename)
    {
        std::vector<KeyFrame*> keyframes = selectFrame(keyframes_list);

        std::ofstream out(filename, std::ios_base::binary);
        if (!out)
        {
            cerr << "Cannot Write to Mapfile: " << filename << std::endl;
            exit(-1);
        }
        cout << "Saving Mapfile: " << filename << std::flush;
        boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        oa << keyframes;
        cout << " ...done" << std::endl;
        out.close();


    }

    void System::SaveMapPoints(const std::vector<MapPoint*> mapPoints,const string &filename)
    {
        //std::vector<KeyFrame*> keyframes = selectFrame(keyframes_list);

        std::ofstream out(filename, std::ios_base::binary);
        if (!out)
        {
            cerr << "Cannot Write to Mapfile: " << filename << std::endl;
            exit(-1);
        }
        cout << "Saving Mapfile: " << filename << std::flush;
        boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        for(auto m:mapPoints)
        {
            oa << *m;
        }

        cout << " ...done" << std::endl;
        out.close();


    }

    void System::LoadMap(const string &filename)
    {
        std::ifstream in(filename, std::ios_base::binary);
        if (!in)
        {
            cerr << "Cannot Open Mapfile: " << filename << " , Create a new one" << std::endl;
            return;
        }
        cout << "Loading Mapfile: " << filename << std::flush;

        std::vector<KeyFrame*> keyframes;

        boost::archive::binary_iarchive ia(in, boost::archive::no_header);
        ia >> keyframes;

        for(int i= 0;i<keyframes.size();i++)
        {
            addKeyFrame(keyframes[i]);
        }

        auto keyframesInMap = mpMap->GetAllKeyFrames();
        for(int i= 0;i<keyframesInMap.size();i++)
        {
            keyframesInMap[i]->UpdateConnections();
        }

        auto mappointsInMap = mpMap->GetAllMapPoints();
        for(int i = 0;i<mappointsInMap.size();i++)
        {
            mappointsInMap[i]->ComputeDistinctiveDescriptors();
            mappointsInMap[i]->UpdateNormalAndDepth();
            //std::cout<<"point pose "<<mappointsInMap[i]->GetWorldPos().t()<<std::endl;
        }
        cout << " ...done" << std::endl;
        cout << "Map Reconstructing" << flush;
        cout << " ...done" << endl;
        in.close();
    }


    void System::addKeyFrame(KeyFrame* keyframe_loaded)
    {
        const int img_height = 2000;
        const int img_width = 2000;
        static float time_stamp = 0;
        time_stamp+=0.03;
        Frame frame(cv::Size(img_width,img_height),keyframe_loaded->GetPose(),keyframe_loaded->mvKeys,keyframe_loaded->mvKeysUn,keyframe_loaded->mDescriptors,time_stamp,mpTracker->mpORBextractorLeft,mpVocabulary,mpTracker->mK,mpTracker->mbf,mpTracker->mThDepth);


        KeyFrame* keyframe = new KeyFrame(frame,mpMap,mpKeyFrameDatabase);
        keyframe->mBowVec = keyframe_loaded->mBowVec;
        keyframe->SetPose(keyframe_loaded->GetPose());
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(keyframe->mDescriptors);
        DBoW2::BowVector bowVec;
        keyframe->mpORBvocabulary->transform(vCurrentDesc,bowVec,keyframe->mFeatVec,4);

        mpMap->AddKeyFrame(keyframe);
        std::vector<MapPoint*> MapPoints = keyframe_loaded->mvpMapPoints;
        for(int i=  0;i<MapPoints.size();i++)
        {
            MapPoint* pMP = new MapPoint(MapPoints[i]->mWorldPos,keyframe,mpMap);

            keyframe->AddMapPoint(pMP,i);

            pMP->AddObservation(keyframe,i);

            //Add to Map
            mpMap->AddMapPoint(pMP);
        }
        mpKeyFrameDatabase->add(keyframe);
    }

    void System::createVocabulary()
    {
        std::vector<KeyFrame*> keyframes = mpMap->GetAllKeyFrames();
        std::vector<std::vector<cv::Mat>> keyframe_features;
        for(int i = 0;i<keyframes.size();i++)
        {
            keyframe_features.push_back(vector<cv::Mat >());
            changeStructure(keyframes[i]->mDescriptors, keyframe_features.back());
        }

        const int k = 10;
        const int L = 3;
        const DBoW2::WeightingType weight = DBoW2::TF_IDF;
        const DBoW2::ScoringType score = DBoW2::L1_NORM;

        ORBVocabulary vocabularyKeyFrame(k, L, weight, score);

        std::cout<<"creating vocabulary of keyframes"<<std::endl;
        vocabularyKeyFrame.create(keyframe_features);
        std::cout<<"saving vocabulary of keyframes"<<std::endl;
        vocabularyKeyFrame.saveToBinFile("keyframeVoc.bin");
    }



} //namespace ORB_SLAM

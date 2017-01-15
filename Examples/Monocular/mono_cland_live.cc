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

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <opencv2/core.hpp>

#include "System.h"
#include "mavlink/common/mavlink.h"

#include "xiApiPlusOcv.h"

#define HandleResult(res,place) if (res!=XI_OK) {printf("Error after %s (%d)\n",place,res);}

using namespace std;


void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{


//	cv::VideoCapture cap(1); // open the default camera
//	    if(!cap.isOpened())  // check if we succeeded
//	        return -1;
////	    cap.set(CV_CAP_PROP_FRAME_WIDTH, static_cast<double>(1280));
////	    cap.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<double>(960));
//
//	    cv::Mat edges;
//	    cv::namedWindow("edges",1);
//	    for(;;)
//	    {
//	        cv::Mat frame;
//	        cap >> frame; // get a new frame from camera
////	        cv::cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
////	        GaussianBlur(edges, edges, cv::Size(7,7), 1.5, 1.5);
////	        Canny(edges, edges, 0, 30, 3);
//	        imshow("edges", frame);
//	        if(cv::waitKey(30) >= 0) break;
//	    }
	/// ximea camera code
	// Sample for XIMEA OpenCV
	xiAPIplusCameraOcv cam;
	// Retrieving a handle to the camera device
	printf("Opening first camera...\n");


	cam.OpenFirst();
	cam.SetAutoExposureAutoGainExposurePriority(0.5);
//	cam.SetDownsamplingType(XI_BINNING);
//	cam.SetDownsampling(XI_DWN_1x1);
	//Set exposure
//	cam.SetExposureTime(10000); //10000 us = 10 ms
	// Note: The default parameters of each camera might be different in different API versions

	printf("Starting acquisition...\n");
	cam.StartAcquisition();



	if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_cland path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;

    cv::Mat mIFrameTransRot;
    int trackingStatus = -1;
    bool cameraHasJumped = false;

    cv::Mat cameraPose = cv::Mat::eye(4,4,CV_32F);
    cv::Mat cameraRotation = cv::Mat::eye(3,3,CV_32F);
    cv::Mat cameraTranslation(1,3,CV_32F);




    for(int ni=0; ni<nImages; ni++)
    {
		cv::Mat im = cam.GetNextImageOcvMat();
        // grab the image from the camera



//		cvWaitKey(500);// Read image from file
//        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);

        // Supply optional inter-frame rotation and translation
        // TODO: pull this info from the Autopilot EKF and produce the matrix
        mIFrameTransRot = cv::Mat(); //cv::Mat::eye(4,4,CV_32F);
        // if we set the mIFrameTransRot to something other than empty it will be used for mVelocity in Tracking.cc.
        // we need to set it to be the rotation and translation between this frame and the last in camera world coordinate system.
        // TODO: ask about the best method to do this step
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system


        cameraPose = SLAM.TrackMonocular(im,tframe,mIFrameTransRot);
        trackingStatus = SLAM.GetStatus();
        cameraHasJumped = SLAM.CameraHasJumped();

        if (!cameraPose.empty())
        {
//        	cout << "cameraPose: " << cameraPose << endl << endl;
        	cameraPose.rowRange(0,3).colRange(0,3).copyTo(cameraRotation.rowRange(0,3).colRange(0,3));
        	//extract rotation matrix (first three col and rows)
//        	cout << "cameraRotation: " << cameraRotation << endl << endl;
        	cameraPose.rowRange(0,3).col(3).copyTo(cameraTranslation);
        	//extract translation (last col, first three rows)
        	cout << "cameraTranslation: " << cameraTranslation << endl << endl;
        	cout << "trackingStatus: " << trackingStatus << endl << endl;
        	cout << "cameraHasJumped: " << cameraHasJumped << endl << endl;
        	//TODO: convert to NED coordinate system and then a fake gps msg
        	//TODO: send back to Autopilot for EKF fusion
        }


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    cam.StopAcquisition();
	cam.Close();
	printf("Done\n");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}

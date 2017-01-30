/*
 * Wrapper.h
 *
 *  Created on: 29th Jan 2017
 *      Author: Samuel Dudley
 */
#ifndef WRAPPER_H
#define WRAPPER_H


#include<iostream>
#include<string>
#include<sstream>
#include<algorithm>
#include<fstream>
#include<vector>
#include<chrono>
#include<iomanip>
#include<opencv2/core.hpp>
#include<Converter.h>
#include<aruco.h>

#include<System.h>
#include<boost/python.hpp>
#include<xiApiPlusOcv.h>

class Wrapper
{
	public:
		void configure();
		void initialize();
		void shutdown();
		void track();
		int  getStatus();
		void reset();
		void getCurrentFrame();


	public:
		std::string msg;
		std::string vocabularyFilePath;
		std::string configurationFilePath;

		xiAPIplusCameraOcv cam;
		cv::Mat src;
		cv::Mat im;

		cv::Mat currentFrame; //im with tracking visualization drawn on it
		cv::FileStorage fsConfiguration;

		const double tframe = 0.1;

		cv::Mat autopilotPoseCurrent = cv::Mat::eye(4,4,CV_32F);

		cv::Mat cameraMatrix;
		cv::Mat distorsionCoeff;

		aruco::CameraParameters cameraParameters;
		aruco::MarkerDetector markerDetector;
		std::map<uint32_t,aruco::MarkerPoseTracker> markerTracker; // use a map so that for each id, we use a different pose tracker
		vector<aruco::Marker> markers;
		float markerSize;


	private:
		ORB_SLAM2::System* SLAM;


};


#endif // WRAPPER_H

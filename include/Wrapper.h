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

//ORB_SLAM2::System SLAM("./../Vocabulary/ORBvoc.bin","./../Examples/Monocular/XIMEA.yaml",ORB_SLAM2::System::MONOCULAR,true);
// Create SLAM system. It initializes all system threads and gets ready to process frames.
//ORB_SLAM2::System SLAM;


class Wrapper
{
	public:
		void set(std::string msg);
		std::string greet();
		void configure();
		void shutdown();
		void track();

	public:
//		ORB_SLAM2::System* SLAM;
		std::string msg;

		xiAPIplusCameraOcv cam;
		cv::Mat src;
		cv::Mat im;

		const double tframe = 0.1;
		vector<aruco::Marker> Markers;
		cv::Mat autopilotPoseCurrent = cv::Mat::eye(4,4,CV_32F);


};


#endif // WRAPPER_H

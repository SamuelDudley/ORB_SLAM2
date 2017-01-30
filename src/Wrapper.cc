/*
 * Wrapper.cc
 *
 *  Created on: 29th Jan. 2017
 *      Author: Samuel Dudley
 *      Brief: Python wrapper for SLAM system
 */

#include <Wrapper.h>

using namespace std;
using namespace aruco;
namespace python = boost::python;


struct MarkerData
{
	MarkerData() :
    name(), dictionary(), id(), ssize(-1)
	{
	}

    string name;
    string dictionary;
	int id;
	float ssize;


  	void read(const cv::FileNode& node)  //Read serialization for this class
	{
  		name = (string)node["name"];
  		dictionary = (string)node["dictionary"];
  		id = (int)node["id"];
  		ssize = (float)node["size"];
	}
};

static void read(const cv::FileNode& node, MarkerData& x, const MarkerData& default_value = MarkerData()){
	if(node.empty())
		x = default_value;
	else
		x.read(node);
}

void Wrapper::configure(string strVocFile, string strConfigFile)
{
	configurationFilePath = strConfigFile; // "./../Examples/Monocular/XIMEA.yaml";
	vocabularyFilePath = strVocFile; // "./../Vocabulary/ORBvoc.bin";



	//Check configuration file
	cv::FileStorage fsConfiguration(configurationFilePath.c_str(), cv::FileStorage::READ);
	if(!fsConfiguration.isOpened())
	{
	   cerr << "Failed to open configuration file at: " << configurationFilePath << endl;
	   exit(-1);
	}

//	http://stackoverflow.com/questions/7940216/parsing-nested-yml-file-at-second-node
	cv::FileNode marker_info = fsConfiguration["Marker.data"];
	cout << "attempting to read marker data" << endl;  //Show default behavior for empty matrix

	for(cv::FileNodeIterator fit = marker_info.begin(); fit != marker_info.end(); ++fit)
	{
		string name = (string)(*fit)["name"];
		string dictionary = (string)(*fit)["dictionary"];
		int id = (int)(*fit)["id"];
		float ssize = (float)(*fit)["size"];

		MarkerData m;
		fit >> m;
//
		cout << id << dictionary << endl;
		cout << m.id << m.ssize << endl;
//		cout << someval << endl;
	}



	// arUco config
    cameraMatrix = (cv::Mat_<float>(3,3) << fsConfiguration["Camera.fx"]   , 0, fsConfiguration["Camera.cx"],
											0, fsConfiguration["Camera.fy"],    fsConfiguration["Camera.cy"],
										    0,                               0,                           1);

    distorsionCoeff  = (cv::Mat_<float>(4,1) << fsConfiguration["Camera.k1"], fsConfiguration["Camera.k2"], fsConfiguration["Camera.p1"], fsConfiguration["Camera.p2"]);

    cameraParameters.setParams(cameraMatrix, distorsionCoeff, cv::Size(fsConfiguration["Camera.width"],fsConfiguration["Camera.height"]));

    markerSize = fsConfiguration["Marker.size"];

    markerDetector.setThresholdParams(7, 7);
    markerDetector.setThresholdParamRange(2, 0);
    markerDetector.setDictionary(fsConfiguration["Marker.dictionary"], 0.f);

}

void Wrapper::initialize()
{
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	SLAM = new ORB_SLAM2::System(vocabularyFilePath, configurationFilePath, ORB_SLAM2::System::MONOCULAR, true);
	// XIMEA OpenCV
	cam.OpenFirst();
	cam.StartAcquisition();
	cam.EnableAutoExposureAutoGain();

//	cam.EnableHDR();
//	cam.EnableWhiteBalanceAuto();

	cam.SetAutoExposureAutoGainExposurePriority(0.8f);

}

void Wrapper::shutdown()
{
	SLAM->Shutdown();
	exit(1);
}

void Wrapper::track()
{
	src = cam.GetNextImageOcvMat();
	cv::resize(src, im, cv::Size(1024,1024), 0, 0, cv::INTER_CUBIC); // resize image from camera (ideally we would be doing binning on the camera here)
	cout << "FPS = " << cam.GetFrameRate() << endl;

	// Detect markers
	markers = markerDetector.detect(im);
	// Do pose estimation for each marker
	for(auto & marker:markers) {
		markerTracker[marker.id].estimatePose(marker,cameraParameters,markerSize);
	}


	SLAM->TrackMonocular(im, tframe, autopilotPoseCurrent, autopilotPoseCurrent, markers);
}

int Wrapper::getStatus()
{
	cout << "status: " << SLAM->GetStatus() << endl;
	return SLAM->GetStatus();
}

void Wrapper::getCurrentFrame()
{
	currentFrame = SLAM->GetFrameDrawer()->DrawFrame();
	// return currentFrame;
}

void Wrapper::reset()
{
	SLAM->Reset();
}

BOOST_PYTHON_MODULE(libSLAM)
{
	using namespace python;
	class_<Wrapper>("Wrapper")
		.def("configure", &Wrapper::configure)
		.def("getStatus", &Wrapper::getStatus)
		.def("shutdown", &Wrapper::shutdown)
		.def("track", &Wrapper::track)
		.def("reset", &Wrapper::reset)
		.def("initialize", &Wrapper::initialize)
		.def("getCurrentFrame", &Wrapper::getCurrentFrame)
	;
}


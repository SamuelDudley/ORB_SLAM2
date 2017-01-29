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

extern ORB_SLAM2::System tmp("./../Vocabulary/ORBvoc.bin","./../Examples/Monocular/XIMEA.yaml",ORB_SLAM2::System::MONOCULAR,true);

void Wrapper::configure()
{
	// Create SLAM system. It initializes all system threads and gets ready to process frames.

//	SLAM = &tmp;


	// XIMEA OpenCV
	cam.OpenFirst();
//		cam.SetDownsamplingType(XI_SKIPPING); //XI_BINNING
//		cam.SetDownsampling(XI_DWN_2x2);
	cam.StartAcquisition();
	cam.EnableAutoExposureAutoGain();

//		cam.EnableHDR();
//		cam.EnableWhiteBalanceAuto();

	cam.SetAutoExposureAutoGainExposurePriority(0.8f);
}

void Wrapper::set(std::string msg)
{
	this->msg = msg;
}

std::string Wrapper::greet()
{
	return msg;
}

void Wrapper::shutdown()
{
	cout << "status: " << tmp.GetStatus() << endl;
//	cout << "status: "<< endl;
}

void Wrapper::track()
{
	src = cam.GetNextImageOcvMat();
	cv::resize(src, im, cv::Size(1024, 1024), 0, 0, cv::INTER_CUBIC); // resize image resolution
	cout << "FPS = " << cam.GetFrameRate() << endl;

	tmp.TrackMonocular(im, tframe, autopilotPoseCurrent, autopilotPoseCurrent, Markers);
}

BOOST_PYTHON_MODULE(libSLAM)
{
	using namespace python;
	class_<Wrapper>("Wrapper")
		.def("greet", &Wrapper::greet)
		.def("set", &Wrapper::set)
		.def("configure", &Wrapper::configure)
		.def("shutdown", &Wrapper::shutdown)
		.def("track", &Wrapper::track)
	;
}


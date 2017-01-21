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


#include<System.h>
//#include<mavlink/common/mavlink.h>

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
        vector<double> &vTimestamps);

void LoadArdupilotData(const string &strPathToSequence, vector<vector<float>> &vAttitudes,
		vector<vector<float>> &vNEDPPositions);

bool ExtractCurrentPosRotation(ORB_SLAM2::System &SLAM, vector<float> &pos,
		vector<float> &euler);

cv::Mat getRotationMatrix(vector<float> &euler);

template<typename T>
   T StringToNumber(const std::string& numberAsString)
   {
      T valor;

      std::stringstream stream(numberAsString);
      stream >> valor;
      if (stream.fail()) {
         std::runtime_error e(numberAsString);
//         throw e;

      }
      return valor;
   }

const char* wsab = " \t\n\r\f\v";

// trim from end of string (right)
inline std::string& rtrim(std::string& s, const char* t = wsab)
{
    s.erase(s.find_last_not_of(t) + 1);
    return s;
}

// trim from beginning of string (left)
inline std::string& ltrim(std::string& s, const char* t = wsab)
{
    s.erase(0, s.find_first_not_of(t));
    return s;
}

// trim from both ends of string (left & right)
inline std::string& trim(std::string& s, const char* t = wsab)
{
    return ltrim(rtrim(s, t), t);
}


int main(int argc, char **argv)
{
    if(argc != 5)
    {
    	// add flatout launch argument... skips the wait in the main loop to run the process as fast as the hardware will allow

        cerr << endl << "Usage: ./mono_cland path_to_vocabulary path_to_settings path_to_sequence realtime" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    vector<vector<float> > vAttitudes;
    vector<vector<float> > vNEDPPositions;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
    LoadArdupilotData(string(argv[3]), vAttitudes, vNEDPPositions);

    bool realTime = 0;
    string runSpeed(argv[4]);
    if (runSpeed == "realtime")
    {
    	realTime = 1;
    }

    /* print contents of autopilot data files */
//    for (auto it = vAttitudes.begin(); it != vAttitudes.end(); ++it) {
//    	double yaw = it[0][0];
//		double pitch = it[0][1];
//		double roll = it[0][2];
//		cout << "roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw << endl;
//    }
//
//    for (auto it = vNEDPPositions.begin(); it != vNEDPPositions.end(); ++it) {
//    	double north = it[0][0];
//		double east = it[0][1];
//		double down = it[0][2];
//		cout << "north=" << north << ", east=" << east << ", down=" << down << endl;
//    }

    int nImages = vstrImageFilenames.size();
    int nAttitudeSamples = vAttitudes.size();
    int nPositionSamples = vNEDPPositions.size();


    if (nImages != nAttitudeSamples || nImages != nPositionSamples) {
    	// if the number of images does not match the number of attitude or position samples
    	return 1;
    }

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

    cv::Mat mAPPoseNED;

    static const float PI = 3.14159265;

    static const float radToDeg = 180.0/PI;
    static const float degToRad = PI/180.0;


    vector<float> eulerAxisAlignmentAutopilotToCamera= {0,0,180*degToRad}; // 180 degree rotation about Z axis
    cv::Mat mAxisAlignmentAutopilotToCamera = getRotationMatrix(eulerAxisAlignmentAutopilotToCamera);


    cv::Mat test_rot = (cv::Mat_<float>(3,3) << -1,0,0,
    											0,-1,0,
												0,0,1);

    cv::Mat cameraPose = cv::Mat::eye(4,4,CV_32F);


    cv::Mat cameraRotationCurrent = cv::Mat::eye(3,3,CV_32F);
    cv::Mat cameraTranslationCurrent(3,1,CV_32F);

    cv::Mat cameraRotationInitial;
    cv::Mat cameraTranslationInitial;


    cv::Mat autopilotPoseCurrent = cv::Mat::eye(4,4,CV_32F);
    cv::Mat autopilotPoseCurrentCameraFrame = cv::Mat::eye(4,4,CV_32F);
    cv::Mat autopilotRotationCurrent = cv::Mat::eye(3,3,CV_32F);
    cv::Mat autopilotTranslationCurrent(3,1,CV_32F);

    cv::Mat autopilotTranslationInitial(3,1,CV_32F);
    cv::Mat autopilotRotationInitial = cv::Mat::eye(3,3,CV_32F);

    float ratioAPToCameraScale;

    vector<float> eulerAutopilotCurrent(3);

    vector<float> pos(3);
    vector<float> euler(3);

    vector<float> eulerb(3);



    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        // grab the image from the camera

        // Supply optional inter-frame rotation and translation
        // TODO: pull this info from the Autopilot EKF and produce the matrix
        mIFrameTransRot = cv::Mat(); //cv::Mat::eye(4,4,CV_32F);
        // if we set the mIFrameTransRot to something other than empty it will be used for mVelocity in Tracking.cc.
        // we need to set it to be the rotation and translation between this frame and the last in camera world coordinate system.
        // TODO: ask about the best method to do this step
        double tframe = vTimestamps[ni];
        float currentYaw = vAttitudes[ni][0];
        float currentPitch = vAttitudes[ni][1];
        float currentRoll = vAttitudes[ni][2];
        float currentNorth = vNEDPPositions[ni][0];
        float currentEast = vNEDPPositions[ni][1];
        float currentDown = vNEDPPositions[ni][2];

        cout << "time=" << tframe << endl;
//        cout << "AP roll 1      =" << currentRoll*radToDeg << ", pitch=" << currentPitch*radToDeg << ", yaw=" << currentYaw*radToDeg << endl;

        eulerAutopilotCurrent = {currentRoll, currentPitch, currentYaw}; // we apply negitive values as the orb world & camera frame use -ve rotation conventions
        autopilotRotationCurrent = getRotationMatrix(eulerAutopilotCurrent); // note euler angles need to be in radians!


//        // this is a test of the rotation matrix math
        vector<float> a = ORB_SLAM2::Converter::toEuler(ORB_SLAM2::Converter::toQuaternion(autopilotRotationCurrent));

//        cout << "AP north       =" << currentNorth << ", east=" << currentEast << ", down=" << currentDown << endl << endl;

		autopilotTranslationCurrent.at<float>(0,0) = currentNorth;
		autopilotTranslationCurrent.at<float>(1,0) = currentEast;
		autopilotTranslationCurrent.at<float>(2,0) = currentDown;

		// copy rotation matrix to first three col and rows of autopilot pose
		autopilotRotationCurrent.rowRange(0,3).colRange(0,3).copyTo(autopilotPoseCurrent.rowRange(0,3).colRange(0,3));

		autopilotPoseCurrent.at<float>(0,3) = currentNorth;
		autopilotPoseCurrent.at<float>(1,3) = currentEast;
		autopilotPoseCurrent.at<float>(2,3) = currentDown;

		mAPPoseNED = autopilotPoseCurrent.clone();

//		cout << "autopilot pose" << endl << autopilotPoseCurrent << endl;




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


        cameraPose = SLAM.TrackMonocular(im, tframe, mAPPoseNED, mIFrameTransRot);
        trackingStatus = SLAM.GetStatus();
        cameraHasJumped = SLAM.CameraHasJumped();

        if (!cameraPose.empty())
        {

        	cameraRotationCurrent = SLAM.GetTracker()->mCurrentFrame.GetRotation();
        	cameraTranslationCurrent = SLAM.GetTracker()->mCurrentFrame.GetCameraCenter();



        	if (cameraRotationInitial.empty())
        	{

        		cameraRotationInitial = cv::Mat::eye(3,3,CV_32F); // the first keyframe always has no rotation
        		cameraTranslationInitial = cv::Mat(3,1,CV_32F,0.0); // the first keyframe always has no translation

        		cv::Mat autopilotPoseInitial = SLAM.GetTracker()->poseAPInitial;
        		autopilotPoseInitial.rowRange(0,3).colRange(0,3).copyTo(autopilotRotationInitial.rowRange(0,3).colRange(0,3));
//        		autopilotRotationInitial = autopilotRotationCurrent;

        		autopilotTranslationInitial.at<float>(0,0) = autopilotPoseInitial.at<float>(0,3);
        		autopilotTranslationInitial.at<float>(1,0) = autopilotPoseInitial.at<float>(1,3);
        		autopilotTranslationInitial.at<float>(2,0) = autopilotPoseInitial.at<float>(2,3);

        		ratioAPToCameraScale = SLAM.GetTracker()->ratioAPToCameraScale;

        		cout << "initial camera & autopilot pose set at time = " << tframe << endl << endl;
        		cout << "initial camera: " << endl << cameraRotationInitial << endl << cameraTranslationInitial << endl << endl;
        		cout << "initial autopilot: " << endl << autopilotRotationInitial << endl << autopilotTranslationInitial << endl << endl;
        		cout << "map scale factor: " << ratioAPToCameraScale << endl << endl;
        	}

        	/* ---Rotation--- */
        	// rotation ap to camera frame

        	cv::Mat autopilotInCameraFrameRotation2 = autopilotRotationInitial.t()*autopilotRotationCurrent;
        	cv::Mat autopilotInCameraFrameRotation3 =autopilotRotationInitial.t()*autopilotRotationCurrent;

        	vector<float> tmp101 = ORB_SLAM2::Converter::toEuler(ORB_SLAM2::Converter::toQuaternion(autopilotInCameraFrameRotation2.t()));
        	vector<float> tmp201= {-tmp101[0],-tmp101[1], tmp101[2]};
        	autopilotInCameraFrameRotation2 = getRotationMatrix(tmp201);

        	vector<float> tmp1 = ORB_SLAM2::Converter::toEuler(ORB_SLAM2::Converter::toQuaternion(autopilotInCameraFrameRotation3));
			vector<float> tmp2= {-tmp1[0],-tmp1[1],-tmp1[2]};

			cv::Mat autopilotInCameraFrameRotation = getRotationMatrix(tmp2);


        	// inverse of above, camera to ap frame
			vector<float> b = ORB_SLAM2::Converter::toEuler(ORB_SLAM2::Converter::toQuaternion(cameraRotationCurrent));
			vector<float> tmp3= {-b[0],-b[1],b[2]};

			cameraRotationCurrent = getRotationMatrix(tmp3);
        	cv::Mat cameraInAutopilotFrameRotation = cameraRotationCurrent*autopilotRotationInitial.t();

        	/* ---Translation--- */
        	// translation ap to orb world
        	cv::Mat translationOrbWorldFrameNED1 = autopilotRotationInitial.t()*(autopilotTranslationCurrent - autopilotTranslationInitial);
        	cv::Mat translationOrbWorldFrameNED = mAxisAlignmentAutopilotToCamera*translationOrbWorldFrameNED1.mul(1.0f/ratioAPToCameraScale);

        	// translation camera to ned world
        	cv::Mat translatioNEDFrameOrbWorld1 = -(autopilotRotationInitial*cameraTranslationCurrent);
        	cv::Mat translatioNEDFrameOrbWorld = translatioNEDFrameOrbWorld1.mul(ratioAPToCameraScale);


        	cv::Mat cameraGlobalToCameraFrame = SLAM.GetTracker()->mCurrentFrame.GetRotation()*cameraTranslationCurrent;
        	// translation ap to orb camera (values will change with the current orientation of the camera)
        	cv::Mat translationCameraFrameNED = -autopilotInCameraFrameRotation2*translationOrbWorldFrameNED;


        	//build the ap pose in the camera frame. we will use this as the velocity estimate
        	autopilotInCameraFrameRotation2.rowRange(0,3).colRange(0,3).copyTo(autopilotPoseCurrentCameraFrame.rowRange(0,3).colRange(0,3));
    		autopilotPoseCurrent.at<float>(0,3) = translationCameraFrameNED.at<float>(0,0); //(row, col)
    		autopilotPoseCurrent.at<float>(1,3) = translationCameraFrameNED.at<float>(1,0);
    		autopilotPoseCurrent.at<float>(2,3) = translationCameraFrameNED.at<float>(2,0);


        	// generate some euler angles to check what is going on
//        	vector<float> a = ORB_SLAM2::Converter::toEuler(ORB_SLAM2::Converter::toQuaternion(autopilotInCameraFrameRotation));
        	vector<float> c = ORB_SLAM2::Converter::toEuler(ORB_SLAM2::Converter::toQuaternion(autopilotInCameraFrameRotation2));

        	// camera estimate of AP roll, pitch, yaw in NED frame VVVV
        	vector<float> d = ORB_SLAM2::Converter::toEuler(ORB_SLAM2::Converter::toQuaternion(cameraInAutopilotFrameRotation.t()));

			cout << "Cam          =" << endl << cameraTranslationCurrent << endl << endl;
			cout << "AP est. Cam  =" << endl << translationOrbWorldFrameNED << endl << endl;
			cout << "AP           =" << endl << autopilotTranslationCurrent - autopilotTranslationInitial << endl << endl;
			cout << "Cam est. AP  =" << endl << translatioNEDFrameOrbWorld << endl << endl;

            cout << "AP        roll =" << a[0]*radToDeg << ", pitch=" << a[1]*radToDeg << ", yaw=" << a[2]*radToDeg << endl;
        	cout << "ORB in AP roll =" << d[0]*radToDeg << ", pitch=" << d[1]*radToDeg << ", yaw=" << d[2]*radToDeg << endl << endl;

        	cout << "AP in ORB roll =" << c[0]*radToDeg << ", pitch=" << c[1]*radToDeg << ", yaw=" << c[2]*radToDeg << endl;
        	cout << "ORB       roll =" << b[0]*radToDeg << ", pitch=" << b[1]*radToDeg << ", yaw=" << b[2]*radToDeg << endl << endl;

        	cout << "Cam pose in Cam frame: " << endl << cameraPose << endl << endl;
        	cout << "AP  pose in Cam frame: " << endl << autopilotPoseCurrent << endl << endl << endl << endl << endl;

//        	cout << "trackingStatus: " << trackingStatus << endl << endl;
//        	cout << "cameraHasJumped: " << cameraHasJumped << endl << endl;


        	getchar(); // if uncommented each frame will require a key press before continuing

        }


//        if (ExtractCurrentPosRotation(SLAM, pos, euler))
//        {
//        	cout << "x=" << pos[0] << ", y=" << pos[1] << ", z=" << pos[2] << ", ";
//        	cout << "roll=" << euler[0] << ", pitch=" << euler[1] << ", yaw=" << euler[2] << endl;
//        }


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

//        getchar(); // if uncommented each frame will require a key press before continuing

//        if (c == 0) {
//            switch(getch()) {
//                // special KEY_ handling here
//                case KEY_UP:
//                    break;
//            }
//        } else {
//            switch(c) {
//                // normal character handling
//                case 'a':
//                    break;
//            }
//         }


        if (realTime)
        {
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
    fTimes.close();

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

void LoadArdupilotData(const string &strPathToSequence, vector<vector<float> > &vAttitudes, vector<vector<float> > &vNEDPPositions)
{
	double PI = 3.14159265;
    ifstream fAttitudes;
    ifstream fNEDPPositions;
    string strPathAttitudeFile = strPathToSequence + "/att.txt";
    string strNEDPPositionFile = strPathToSequence + "/pos.txt";

    //open the attitude file for reading
    fAttitudes.open(strPathAttitudeFile.c_str());
    while(!fAttitudes.eof())
    {
        string line;
        getline(fAttitudes,line);
        line = trim(line);
        if(!line.empty())
        {
            stringstream ss(line);
            vector<float> attitude;
            do
            {
			   string sub;
			   ss >> sub; // parting each string in the line
			   if(!sub.empty())
			   {
				   float attitude_value = StringToNumber<float>(sub);
				   attitude.push_back(attitude_value * (PI/180.0));  //convert the value to radians before storing it
			   }
            }
            while (ss);
            vAttitudes.push_back(attitude);
        }
    }
    fAttitudes.close();



    //open the position file for reading
    fNEDPPositions.open(strNEDPPositionFile.c_str());
    while(!fNEDPPositions.eof())
    {
        string line;
        getline(fNEDPPositions,line);
        line = trim(line);
        if(!line.empty())
        {
            stringstream ss(line);
            vector<float> position;
            do
            {
			   string sub;
			   ss >> sub; // parting each string in the line
			   if(!sub.empty())
			   {
				   float position_value = StringToNumber<float>(sub);
				   position.push_back(position_value);
			   }
            }
            while (ss);
            vNEDPPositions.push_back(position);
        }
    }
    fNEDPPositions.close();
}

bool ExtractCurrentPosRotation(ORB_SLAM2::System &SLAM, vector<float> &pos, vector<float> &euler)
{
	cv::Mat curPos = SLAM.GetTracker()->mCurrentFrame.GetCameraCenter();
	cv::Mat curRotation = SLAM.GetTracker()->mCurrentFrame.GetRotation();

	if (!curPos.empty() && !curRotation.empty())
	{

		pos[0] = curPos.at<float>(0,0);
		pos[1] = curPos.at<float>(0,1);
		pos[2] = curPos.at<float>(0,2);
		vector<float> q = ORB_SLAM2::Converter::toQuaternion(curRotation);
		euler = ORB_SLAM2::Converter::toEuler(q);

		return true;
	}

	return false;

}

cv::Mat getRotationMatrix(vector<float> &euler)
{
	// euler values must be in radians!
	int x = 0;
	int y = 1;
	int z = 2;

	float cX = cos(euler[x]);
	float sX = sin(euler[x]);

	float cY = cos(euler[y]);
	float sY = sin(euler[y]);

	float cZ = cos(euler[z]);
	float sZ = sin(euler[z]);

//	cv::Mat rotationMatrixX = (cv::Mat_<float>(3,3) << 1, 0, 0,
//														0, cX, -sX,
//														0, sX, cX);
//
//	cv::Mat rotationMatrixY = (cv::Mat_<float>(3,3) << cY, 0, sY,
//													    0, 1, 0,
//													    -sY, 0, cY);
//
//	cv::Mat rotationMatrixZ = (cv::Mat_<float>(3,3) << cZ -sZ, 0,
//													    sZ, cZ, 0,
//													    0, 0, 1);
//
//	cv::Mat rotationMatrix = rotationMatrixZ*rotationMatrixY*rotationMatrixX;

//	cv::Mat rotationMatrix = (cv::Mat_<float>(3,3) << cY*cZ, cX*sZ+sX*sY*cZ, sX*sZ-cX*sY*cZ,
//													-1*cY*sZ, cX*cZ-sX*sY*sZ, sX*cZ+cX*sY*sZ,
//													sY,          -1*sX*cY,     cX*cY        );
	cv::Mat rotationMatrix = (cv::Mat_<float>(3,3) << cY*cZ, sX*sY*cZ-cX*sZ, cX*sY*cZ+sX*sZ,
													  cY*sZ, sX*sY*sZ+cX*cZ, cX*sY*sZ-sX*cZ,
													-1.0f*sY,          sX*cY,     cX*cY        );

	return rotationMatrix;
}



##Alternate Navigation System for ArduPilot
This project brings together SLAM and marker recognition

#### What?
#####it is trying to do...
* Allow an air vehicle to localise itself in large environments (e.g. outdoors) and operate in the absence of GPS

#####is it **not** trying to do (yet)...
* Perform navigation through cluttered, complex environments 

#### How?
This project is primarily leveraging two computer vision projects; [ORB-SLAM 2](https://github.com/raulmur/ORB_SLAM2) and [ArUco](https://www.uco.es/investiga/grupos/ava/node/26). ORB-SLAM is used to generate the local 'orb world' which the UAV navigates, while ArUco serves as a method to constrain scale, drift and tie the 'orb world' back to 'real world' e.g. lat, lon, alt. 

Feature based mapping with a graph based back end. Marker observations are used to add 'hard' edges to the graph. 

#### Why?
I'm personally interested in computer vision and unmanned systems. This project allows me to develop skills while contributing ideas back to the Ardupilot community. 

#### TODO
* Write notes on camera cal
* Write notes on loose Ardupilot EKF integration.  
//! \addtogroup 0013 Position_from_Video_using_Tags
//! \brief Apriltag recognition 
//!
//! Module to recognise AprilTags 
//! 
//! \file   VideoTag.hpp
//! \author Ole Agterberg, 1651981
//! \author Daniel Klomp, 1661521
//! \date   Created: 9-6-2016
//! \date   Last Modified: 16-6-2016
//! \brief  Function defines for VideoTag.cpp
//!
//! The implementation of all the functions te enable recognition of the video tag.
//! Most functions are a copy from the Demo from the AprilTags library.
//! Added functionality to wrap the apriltags libary in the adt sensor interface.
//! also made functions to calculate proper x and y distance from camera, and distance
//! from the tag position to the origin of the local coordinate system.
//! Info is linked to the tag in a config file.
//!
//! More info on this system can be found on the wiki page:
//! https://roborescue.nl/index.php/Position_from_Video_using_Tags
//!
//! \copyright Copyright © 2016, HU University of Applied Sciences Utrecht. 
//! All rights reserved.
//! 
//! License: newBSD
//!
//! Redistribution and use in source and binary forms, 
//! with or without modification, are permitted provided that 
//! the following conditions are met:
//! - Redistributions of source code must retain the above copyright notice, 
//!   this list of conditions and the following disclaimer.
//! - Redistributions in binary form must reproduce the above copyright notice, 
//!   this list of conditions and the following disclaimer in the documentation 
//!   and/or other materials provided with the distribution.
//! - Neither the name of the HU University of Applied Sciences Utrecht 
//!   nor the names of its contributors may be used to endorse or promote 
//!   products derived from this software without specific prior written 
//!   permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//! "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
//! BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
//! AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
//! IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
//! BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
//! PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
//! OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
//! WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
//! OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
//! EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ~< HEADER_VERSION 2016 04 12 >~

#ifndef _VIDEOTAG_HPP
#define _VIDEOTAG_HPP

#include <fstream>
#include <iostream>
#include <cstring>
#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <sys/time.h>
#include <cmath>
#include <thread>
#include <chrono>

#include "CoordinateAttitude.hpp"
#include "Sensor.hpp"
#include "Coordinate.hpp"
#include "Length.hpp"
#include "LockingSharedObject.hpp"


#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "TagDetector.h"
#include "Tag16h5.h"
#include "Tag25h7.h"
#include "Tag25h9.h"
#include "Tag36h9.h"
#include "Tag36h11.h"

// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

const char* windowName = "apriltags_demo";

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

class VideoTag : public r2d2::Sensor<r2d2::Coordinate>{
public:
	//! Constructor
	//! Param factor, the error of the sensor
	VideoTag(double factor);
	
	//! Returned the coordinates of the camera
	//! return SensorResult, with the coordinates of the camera
	SensorResult get_data() override;

	//! Loop function for checking video input for tags 
	//! And saves the current position in result
	//! return void
	void loop();
					  
private:
	//! Struct for tag info
	//! id, the id of the tag
	//! position, the coordinates and the attitude of the tag
	typedef struct tag_info{
		int id;
		r2d2::CoordinateAttitude position;
	}tag;

	//! list of tags
	vector<tag_info> tags;

	//! Reads the tags and it's information out of the file
	//! return void
	void read_tag_info();

	//! Function to return the current system time
	//! return current system time in seconds
	double tic();
	
	//! Normalize angle to be within the interval [-pi,pi]
	//! Param double t the degrees
	//! return angle in radial between -pi and pi
	double standardRad(double t);

	//! Convert rotation matrix to Euler angles
	//! Param double yaw the rotation of the z-as
	//! Param double pitch the rotation of the y-as
	//! Param double roll the roation of the x-as
	//! return void
	void wRo_to_euler( Eigen::Matrix3d& wRo, double& yaw, 
					  double& pitch, double& roll) ;

	//! Sets the TagCode type for detection
	//! param s string that contains the type of TagCode
	//! return void
	void setTagCodes(string s);

	//! Opends video input
	//! Sets the excposure, gain and brightness
	//! return void
	void setupVideo();

	//! Prints the detection
	//! Param detection, the TagDetections
	//! return void
	void print_detection(AprilTags::TagDetection& detection);

	//! Processed the image	and checked if there is a tag
	//! Param image, the image in color
	//! Param image_gray, the image in gray
	//! return vector with the tagdetections
	vector<AprilTags::TagDetection> processImage(cv::Mat& image, cv::Mat& image_gray);

	//! Calculates the position of the camera with the detected tags
	//! Param detection, the detected tags
	//! return r2d2::Coordinate with the position of the camera
	r2d2::Coordinate calculatePosition(AprilTags::TagDetection& detection);

 	AprilTags::TagCodes m_tagCodes = AprilTags::tagCodes36h11;
	AprilTags::TagDetector * m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

	bool m_draw = false; // draw image and April tag detections?
 	bool m_arduino = false; // send tag detections to serial port?
	bool m_timing = false; // print timing information for each tag extraction call

	int m_width = 320; // image size in pixels
	int m_height = 240;
	double m_tagSize = 0.172; // April tag side length in meters of square black frame
  	double m_fx = 335; // camera focal length in pixels
  	double m_fy = 335;
  	double m_px = m_width/2; // camera principal point
  	double m_py = m_height/2;

  	int m_deviceId = 0; // camera id (in case of multiple cameras)

  	list<string> m_imgNames;
  	cv::VideoCapture m_cap;

  	int m_exposure = -1 ;
  	int m_gain = -1 ;
  	int m_brightness = -1;
	r2d2::Coordinate cor = r2d2::Coordinate(0 * r2d2::Length::METER,
						0 * r2d2::Length::METER,
						0 * r2d2::Length::METER);

  	LockingSharedObject<r2d2::Coordinate> shared_coordinate;
	//SharedObject<r2d2::Coordinate>::Accessor acc1(shared_coordinate);


	SensorResult result = SensorResult(0.0, cor);
		
};
#endif
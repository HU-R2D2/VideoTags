#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <cmath>

#include "../../../sensorInterfaces/source/include/Sensor.hpp"
#include "../../../adt/source/include/Coordinate.hpp"

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
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

const char* windowName = "apriltags_demo";

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

// For Arduino: locally defined serial port access class
#include "Serial.h"

class VideoTag : public Sensor<r2d2::Coordinate>{
	public:
	VideoTag(
			double error
	);
	
	r2d2::Coordinate get_data() override;
	
	double tic();
	double standardRad(double t);
	void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, 
					  double& pitch, double& roll);
	void setTagCodes(string s);
	void setupVideo();
	const void print_detection(AprilTags::TagDetection& detection);
	void processImage(cv::Mat& image, cv::Mat& image_gray);
					  
private:
  AprilTags::TagDetector* m_tagDetector = NULL;
  AprilTags::TagCodes m_tagCodes = AprilTags::tagCodes36h11;

  bool m_draw = false; // draw image and April tag detections?
  bool m_arduino = false; // send tag detections to serial port?
  bool m_timing = false; // print timing information for each tag extraction call

  int m_width = 320; // image size in pixels
  int m_height = 240;
  double m_tagSize = 0.166; // April tag side length in meters of square black frame
  double m_fx = 600; // camera focal length in pixels
  double m_fy = 600;
  double m_px = m_width/2; // camera principal point
  double m_py = m_height/2;

  int m_deviceId = 0; // camera id (in case of multiple cameras)

  list<string> m_imgNames;
  cv::VideoCapture m_cap;

  int m_exposure = -1 ;
  int m_gain = -1 ;
  int m_brightness = -1;

  Serial m_serial;
  m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
	
	
}

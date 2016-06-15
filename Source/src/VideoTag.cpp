#include "../Include/VideoTag.hpp"

VideoTag::VideoTag(double factor):
	Sensor{ factor }
{
	setupVideo();
	read_tag_info();
	
	std::thread t(&VideoTag::loop, this);
	t.join();
	
}

void VideoTag::read_tag_info(){
	std::string line;
	std::ifstream tag_file("Tag_Info.txt", std::ifstream::in);
	if(tag_file.is_open()){
		cout<<"Loaded Tag_Infor.txt";
		while(getline(tag_file, line)){
			std::istringstream is(line);
			int id;
			double x, y, z, yaw, roll, pitch;
			is >> id >> x >> y >> z >> yaw >> roll >> pitch;
			r2d2::Coordinate cor(x * r2d2::Length::METER, 
					     y * r2d2::Length::METER, 
					     z * r2d2::Length::METER);
			r2d2::Attitude at(pitch * r2d2::Angle::rad,
					  yaw * r2d2::Angle::rad,
					  roll * r2d2::Angle::rad);
			tag_info tag;
			tag.id = id;
			tag.position = r2d2::CoordinateAttitude(cor, at);
			tags.push_back(tag);
			std::cout << "id: " << id << "x: " << x << std::endl;
		}
	}
	else{
		cout << "Could not load file!" << endl;
	}
	cout << "amount of tags: " << tags.size() << endl;
	for(int i = 0; i<tags.size(); i++){
		cout << tags[i].id << " " << tags[i].position << " " << endl;
	}
}

void VideoTag::loop(){
	cv::Mat image;
	cv::Mat image_gray;
	while(true){
    	// capture frame
      	m_cap >> image;
		vector<AprilTags::TagDetection> detections = processImage(image, image_gray);

		int detect_count = detections.size();
		if(detect_count > 0){
		    // get distance of tags;
			// and calculate average of de coordinates
			r2d2::Length x = 0 * r2d2::Length::METER, y = 0 * r2d2::Length::METER, z = 0 * r2d2::Length::METER;
			for (int i = 0; i < detect_count; i++) {
				r2d2::Coordinate temp = calculatePosition(detections[i]);
				x += temp.get_x();
				y += temp.get_y();
				z += temp.get_z();
                		print_detection(detections[i]);
			}
			x /= detect_count;
			y /= detect_count;
			z /= detect_count;
			r2d2::Coordinate average_coordinate(x, y, z);
			result = SensorResult(0.0, average_coordinate);
		}
	}
}

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double VideoTag::tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
double VideoTag::standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void VideoTag::wRo_to_euler( Eigen::Matrix3d& wRo, double& yaw, double& pitch, double & roll)  {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

  // changing the tag family
  void VideoTag::setTagCodes(string s) {
    if (s=="16h5") {
      m_tagCodes = AprilTags::tagCodes16h5;
    } else if (s=="25h7") {
      m_tagCodes = AprilTags::tagCodes25h7;
    } else if (s=="25h9") {
      m_tagCodes = AprilTags::tagCodes25h9;
    } else if (s=="36h9") {
      m_tagCodes = AprilTags::tagCodes36h9;
    } else if (s=="36h11") {
      m_tagCodes = AprilTags::tagCodes36h11;
    } else {
      cout << "Invalid tag family specified" << endl;
      exit(1);
    }
  }


  void VideoTag::setupVideo() {
#ifdef EXPOSURE_CONTROL
    // manually setting camera exposure settings; OpenCV/v4l1 doesn't
    // support exposure control; so here we manually use v4l2 before
    // opening the device via OpenCV; confirmed to work with Logitech
    // C270; try exposure=20, gain=100, brightness=150

    string video_str = "/dev/video0";
    video_str[10] = '0' + m_deviceId;
    int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

    if (m_exposure >= 0) {
      // not sure why, but v4l2_set_control() does not work for
      // V4L2_CID_EXPOSURE_AUTO...
      struct v4l2_control c;
      c.id = V4L2_CID_EXPOSURE_AUTO;
      c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
      if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
        cout << "Failed to set... " << strerror(errno) << endl;
      }
      cout << "exposure: " << m_exposure << endl;
      v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
    }
    if (m_gain >= 0) {
      cout << "gain: " << m_gain << endl;
      v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
    }
    if (m_brightness >= 0) {
      cout << "brightness: " << m_brightness << endl;
      v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
    }
    v4l2_close(device);
#endif

    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(m_deviceId);
        if(!m_cap.isOpened()) {
      cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
      exit(1);
    }
    m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
         << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
  }

  void VideoTag::print_detection(AprilTags::TagDetection& detection) {
    cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";

    // recovering the relative pose of a tag:

    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size
    // (m_fx, m_fy, m_px, m_py, m_tagSize)

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);

    Eigen::Matrix3d F;
    F <<
      1, 0,  0,
      0,  -1,  0,
      0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    cout << "  distance=" << translation.norm()
         << "m, x=" << translation(0)
         << ", y=" << translation(1)
         << ", z=" << translation(2)
         << ", yaw=" << yaw
         << ", pitch=" << pitch
         << ", roll=" << roll
         << endl;

    // Also note that for SLAM/multi-view application it is better to
    // use reprojection error of corner points, because the noise in
    // this relative pose is very non-Gaussian; see iSAM source code
    // for suitable factors.
  }

  r2d2::Coordinate VideoTag::calculatePosition(AprilTags::TagDetection& detection) {
	Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);

	int id = detection.id;

	r2d2::Coordinate tag_coordinate;
	// todo --> get coordinate of de tag with his ID
	r2d2::Length tag_x = 0 * r2d2::Length::METER;
	r2d2::Length tag_y = 0 * r2d2::Length::METER;
	r2d2::Length tag_z = 0 * r2d2::Length::METER;

	// distance from camera to tag
	r2d2::Length camera_x = translation(0) * r2d2::Length::METER;
	r2d2::Length camera_y = translation(1) * r2d2::Length::METER;
	r2d2::Length camera_z = translation(2) * r2d2::Length::METER;
	return r2d2::Coordinate(tag_x + camera_x, tag_y + camera_y, tag_z + camera_y);
}

  vector<AprilTags::TagDetection> VideoTag::processImage(cv::Mat& image, cv::Mat& image_gray) {
    // alternative way is to grab, then retrieve; allows for
    // multiple grab when processing below frame rate - v4l keeps a
    // number of frames buffered, which can lead to significant lag
    m_cap.grab();
    m_cap.retrieve(image);

    // detect April tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);

    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);

    // print out each detection
    cout << detections.size() << " tags detected:" << endl;
    for (int i=0; i<detections.size(); i++) {
      print_detection(detections[i]);
    }

	return detections;
 }

  // The processing loop where images are retrieved, tags detected,
  // and information about detections generated
  VideoTag::SensorResult VideoTag::get_data() {
	return result;
  }

// here is were everything begins
int main(int argc, char* argv[]) {
	
	VideoTag tag (0.0);
    	// the actual processing loop where tags are detected and visualized
    	
	
	
	tag.get_data();
  return 0;
}

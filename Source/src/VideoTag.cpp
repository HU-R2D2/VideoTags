#include "../Include/VideoTag.hpp"

VideoTag::VideoTag(double factor):
	Sensor{ factor },
	shared_coordinate(cor)
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
		cout<<"Loaded Tag_Info.txt";
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
		}
	}
	else {
		cout << "Could not load file!" << endl;
	}
	cout << "amount of tags: " << tags.size() << endl;
	for(int i = 0; i<tags.size(); i++){
		std::cout << tags[i].id << ", ";
	}
	std::cout << std::endl;
}

void VideoTag::loop(){
	cv::Mat image;
	cv::Mat image_gray;
	while(true){
    	// capture frame
        cout<<"Scanning...";
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
			SharedObject<r2d2::Coordinate>::Accessor acc(shared_coordinate);
			acc.access() = average_coordinate;
		}
	std::this_thread::sleep_for (std::chrono::seconds(1));
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

	Eigen::Matrix3d F;
    	F <<
      	    1,  0,   0,
      	    0,  -1,  0,
      	    0,  0,   1;
    	Eigen::Matrix3d fixed_rot = F*rotation;
    	double yaw, pitch, roll;
    	wRo_to_euler(fixed_rot, yaw, pitch, roll);	
		
	// get tag information
	int id = detection.id;
	r2d2::Coordinate tag_coordinate;
	tag_info detected_tag;
	for (int i = 0; i < tags.size(); i++){
		if(tags[i].id == id){
			detected_tag = tags[i];
		}
	}

	// distance between the tag and the camera
	double distance = translation.norm();
      
	// tag_cor, the coordinate of the tag
	r2d2::Coordinate tag_cor = detected_tag.position.get_coordinate();
	// tag_at, the attitude of the tag
	r2d2::Attitude tag_at = detected_tag.position.get_attitude();

	// tag_pitch, the pitch of tag and the camera
	r2d2::Angle tag_pitch = tag_at.get_pitch();
	
	// refference angle, the absolute pitch of the tag + the pitch between camera and tag 
	// with gifs the absolute rotation of the camera
	r2d2::Angle ref_angle = tag_pitch + (pitch * r2d2::Angle::rad);

	// if ref_angle is bigger than 2 PI, than substract 2 PI
	if (ref_angle > ((2 * PI)* r2d2::Angle::rad)){ ref_angle -= (2 * PI)* r2d2::Angle::rad;}
	
	// calculate in witch kwadrant the camera is
	int kwadrant = ref_angle.get_angle() / ((0.5 * PI));
	
	// calculate the x and y offset between the camera and the tag
	r2d2::Angle tmp_angle = (ref_angle.get_angle() - (kwadrant* 0.5 * PI)) * r2d2::Angle::rad;
	double tmp_y = sin(tmp_angle.get_angle()) * distance;
	double tmp_x = cos(tmp_angle.get_angle()) * distance;

	r2d2::Length position_x = 0 * r2d2::Length::METER;
	r2d2::Length position_y = 0 * r2d2::Length::METER;
	r2d2::Length position_z = 1 * r2d2::Length::METER;

	// calculate the x and y position of the camera
	if (kwadrant == 0){ 
		position_x = tag_cor.get_x() + (tmp_x * r2d2::Length::METER);
		position_y = tag_cor.get_y() - (tmp_y * r2d2::Length::METER);}
	else if(kwadrant == 1){
        position_x = tag_cor.get_x() + (tmp_x * r2d2::Length::METER);
		position_y = tag_cor.get_y() + (tmp_y * r2d2::Length::METER);}
	else if(kwadrant == 2){
        position_x = tag_cor.get_x() - (tmp_x * r2d2::Length::METER);
		position_y = tag_cor.get_y() + (tmp_y * r2d2::Length::METER);}
	else if(kwadrant == 3){
        position_x = tag_cor.get_x() - (tmp_x * r2d2::Length::METER);
		position_y = tag_cor.get_y() - (tmp_y * r2d2::Length::METER);}
	//cout<<"position_x: " << position_x<< " position_y : " << position_y<< endl;
	return r2d2::Coordinate(position_x, position_y, position_z);
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
	SharedObject<r2d2::Coordinate>::Accessor acc(shared_coordinate);
	result = SensorResult(0.0, acc.access());
	return result;
  }

// here is were everything begins
int main(int argc, char* argv[]) {	
	VideoTag tag (0.0);
	while(true){
	  tag.get_data(); 
	}
  return 0;
}

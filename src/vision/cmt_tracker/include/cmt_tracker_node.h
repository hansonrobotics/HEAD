#ifndef CMT_TRACKER_H
#define CMT_TRACKER_H

#include <cmt_tracker_msgs/Tracker.h>
#include <cmt_tracker_msgs/Trackers.h>
#include <cmt_tracker_msgs/Object.h>
#include <cmt_tracker_msgs/Objects.h>
#include <cmt_tracker_msgs/TrackedImages.h>
#include <cmt_tracker_msgs/TrackerNames.h>
#include <cmt_tracker_msgs/MergeNames.h>
//OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//ROS - OpenCV libraries
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
//#include <sensor_msgs/ImageConstPtr.h>

//Service call to reset the values of the images in the system
#include <cmt_tracker_msgs/Clear.h>
#include <cmt_tracker_msgs/Update.h>
#include <cmt_tracker_msgs/Delete.h>
#include <cmt_tracker_msgs/Reinitialize.h>
#include <pi_face_tracker/Face.h>
#include <pi_face_tracker/Faces.h>
#include <pi_face_tracker/FaceEvent.h>
#include <cmt_tracker_msgs/TrackerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>
#include <stdlib.h>
#include <time.h>

#include <new>

#include "CMT_MAP.h"
#include "Config.h"

#include <string>

#include <cmath> 



namespace cmt_wrap {

struct camera_properties
{
	int width; 
	int height; 
	double fov; 
	int offset; 
} camera_config;

typedef cv::Matx44d head_pose;

class TrackerCMT
{

public: 
	//constructor
	TrackerCMT();


	//Services
	//Remove all the tracking instances of the system. 
	bool clear(cmt_tracker_msgs::Clear::Request &req, cmt_tracker_msgs::Clear::Response &res); 

	//Check if there are changes in what is being tracke in cmt arrays. 
	bool updated(cmt_tracker_msgs::Update::Request &req, cmt_tracker_msgs::Update::Response &res);

	//Get the list of images from the cmt instances that we are tracking
	bool getTrackedImages(cmt_tracker_msgs::TrackedImages::Request &req,cmt_tracker_msgs::TrackedImages::Response &res);

	//This updates the names of the trackers during runs when a name is detected. 
	bool updateTrackerNames(cmt_tracker_msgs::TrackerNames::Request &req, cmt_tracker_msgs::TrackerNames::Response &res);


	//Callbacks for subscribers

	//image subscriber
	void imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& camerainfo);


	void list_of_faces_update(const cmt_tracker_msgs::Objects& faces_info);

	void list_of_faces_emo_update(const cmt_tracker_msgs::Objects& faces_info);

	//Subscriber to set the cmt to location specified by the tracker_location
	void set_tracker(const cmt_tracker_msgs::Tracker& tracker_location); 

	//Subscriber to set the cmt to locations specified by the tracker locations
	void set_trackers(const cmt_tracker_msgs::Trackers& tracker_location);

	//A function that removes the ith element from the vector of trackers. 
//	void remove_tracker(int index);
	void remove_tracker(std::string name);

	bool merge_elements(cmt_tracker_msgs::MergeNames::Request &req, cmt_tracker_msgs::MergeNames::Response &res);

	bool delete_elements(cmt_tracker_msgs::Delete::Request &req, cmt_tracker_msgs::Delete::Response &res);

	bool updateArea(cmt_tracker_msgs::Reinitialize::Request &req, cmt_tracker_msgs::Reinitialize::Response &res);

	bool googleUpdate(cmt_tracker_msgs::Reinitialize::Request &req, cmt_tracker_msgs::Reinitialize::Response &res);

    bool validate(cmt_tracker_msgs::TrackerNames::Request &req, cmt_tracker_msgs::TrackerNames::Response &res);

    bool reinforce(cmt_tracker_msgs::TrackerNames::Request &req, cmt_tracker_msgs::TrackerNames::Response &res);

	void callback(cmt_tracker_msgs::TrackerConfig &config, uint32_t level); 

	//Rules
	void deleteOnLost();

    pi_face_tracker::Face filter_point(pi_face_tracker::Face f);


private: 

	ros::NodeHandle nh_;
	image_geometry::PinholeCameraModel cameramodel;
	//To hold the images of from image callback;
	cv::Mat conversion_mat_;
	cv::Mat frame_gray;
	cv::Mat im_gray;
	cv::Mat im_masked;
	//To listen to dynamic reconfigure variables in the system.
	dynamic_reconfigure::Server<cmt_tracker_msgs::TrackerConfig> server;
	dynamic_reconfigure::Server<cmt_tracker_msgs::TrackerConfig>::CallbackType f;


	cmt_tracker_msgs::Tracker track_location;

	cmt_tracker_msgs::Trackers trackers_results;
	cmt_tracker_msgs::Trackers temp_results;

	cmt_tracker_msgs::Tracker tracker_set;
	cmt_tracker_msgs::Objects face_locs;

	//This is a poor persons version of the results. Would need to be updated in the system mgoing forward.
	cmt_tracker_msgs::Objects emo_locs;

	ros::ServiceServer clear_service;
	ros::ServiceServer image_service;
	ros::ServiceServer update_service;
	ros::ServiceServer recognition_service;
    ros::ServiceServer google_service;
    ros::ServiceServer reinforce_service;
    ros::ServiceServer merge_service;
    ros::ServiceServer updateArea_service;
    ros::ServiceServer delete_service;


    ros::ServiceClient add_to_tracker;

    std::map<std::string, std::string> google_results;
    std_srvs::Empty empty_info;
	image_transport::ImageTransport it_;

	image_transport::CameraSubscriber image_sub_;

	//Probably need to be enabled for debuggin puproses. 
	image_transport::Publisher image_pub_;

	image_transport::Publisher image_face_pub;

	//tracker results locations. 
	ros::Publisher tracker_locations_pub;

	ros::Publisher tracker_results_pub;
	ros::Publisher tracker_results_temp;

	ros::Publisher pi_vision_results; 
	ros::Publisher pi_events; 
    std::map<std::string,pi_face_tracker::Face> face_filtered;
    std::map<std::string,std::string> pi_registry;
	//Now this is face results; Moving to another function. 
	ros::Subscriber face_results;
	ros::Subscriber emo_results;

	//Now this is how to set the locator functions. For single tracker and multiple set
	ros::Subscriber tracker_subscriber;
	ros::Subscriber trackers_subscriber;

	//what is this
	ros::Subscriber face_subscriber;

	//Instances of MAP.
	cmt::CMT_MAP cmt_;
	cmt::Config config;

    //This one holds
    std::map<std::string,std::string> recognized_names;
    std::map<std::string,bool> validated;

    std::map<std::string,int> poor_results_counter;

	std::vector<std::string> poorly_tracked;
	std::vector<std::string> newly_tracked;
	cv::Rect bb_rect;
	//premove
	std::string tracking_method;

	std::string input_camera;
	std::string face_location_topics;
	std::string face_event_topics;
	std::string filtered_face_locations;

	bool emotime;

	//the threshold value that is passed to the cmt instance. 
	double factor;

	std::string detector;
	std::string descriptor;
	std::string estimation;

	int delete_counter;
	int downgrade;

	//for setting the tracking to higher levels.
	std::vector<cv::Rect> locations_of_trackers;

    std::map<std::string, std::string> merge;
    std::vector<std::string> delete_trackers;

	std::string subscribe_topic;
	std::string tool;

	sensor_msgs::ImagePtr masked_image;

	//for debugging purposes. 
	int frame_counters;
	int frame_previous;

	//value of the system. 
	bool update_ui;
	bool update_ui_wait; 

	bool clear_cmt;

	float focalLength;
  	float opticalCenterX;
  	float opticalCenterY;




};
namespace {
  cmt_tracker_msgs::Trackers convert(std::vector<cv::Rect> faces);
  pi_face_tracker::Face returnPiMessage(cmt_tracker_msgs::Tracker loc, camera_properties camera_config); 
  pi_face_tracker::Faces returnPiMessages(cmt_tracker_msgs::Trackers locs, camera_properties camera_config); 
  pi_face_tracker::FaceEvent returnPiEvents(std::string event, std::string face_id);
  cmt_tracker_msgs::Trackers returnOverlappingEmotion(cmt_tracker_msgs::Trackers locs, cmt_tracker_msgs::Objects facelocs);
  cmt_tracker_msgs::Trackers returnOverlappingPose(cmt_tracker_msgs::Trackers locs, cmt_tracker_msgs::Objects facelocs);
}
}
#endif // CMT_TRACKER_H
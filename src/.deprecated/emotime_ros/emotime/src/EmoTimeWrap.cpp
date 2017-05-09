#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*********** cmt headers****************/
#include <cmt_tracker_msgs/Object.h>
#include <cmt_tracker_msgs/Objects.h>
//
#include "SVMEmoDetector.h"
#include "BoostEmoDetector.h"
#include "matrix_io.h"
#include "FacePreProcessor.h"

#include "TrainingParameters.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <map>
#include <iostream>
#include "std_msgs/String.h"
#include <sstream>
#include <ros/package.h>

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

using namespace cv;
using namespace emotime;

using std::cout;
using std::endl;
using std::cerr;
using std::string;
using std::pair;

static const std::string OPENCV_WINDOW = "Image window";

class EmoTimeWrap
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber face_location_sub;
  image_transport::Publisher image_pub_;
  ros::Publisher emotion_pub;
  ros::Publisher faces_locations;
  std_msgs::String emotion_msg;
  string method;
  string config;
  string config_e;
  string profile;
  string eye_glass;
  Size size;
  int counter;
  int nwidths, nlambdas, nthetas;
  vector<string> classifier_paths;
  FacePreProcessor* preprocessor;
  EmoDetector* emodetector;
  string text;
  int fontFace;
  double fontScale;
  int thickness;
  int lineType;
  bool bottomLeftOrigin;
//cv::Point textOrg(10, 130);
  Point textOrg;
  string emotionString;
  float emotionAccuracy; 
  string subscribe_topic;
  std::string filtered_face_locations;
  cmt_tracker_msgs::Objects face_locs;
  cmt_tracker_msgs::Objects emot_pub_faces;
  //string config;
  //string config_e;

  string path;
public:
  EmoTimeWrap()
    : it_(nh_)
  {

// Subscrive to input video feed and sadness_vs_anger_contempt_disgust_fear_happy_neutral_surprise_featspublish output video feed
    counter = 0;
    nh_.getParam("camera_topic", subscribe_topic);
    nh_.getParam("filtered_face_locations", filtered_face_locations);
    image_sub_ = it_.subscribe(subscribe_topic, 1,&EmoTimeWrap::imageCb, this);
    face_location_sub = nh_.subscribe(filtered_face_locations, 1, &EmoTimeWrap::list_of_faces_update, this);
    //image_pub_ = it_.advertise("/emotime_node/output_video", 1);
    //emotion_pub = nh_.advertise<std_msgs::String>("emotion_states", 1000);
    faces_locations = nh_.advertise<cmt_tracker_msgs::Objects>("emo_pub_registered", 10);
    method = "svm";
    // config = "/home/lina/Desktop/emotime_final/emotime/resources/haarcascade_frontalface_cbcl1.xml";
    config = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
    config_e = "/usr/share/opencv/haarcascades/haarcascade_eye.xml";
    size.width = 52;
    size.height = 52;
    nwidths = 1;
    nlambdas = 5;
    nthetas = 8;

    path= ros::package::getPath("emotime");
    classifier_paths.push_back(path + "/svm/anger_vs_contempt_disgust_fear_happy_neutral_sadness_surprise_feats.xml");
    classifier_paths.push_back(path + "/svm/contempt_vs_anger_disgust_fear_happy_neutral_sadness_surprise_feats.xml");
    classifier_paths.push_back(path + "/svm/disgust_vs_anger_contempt_fear_happy_neutral_sadness_surprise_feats.xml");
    classifier_paths.push_back(path + "/svm/fear_vs_anger_contempt_disgust_happy_neutral_sadness_surprise_feats.xml");
    classifier_paths.push_back(path + "/svm/happy_vs_anger_contempt_disgust_fear_neutral_sadness_surprise_feats.xml");
    classifier_paths.push_back(path + "/svm/neutral_vs_anger_contempt_disgust_fear_happy_sadness_surprise_feats.xml");
    classifier_paths.push_back(path + "/svm/sadness_vs_anger_contempt_disgust_fear_happy_neutral_surprise_feats.xml");
    classifier_paths.push_back(path + "/svm/surprise_vs_anger_contempt_disgust_fear_happy_neutral_sadness_feats.xml");

    //cout << "Length of the classifiers: " << classifier_paths.size() << std::endl;
    preprocessor = new FacePreProcessor(config, config_e, size.width,
                                        size.height, nwidths, nlambdas, nthetas);

    emodetector = new SVMEmoDetector(kCfactor, kMaxIteration, kErrorMargin);
    emodetector->init(classifier_paths);
    //text = "Funny text inside the box";
    fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    fontScale = 2;
    thickness = 3;
    textOrg.x = 10;
    textOrg.y = 130;
    int lineType = 8;
    bool bottomLeftOrigin = false;
    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~EmoTimeWrap()
  {
    cv::destroyWindow(OPENCV_WINDOW);

    delete emodetector;
    delete preprocessor;
  }

  void list_of_faces_update(const cmt_tracker_msgs::Objects& faces_info)
  {
    //ROS_DEBUG("It get's here in the faces update");
  face_locs.objects.clear();
  for (int i = 0; i < faces_info.objects.size(); i++)
  {
    face_locs.objects.push_back(faces_info.objects[i]);
  }
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat img = cv_ptr->image;
    Mat features;
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    for (int i = 0; i < face_locs.objects.size(); i++)
    {
      Mat roi = img(cv::Rect(face_locs.objects[i].object.x_offset, face_locs.objects[i].object.y_offset,
                              face_locs.objects[i].object.width, face_locs.objects[i].object.height)).clone();


      bool canPreprocess = preprocessor->preprocess(roi, features);
      if (!canPreprocess) {
       // cout << " Can't Process" << endl;
      }
      else {
        pair<Emotion, float> prediction = emodetector->predict(features);
        emotionString = emotionStrings(prediction.first);
        emotionAccuracy = prediction.second;
        //emotion_msg.data = emotionStrings(prediction.first);
       // ROS_INFO("%s", emotion_msg.data.c_str());

      }
      //cv::putText(roi, emotionString, textOrg, fontFace, fontScale, CV_RGB(255, 255, 0), thickness, lineType);
      // Output modified video stream


//      std::string s = SSTR( i );

      //cv::imshow(s, roi);
      //cv::waitKey(3);
      //emotion_pub.publish(emotion_msg);
      //image_pub_.publish(cv_ptr->toImageMsg());

      cmt_tracker_msgs::Object face_description;
      face_description = face_locs.objects[i];


      face_description.obj_states.data = emotionString;
      face_description.obj_accuracy.data = emotionAccuracy;
      emot_pub_faces.objects.push_back(face_description);

    }
    faces_locations.publish(emot_pub_faces);
    face_locs.objects.clear();
    emot_pub_faces.objects.clear();




  }
};

int main(int argc,  char* argv[]) {

  ros::init(argc, argv, "EmoTimeWrap");
  EmoTimeWrap etw;
  ros::spin();
  return 0;
}




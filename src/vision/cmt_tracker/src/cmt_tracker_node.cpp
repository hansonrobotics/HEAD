#include "cmt_tracker_node.h"
#include "face_locator_node.h"
#include "ros/console.h"
namespace cmt_wrap {

TrackerCMT::TrackerCMT() : it_(nh_)
{
  //nh_.getParam("camera_topic", input_camera);

  nh_.getParam("publish_topic_event", face_event_topics);
  nh_.getParam("publish_topic_location", face_location_topics);

  nh_.getParam("camera_topic", subscribe_topic);

  nh_.getParam("filtered_face_locations", filtered_face_locations);

  nh_.getParam("emotime", emotime);
  nh_.getParam("tracker_tool",tool);

  //For benchmarking elements
  frame_counters = 0;
  frame_previous = 0;

  update_ui = false;
  //To acquire the image that we will be doing processing on
  image_sub_ = it_.subscribeCamera(subscribe_topic, 1, &cmt_wrap::TrackerCMT::imageCb, this);

  //Service
  clear_service = nh_.advertiseService("clear", &cmt_wrap::TrackerCMT::clear, this);
  image_service = nh_.advertiseService("get_cmt_rects", &cmt_wrap::TrackerCMT::getTrackedImages, this);
  update_service = nh_.advertiseService("update", &cmt_wrap::TrackerCMT::updated, this);
  recognition_service  = nh_.advertiseService("recognition", &cmt_wrap::TrackerCMT::updateTrackerNames,this);
  google_service = nh_.advertiseService("google_scraper", &cmt_wrap::TrackerCMT::validate, this);
  reinforce_service = nh_.advertiseService("reinforce", &cmt_wrap::TrackerCMT::reinforce, this);
  merge_service = nh_.advertiseService("merge", &cmt_wrap::TrackerCMT::merge_elements, this);
  delete_service = nh_.advertiseService("delete", &cmt_wrap::TrackerCMT::delete_elements, this);
  updateArea_service = nh_.advertiseService("update_area", &cmt_wrap::TrackerCMT::updateArea, this);

  add_to_tracker = nh_.serviceClient<std_srvs::Empty>("can_add_tracker");
  //subscribers
  face_subscriber = (nh_).subscribe(filtered_face_locations, 1, &cmt_wrap::TrackerCMT::list_of_faces_update, this);
  emo_results = (nh_).subscribe("emo_pub_registered",1,&cmt_wrap::TrackerCMT::list_of_faces_emo_update,this);


  //this is the one that creates what add to track based on the overlay of previous cmt tracking results and faces in face_locator_node
  tracker_locations_pub = (nh_).advertise<cmt_tracker_msgs::Trackers>("tracking_locations", 1);
  tracker_subscriber = (nh_).subscribe("tracking_location", 1, &cmt_wrap::TrackerCMT::set_tracker, this);
  trackers_subscriber = (nh_).subscribe("tracking_locations", 1, &cmt_wrap::TrackerCMT::set_trackers, this);

  //This is the one that's publishing the message.
  tracker_results_pub = nh_.advertise<cmt_tracker_msgs::Trackers>("tracker_results", 10);
  tracker_results_temp = nh_.advertise<cmt_tracker_msgs::Trackers>("temporary_trackers", 10);


  //pi_vision_results = nh_.advertise<pi_face_tracker::Faces>(face_location_topics, 10);
  face_events_pub = (nh_).advertise<cmt_tracker_msgs::TrackerEvents>(face_event_topics, 10);

  //Bind to call the dynamic reconfigure values by which we delete the cmt instances.
  f = boost::bind(&cmt_wrap::TrackerCMT::callback, this, _1, _2);
  //TODO initialization doesn't trigger unless the rqt_reconfigure is triggered.
  
  server.setCallback(f);

  //Now let's read the camera pictures form the system.
  nh_.param<int>("width", camera_config.width, 640);
  nh_.param<int>("height", camera_config.height, 480);
  camera_config.fov = 1.42;
}

bool TrackerCMT::clear(cmt_tracker_msgs::Clear::Request &req, cmt_tracker_msgs::Clear::Response &res)
{
  std::cout<<"Remove all tracked service"<<std::endl;

  cmt_.clear();
  poorly_tracked = cmt_.lostFace();
  deleteOnLost();
  std::cout<<"Remove all tracked service"<<std::endl;
  return true;
}
//this is a service that indicates wheteher new elements are going to be tracked or note.
bool TrackerCMT::updated(cmt_tracker_msgs::Update::Request &req, cmt_tracker_msgs::Update::Response &res)
{
  res.update.data = false;
  if (update_ui)
  {
    res.update.data = true;
    update_ui = false;
  }
  return true;
}
bool TrackerCMT::validate(cmt_tracker_msgs::TrackerNames::Request &req, cmt_tracker_msgs::TrackerNames::Response &res)
{
    google_results[std::to_string(req.index)] = req.names;
    return true;
}

bool TrackerCMT::reinforce(cmt_tracker_msgs::TrackerNames::Request &req, cmt_tracker_msgs::TrackerNames::Response &res)
{
    if(cmt_.reinforce(req.names,req.index))
    return true;
    else
    return false;
}

bool TrackerCMT::updateArea(cmt_tracker_msgs::Reinitialize::Request &req, cmt_tracker_msgs::Reinitialize::Response &res)
{

cmt_.updateArea(im_gray, cv::Rect(req.object.x_offset, req.object.y_offset, req.object.width, req.object.height),req.name);

return true;
}
/*
Request the images in the system.
*/
bool TrackerCMT::getTrackedImages(cmt_tracker_msgs::TrackedImages::Request &req,
                                  cmt_tracker_msgs::TrackedImages::Response &res)
{
  std::map<string, cv::Mat> bgImages = cmt_.getImages();
  for(std::map<std::string, cv::Mat>::iterator v = bgImages.begin(); v!= bgImages.end(); v++)
  {
    res.names.push_back(v->first);

    sensor_msgs::ImagePtr masked_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", v->second).toImageMsg();

    res.image.push_back(*masked_image);
  }
  return true;
}
bool TrackerCMT::merge_elements(cmt_tracker_msgs::MergeNames::Request &req, cmt_tracker_msgs::MergeNames::Response &res)
{
    //Callback are handled sequentially; http://answers.ros.org/question/28373/race-conditions-in-callbacks/
    merge.clear();
    if(req.merge_to.size() == req.merge_from.size())
    {
    for(int i = 0; i< req.merge_to.size(); i++)
    {
        merge[req.merge_to[i]] = req.merge_from[i];
    }
    return true;
    }
    else
    return false;
}
bool TrackerCMT::delete_elements(cmt_tracker_msgs::Delete::Request &req, cmt_tracker_msgs::Delete::Response &res)
{
    //Callback are handled sequentially; http://answers.ros.org/question/28373/race-conditions-in-callbacks/
    std::cout<<"Deleting Elements Because of Multiple overlap or Area Became very high"<<std::endl;
    delete_trackers.clear();
    for(int i = 0; i< req.delete_trackers.size(); i++)
    {
        delete_trackers.push_back(req.delete_trackers[i]);
        cmt_.deleteTracker(req.delete_trackers[i]);
    }
    poorly_tracked = delete_trackers;
    deleteOnLost();
    return true;

}
bool TrackerCMT::updateTrackerNames(cmt_tracker_msgs::TrackerNames::Request &req, cmt_tracker_msgs::TrackerNames::Response &res)
{
    if( cmt_.updatemapname(req.names, req.index))
    {
    //TODO trigger an event here.
    //nh_.setParam("tracker_updated", 2);
    return true;
    }
    else
    {
    return false;
    }
}
/*
This  function is a callback function that happens when an image update occurs.
*/
void TrackerCMT::imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& camerainfo)
{

  //std::cout<<"Enters imageCB"<<std::endl;
  //-//std::cout<<"Enters imageCB conversion"<<std::endl;
  cameramodel.fromCameraInfo(camerainfo);

      if(cameramodel.intrinsicMatrix()(0,0) == 0.0) {
            ROS_ERROR("Camera publishes uncalibrated images. Can not estimate face position.");
            ROS_WARN("Detection will start over again when camera info is available.");
    }

  focalLength = cameramodel.fx();
  opticalCenterX = cameramodel.cx();
  opticalCenterY = cameramodel.cy();
  //TODO why not just subscribe to the MONO image and skip all this handling. A way must be included to do that functionality.
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      }  else {
        // qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        std::cerr << "Error in the conversion of the iamge" << std::endl;
        // ui_.image_frame->setImage(QImage());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      std::cerr << "Error in the conversion of the image" << std::endl;
      // qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      // ui_.image_frame->setImage(QImage());
      return;
    }
  }

  cv::cvtColor(conversion_mat_, frame_gray, CV_BGR2GRAY);

  im_gray = frame_gray.clone();
  //Now let's copy an additional mat to do additional removal.

  poorly_tracked.clear();
  newly_tracked.clear();

  std::vector<cmt::cmt_message> messages =  cmt_.process_map(im_gray, factor,merge,delete_counter);
  merge.clear();

  std::vector<std::string> available_names;
  for(std::vector<cmt::cmt_message>::iterator v = messages.begin(); v!= messages.end(); v++)
  {
    if (!((*v).tracker_lost))
    {
    cmt_tracker_msgs::Tracker tracker;
    tracker.initial_points.data = (*v).initial_active_points;
    tracker.active_points.data = (*v).active_points;
    tracker.tracker_name.data = (*v).tracker_name;

    if (google_results.find(tracker.tracker_name.data) != google_results.end())
        tracker.google_best_guess.data = google_results[tracker.tracker_name.data];

    tracker.object.object.x_offset = (*v).rect.x;
    tracker.object.object.y_offset = (*v).rect.y;

    tracker.object.object.width = (*v).rect.width;
    tracker.object.object.height = (*v).rect.height;


    tracker.quality_results.data = !((*v).tracker_lost);
    tracker.recognized.data = (*v).recognized;
    tracker.recognized_name.data = (*v).recognized_as;
    tracker.before_demotion.data = (*v).before_being_demoted;
    tracker.point = return3DLocations(tracker, camera_config);
    tracker.header.stamp = ros::Time::now();
    tracker.validated.data = (*v).validated;
    if ((*v).validated)
    {
    trackers_results.tracker_results.push_back(tracker);
//    if(pi_registry.find(tracker.tracker_name.data)==pi_registry.end())
//    {
//    pi_face_tracker::FaceEvent m = returnPiEvents("new_face", tracker.tracker_name.data);
//    pi_events.publish(m);
//    pi_registry[tracker.tracker_name.data] = tracker.tracker_name.data;
//    }

    }
    else
    temp_results.tracker_results.push_back(tracker);
    }
  }
    cmt_.removeLost();

    poorly_tracked = cmt_.lostFace();
    newly_tracked = cmt_.newFace();
    cmt_.clearFace();
//    bool emo_enabled;
//    nh_.getParam("emotime",emo_enabled);
//    if(emo_enabled)
//    {
    //TODO there needs to be a way to better the emotion recognizer here.
    trackers_results = returnOverlappingEmotion(trackers_results, emo_locs);
//    }

    //pi_face_tracker::Faces pi_results = returnPiMessages(trackers_results, camera_config);//Currently zero then let's get the overlapped


    //Check if it exists in here before publishing:
    for (int j = 0; j < trackers_results.tracker_results.size(); j ++)
    {
        if(std::find(newly_tracked.begin(), newly_tracked.end(),trackers_results.tracker_results[j].tracker_name.data)!=newly_tracked.end())
        {
            face_events.face_event.data = "new_face";
            face_events.face_id.data = std::atoi(trackers_results.tracker_results[j].tracker_name.data.c_str());
            face_events_pub.publish(face_events);
            pi_registry[trackers_results.tracker_results[j].tracker_name.data] = trackers_results.tracker_results[j].tracker_name.data;
        }
    }



//    for (int pi_res = 0; pi_res < pi_results.faces.size(); pi_res++)
//    {
//        pi_results.faces[pi_res] = filter_point(pi_results.faces[pi_res]);
//    }
//  bool emo_enabled,pose_enabled;
//  nh_.getParam("pose",pose_enabled);
//  nh_.getParam("emotime",emo_enabled);
//

//  if(pose_enabled)
//  {
//  trackers_results = returnOverlappingPose(trackers_results, face_locs);
//  }

  //-//std::cout<<"Going to Publish"<<std::endl;
  trackers_results.header.stamp = ros::Time::now();
  temp_results.header.stamp = ros::Time::now();
  tracker_results_pub.publish(trackers_results);
  tracker_results_temp.publish(temp_results);
  //pi_vision_results.publish(pi_results);
//  //std::cout<<"Finished Publish"<<std::endl;
  deleteOnLost();
  trackers_results.tracker_results.clear();
  temp_results.tracker_results.clear();
  //std::cout<<"images Callback"<<std::endl;
}
//Now this function deltes all the poor tracked values and adds trackers if there are overlaps in the system.
void TrackerCMT::deleteOnLost()
{
  //std::cout<<"Enters DeleteOnLost"<<std::endl;

  for (int i = 0; i < poorly_tracked.size(); i++)
  {
  remove_tracker(poorly_tracked[i]);
//  face_filtered.erase(poorly_tracked[i]);
  }

  //nh_.setParam("tracker_updated", 2);

  poorly_tracked.clear();
  //std::cout<<"Exits DeleteOnLost"<<std::endl;
}

void TrackerCMT::callback(cmt_tracker_msgs::TrackerConfig &config, uint32_t level)
{
  //std::cout<<"Factor to be Updated"<<std::endl;
  factor = config.factor;
  //std::cout<<"config: "<<config.frame_counter;
  delete_counter = config.frame_counter;
  //std::cout<<"Factor Updated to: "<<factor<<std::endl;
}
void TrackerCMT::list_of_faces_update(const cmt_tracker_msgs::Objects& faces_info)
{
  //std::cout<<"Enters list_of_faces_update"<<std::endl;
  face_locs.objects.clear();
  for (int i = 0; i < faces_info.objects.size(); i++)
  {
    face_locs.objects.push_back(faces_info.objects[i]);
  }
  //std::cout<<"Enters list_of_faces_update"<<std::endl;
}
void TrackerCMT::list_of_faces_emo_update(const cmt_tracker_msgs::Objects& faces_info)
{
  //-//std::cout<<"Enters list_of_faces_emo_update"<<std::endl;
  emo_locs.objects.clear();
  for (int i = 0; i < faces_info.objects.size(); i++)
  {
    emo_locs.objects.push_back(faces_info.objects[i]);
  }
  //-//std::cout<<"Enters list_of_faces_emo_update"<<std::endl;
  //nh_.setParam("tracker_updated",true);
}
void TrackerCMT::set_tracker(const cmt_tracker_msgs::Tracker& tracker_location)
{
  std::cout<<"Enter New Face Started"<<std::endl;
  cv::Mat im_gray = frame_gray.clone(); //To avoid change when being run.
  cv::Rect rect(tracker_location.object.object.x_offset, tracker_location.object.object.y_offset,
                      tracker_location.object.object.width, tracker_location.object.object.height);
  rect = rect & cv::Rect(0, 0, im_gray.size().width, im_gray.size().height);
  if (!im_gray.empty() && rect.area() > 50)
  {
    std::string tracker_name = cmt_.addtomap(im_gray, rect);

  }
  std::cout<<"Enter New Face Finished"<<std::endl;
}


void TrackerCMT::set_trackers(const cmt_tracker_msgs::Trackers& tracker_location)
{
  for (int i = 0; i < tracker_location.tracker_results.size(); i++)
  {
    set_tracker(tracker_location.tracker_results[i]);
  }

  //nh_.setParam("tracker_updated", 2);
  int sizeof_t = cmt_.size_map();
  nh_.setParam("tracker_size",sizeof_t);
  ros::service::call("can_add_tracker",empty_info);

}

void TrackerCMT::remove_tracker(std::string name)
{
  std::cout<<"Enters remove_tracker"<<std::endl;

  if(pi_registry.find(name)!=pi_registry.end())
  {
   face_events.face_event.data = "lost_face";
   face_events.face_id.data = std::atoi(name.c_str());
   face_events_pub.publish(face_events);

   pi_registry.erase(name);
  }

  std::cout<<"Exits remove_trackers"<<std::endl;
}

//pi_face_tracker::Face TrackerCMT::filter_point(pi_face_tracker::Face f)
//{
//double yz_sf = 0.64;
//double x_sf = 0.95;
////std::cout<<"Filtered Locations: "<<face_filtered<<std::endl;
//
//if(face_filtered.find(std::to_string(f.id))==face_filtered.end())
//{
//    face_filtered[std::to_string(f.id)] = f;
//}
//pi_face_tracker::Face fac = f;
//
////Now there is a vlaue that would be assinged that is self.loc_3d = p
//double pha = yz_sf;
//double bet = 1 - pha;
//
// fac.point.y = pha * face_filtered[std::to_string(f.id)].point.y + bet * f.point.y;
// fac.point.z = pha * face_filtered[std::to_string(f.id)].point.z + bet * f.point.z;
// pha = x_sf;
// bet = 1.0 - pha;
// fac.point.x = pha * face_filtered[std::to_string(f.id)].point.x + bet * f.point.x;
//
// face_filtered[std::to_string(f.id)] = fac;
//return fac;
//}

namespace {
cmt_tracker_msgs::Trackers convert(std::vector<cv::Rect> faces)
{
  cmt_tracker_msgs::Trackers tracker_description;
  for (size_t i = 0; i < faces.size(); i++)
  {

    cmt_tracker_msgs::Tracker face_description;
    face_description.object.object.x_offset = faces[i].x;
    face_description.object.object.y_offset = faces[i].y;
    face_description.object.object.height = faces[i].height;
    face_description.object.object.width = faces[i].width;


    tracker_description.tracker_results.push_back(face_description);
  }

  return tracker_description;
}




geometry_msgs::Point return3DLocations(cmt_tracker_msgs::Tracker locs, camera_properties camera_config)
{
  geometry_msgs::Point msg;
  double dp;
  double width;
  double height;

  if( locs.object.object.width != 0)
  dp = 0.17 / (float)(locs.object.object.width); //This is how it's defined as above
  else
  dp = 0;

  width = (double)camera_config.width / 2.0;
  height = (double)camera_config.height / 2.0;
  double k_const = (double)width / (double) (tan(camera_config.fov/2.0));

  msg.x = dp * k_const;

  msg.y = dp * (width - ( (double)(locs.object.object.x_offset + locs.object.object.width) +
                locs.object.object.x_offset) / 2.0 );
  msg.z = dp * (height - ((double)(locs.object.object.y_offset + locs.object.object.height) +
                locs.object.object.y_offset) / 2.0 );

  return msg;
}
cmt_tracker_msgs::Trackers returnOverlappingEmotion(cmt_tracker_msgs::Trackers locs, cmt_tracker_msgs::Objects facelocs)
{
  cmt_tracker_msgs::Trackers registeredEmotions; 
  for (int i = 0; i < locs.tracker_results.size(); i++)
  {
    cmt_tracker_msgs::Tracker emotion; 
    cv::Rect emotion_overlap(locs.tracker_results[i].object.object.x_offset, locs.tracker_results[i].object.object.y_offset,
      locs.tracker_results[i].object.object.width, locs.tracker_results[i].object.object.height);
    emotion = locs.tracker_results[i]; 
    //Now the other part of the tracker are the same as the locs results; we just need to append the emotional state to the system
    bool overlap = false;
    for (int j = 0; j < facelocs.objects.size(); j++)
    {
      //if it overlaps with the one then break.
      cv::Rect cmt_overlap(facelocs.objects[i].object.x_offset, facelocs.objects[i].object.y_offset,
                              facelocs.objects[i].object.width, facelocs.objects[i].object.height);
       double intersection_area = (emotion_overlap & cmt_overlap).area();
       double union_area = (emotion_overlap | cmt_overlap).area();

       overlap = (intersection_area/union_area) > 0.5;
      if (overlap)
        {
        //Now we add the to the cmt the overlapped emotion; if there are no emotion overlap then we add unknown emotion state to the system. 
        //assign and break
          emotion.object.obj_states.data = facelocs.objects[i].obj_states.data;
          emotion.object.obj_accuracy.data = facelocs.objects[i].obj_accuracy.data;
          //Now update this value to the system as the rest of the overlapps are unnecessary
          break; 
        }
      else
      {
          emotion.object.obj_states.data = "unknown";
          emotion.object.obj_accuracy.data = 0;
      }
    }
    registeredEmotions.tracker_results.push_back(emotion); 
  }

  return registeredEmotions;
}

cmt_tracker_msgs::Trackers returnOverlappingPose(cmt_tracker_msgs::Trackers locs, cmt_tracker_msgs::Objects facelocs)
{
  cmt_tracker_msgs::Trackers registeredPose;
  for (int i = 0; i < locs.tracker_results.size(); i++)
  {
    cmt_tracker_msgs::Tracker pose;
    cv::Rect pose_overlap(locs.tracker_results[i].object.object.x_offset, locs.tracker_results[i].object.object.y_offset,
      locs.tracker_results[i].object.object.width, locs.tracker_results[i].object.object.height);
    pose = locs.tracker_results[i];
    //Now the other part of the tracker are the same as the locs results; we just need to append the emotional state to the system
    bool overlap = false;
    for (int j = 0; j < facelocs.objects.size(); j++)
    {
      //if it overlaps with the one then break.
      cv::Rect cmt_overlap(facelocs.objects[i].object.x_offset, facelocs.objects[i].object.y_offset,
                              facelocs.objects[i].object.width, facelocs.objects[i].object.height);
      overlap = (pose_overlap & cmt_overlap).area() > 0;
      if (overlap)
        {
          pose.object.feature_point = facelocs.objects[i].feature_point;
          break;
        }
      else
      {
        //May Be do some rough guessing?
      }
    }
    registeredPose.tracker_results.push_back(pose);
  }

  return registeredPose;
}

}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmt_tracker");
  cmt_wrap::TrackerCMT ic;
  ros::spin();
  return 0;
}
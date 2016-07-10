#include "face_locator_node.h"
namespace face_detect {

Face_Detection::Face_Detection()
  : it_(nh_)
{
  counter = 0;
  // Subscribe to input video feed and publish output video feed
  nh_.getParam("face_cascade", cascade_file_face);
  nh_.getParam("eyes_cascade", cascade_file_eyes);
  nh_.getParam("camera_topic", subscribe_topic);
  nh_.getParam("filtered_face_locations", publish_topic);
  image_sub_ = it_.subscribeCamera(subscribe_topic, 1,&face_detect::Face_Detection::imageCb, this);

  faces_locations = nh_.advertise<cmt_tracker_msgs::Objects>(publish_topic, 10);

  if ( !face_cascade.load(cascade_file_face ))
  { setup = false;  };
  setup = true;

  nh_.getParam("tracker_set_time", time_sec);
   detector = dlib::get_frontal_face_detector();

  nh_.getParam("shape_predictor",shape_predictor_dat);

//  nh_.getParam("camerainfo", camerainfo);
  dlib::deserialize(shape_predictor_dat) >> sp;

  f = boost::bind(&face_detect::Face_Detection::callback, this, _1, _2);
  //TODO initialization doesn't trigger unless the rqt_reconfigure is triggered.

  server.setCallback(f);

}

void Face_Detection::callback(cmt_tracker_msgs::FaceConfig &config, uint32_t level)
{
  //std::cout<<"Factor to be Updated"<<std::endl;
   scale_factor = config.face_scale;
  //std::cout<<"Factor Updated to: "<<factor<<std::endl;
}

inline cv::Point3d toPoint3d(const cv::Vec4d coords)
{
    return cv::Point3d(coords[0], coords[1], coords[2]);
}

inline cv::Vec3d toVec3d(const cv::Vec4d coords)
{
    return cv::Vec3d(coords[0], coords[1], coords[2]);
}

template<typename T> inline cv::Point3d toPoint3d(const T coords)
{
    return Point3d(coords(0,0), coords(1,0), coords(2,0));
}

template<typename T> inline cv::Vec3d toVec3d(const T coords)
{
    return cv::Vec3d(coords(0,0), coords(1,0), coords(2,0));
}

void Face_Detection::imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& camerainfo )
{
cameramodel.fromCameraInfo(camerainfo);

      if(cameramodel.intrinsicMatrix()(0,0) == 0.0) {
            ROS_ERROR("Camera publishes uncalibrated images. Can not estimate face position.");
            ROS_WARN("Detection will start over again when camera info is available.");
    }

  focalLength = cameramodel.fx();
  opticalCenterX = cameramodel.cx();
  opticalCenterY = cameramodel.cy();


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
        std::cerr << "Error in the conversion of the iamge" << std::endl;
        // qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
        // ui_.image_frame->setImage(QImage());
        return;
      }
    }
    std::string vale;
    nh_.getParam("face_detection_method",vale);

    cv::Mat dlib_image;
    cv::Mat ocv_image;

    conversion_mat_.copyTo(dlib_image);
    conversion_mat_.copyTo(ocv_image);
    _debug = conversion_mat_.clone();
    boost::thread thread_1 = boost::thread(&Face_Detection::dlib_detector, this, dlib_image);
    boost::thread thread_2 = boost::thread(&Face_Detection::opencv_detector, this, ocv_image);

    thread_2.join();
    thread_1.join();

    //Now let's filter
    merged_message cv_message = returnNoneOverlappingCV(dlib_faces,opencv_faces);
    for(int i= 0; i < cv_message.message.objects.size(); i++)
    {
        cmt_face_locations.objects.push_back(cv_message.message.objects[i]);
    }
    for(int i= 0; i < cv_message.overlapedDlibCV.size(); i++)
    {
        cmt_face_locations.objects[cv_message.overlapedDlibCV[i]].tool_used_for_detection.data = "dlib_u";
    }
    cmt_face_locations.header.stamp = ros::Time::now();
    faces_locations.publish(cmt_face_locations);
    cmt_face_locations.objects.clear();
    dlib_faces.clear();
    opencv_faces.clear();


  }
  void Face_Detection::dlib_detector(cv::Mat dlib_image)
  {
     dlib::cv_image<dlib::rgb_pixel> cimg(dlib_image);
     dlib::array2d<dlib::rgb_pixel> img;
     dlib::assign_image(img, cimg);
     dlib::pyramid_up(img);
     std::vector<dlib::rectangle> facesd = detector(img);
     std::vector<dlib::full_object_detection> pose_shapes;
     for(size_t i = 0; i < facesd.size(); i++)
     {
        dlib::full_object_detection shape= sp(img,facesd[i]);
        dlib::rectangle face_rect;

        face_rect =  pyd.rect_down(shape.get_rect());

        std::vector<dlib::point> point_down;

        cmt_tracker_msgs::Object face_description;

        for (size_t j = 0; j < shape.num_parts(); j++)
        {
             dlib::point p;

             p = pyd.point_down(shape.part(j));

             point_down.push_back(p);

            //Now let's add this to the feature_point in Object.
            opencv_apps::Point2D pt;
             pt.x = p.x();
             pt.y = p.y();
            face_description.feature_point.points.push_back(pt);
        }
        auto pose = facepose(face_description);
        tf::Transform face_pose;

        auto z = -pose(2,3);

        if (z < 0) continue;

        face_pose.setOrigin( tf::Vector3( pose(0,3), pose(1,3), z));
        tf::Vector3 t( pose(0,3), pose(1,3), z);
        face_description.pose.position.x = pose(0,3);
        face_description.pose.position.y = pose(1,3);
        face_description.pose.position.z = z;
        face_description.tool_used_for_detection.data = "dlib";
        tf::Quaternion qrot;

        tf::Matrix3x3 mrot(
                pose(0,0), pose(0,1), pose(0,2),
                pose(1,0), pose(1,1), pose(1,2),
                pose(2,0), pose(2,1), pose(2,2));

        mrot.getRotation(qrot);
        face_pose.setRotation(qrot);
        tf::quaternionTFToMsg(qrot,face_description.pose.orientation);
//        br.sendTransform(
//                tf::StampedTransform(face_pose,
//                                     ros::Time::now() + ros::Duration(0),
//                                     cameramodel.tfFrame(),
//                                     "face"));
        face_description.header.stamp = ros::Time::now();
        dlib::full_object_detection shape_down(face_rect,point_down);
        pose_shapes.push_back(shape_down); //The Bigger Image Pose Location.

        dlib::rectangle face_loc_;

        face_loc_ = pyd.rect_down(facesd[i]);


        //To Assert only area in face is included to be tracked.

        cv::Rect rect = cv::Rect(cv::Point(face_loc_.left(), face_loc_.top()),cv::Size(face_loc_.width(), face_loc_.height()));

        //now let's resize the element.
        rect = rect & cv::Rect(0, 0, dlib_image.size().width, dlib_image.size().height);
        dlib_faces.push_back(rect);
        face_description.object.x_offset = rect.x;
        face_description.object.y_offset = rect.y;

        face_description.object.height = rect.height;
        face_description.object.width = rect.width;

        counter++;

        face_description.obj_states.data = "Neutral";

        cmt_face_locations.objects.push_back(face_description);
     }
  }
  void Face_Detection::opencv_detector(cv::Mat opencv_img)
  {
    cv::Mat frame_gray;
    cv::cvtColor(opencv_img, frame_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist( frame_gray, frame_gray );

    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 3, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(40, 40) );
    //TODO: namespace mapping to the system.

    for (size_t i = 0; i < faces.size(); i++)
    {
//        cmt_tracker_msgs::Object face_description;
//        face_description.object.x_offset = faces[i].x;
//        face_description.object.y_offset = faces[i].y;
//
//        face_description.object.height = faces[i].height;
//        face_description.object.width = faces[i].width;
        opencv_faces.push_back(cv::Rect(faces[i].x,faces[i].y,faces[i].width,faces[i].height));
//        face_description.object.id.data = counter;
//        counter++;

//        face_description.obj_states.data = "Neutral";

        //cv_face_locations.objects.push_back(face_description);
    }
  }


head_pose Face_Detection::facepose(cmt_tracker_msgs::Object face_description)
{

    cv::Mat projectionMat = cv::Mat::zeros(3,3,CV_32F);
    cv::Matx33f projection = projectionMat;
    projection(0,0) = focalLength;
    projection(1,1) = focalLength;
    projection(0,2) = opticalCenterX;
    projection(1,2) = opticalCenterY;
    projection(2,2) = 1;

    std::vector<cv::Point3f> head_points;

    head_points.push_back(P3D_SELLION);
    head_points.push_back(P3D_RIGHT_EYE);
    head_points.push_back(P3D_LEFT_EYE);
    head_points.push_back(P3D_RIGHT_EAR);
    head_points.push_back(P3D_LEFT_EAR);
    head_points.push_back(P3D_MENTON);
    head_points.push_back(P3D_NOSE);
    head_points.push_back(P3D_STOMMION);

    std::vector<cv::Point2f> detected_points;

    detected_points.push_back(cv::Point2f(face_description.feature_point.points[SELLION].x, face_description.feature_point.points[SELLION].y));
    detected_points.push_back(cv::Point2f(face_description.feature_point.points[RIGHT_EYE].x, face_description.feature_point.points[RIGHT_EYE].y));
    detected_points.push_back(cv::Point2f(face_description.feature_point.points[LEFT_EYE].x, face_description.feature_point.points[LEFT_EYE].y));
    detected_points.push_back(cv::Point2f(face_description.feature_point.points[RIGHT_SIDE].x, face_description.feature_point.points[RIGHT_SIDE].y));
    detected_points.push_back(cv::Point2f(face_description.feature_point.points[LEFT_SIDE].x, face_description.feature_point.points[LEFT_SIDE].y));
    detected_points.push_back(cv::Point2f(face_description.feature_point.points[MENTON].x, face_description.feature_point.points[MENTON].y));
    detected_points.push_back(cv::Point2f(face_description.feature_point.points[NOSE].x, face_description.feature_point.points[NOSE].y));

    cv::Point2f stomion = (cv::Point2f(face_description.feature_point.points[MOUTH_CENTER_TOP].x, face_description.feature_point.points[MOUTH_CENTER_TOP].y)
                         + cv::Point2f(face_description.feature_point.points[MOUTH_CENTER_BOTTOM].x, face_description.feature_point.points[MOUTH_CENTER_BOTTOM].y));
    detected_points.push_back(stomion);

    cv::Mat rvec, tvec;

    // Find the 3D pose of our head
    cv::solvePnP(head_points, detected_points,
            projection, cv::noArray(),
            rvec, tvec, false,
#ifdef OPENCV3
            cv::SOLVEPNP_ITERATIVE);
#else
            cv::ITERATIVE);
#endif

    cv::Matx33d rotation;
    cv::Rodrigues(rvec, rotation);


    head_pose pose = {
        rotation(0,0),    rotation(0,1),    rotation(0,2),    tvec.at<double>(0)/1000,
        rotation(1,0),    rotation(1,1),    rotation(1,2),    tvec.at<double>(1)/1000,
        rotation(2,0),    rotation(2,1),    rotation(2,2),    tvec.at<double>(2)/1000,
                    0,                0,                0,                     1
    };


    return pose;
}
namespace {
cmt_tracker_msgs::Objects convert(std::vector<cv::Rect> faces)
{
  cmt_tracker_msgs::Objects tracker_description;
  for (size_t i = 0; i < faces.size(); i++)
  {

    cmt_tracker_msgs::Object face_description;
    face_description.object.x_offset = faces[i].x;
    face_description.object.y_offset = faces[i].y;
    face_description.object.height = faces[i].height;
    face_description.object.width = faces[i].width;
    face_description.tool_used_for_detection.data = "opencv";

    tracker_description.objects.push_back(face_description);
  }

  return tracker_description;
}

merged_message returnNoneOverlappingCV(std::vector<cv::Rect> dlib_locations, std::vector<cv::Rect> opencv_locs)
{
//This function returns non overlapped Rects for the opencv to be published
merged_message message;
std::vector<cv::Rect> non_overlaped_rects;
std::vector<int> string_val;
  for (int i = 0; i < opencv_locs.size(); i++)
    {
    bool overlap = false;
    for (int j = 0; j < dlib_locations.size(); j++)
    {

     double intersection_area = (opencv_locs[i] & dlib_locations[j]).area();
     double union_area = (opencv_locs[i] | dlib_locations[j]).area();

     overlap = (intersection_area/union_area) > 0.5;
     if (overlap)
     {
     string_val.push_back(j);
     break;
     }
    }

    if(!overlap)
    non_overlaped_rects.push_back(opencv_locs[i]);

    }
    message.message = convert(non_overlaped_rects);
    message.overlapedDlibCV = string_val;
    return message;
}

}


}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_locator");
  face_detect::Face_Detection ic;
  ros::spin();
  return 0;
}
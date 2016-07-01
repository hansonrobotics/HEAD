#include "tracker_plugin.h"



#define SSTR( x ) dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x ) ).str()

namespace rqt_tracker_view {

tracker_plugin::tracker_plugin()
  : rqt_gui_cpp::Plugin(),
    widget_(0), it(nh)
{

  setObjectName("TrackerView");
}

void tracker_plugin::initPlugin(qt_gui_cpp::PluginContext& context)
{

  widget_ = new QWidget();
  ui.setupUi(widget_);


  context.addWidget(widget_);

  img.create(100, 100, CV_8UC3);
  img.setTo(cv::Scalar(0,0,0));


  firstrun = true;
  //ui.face_choice_method->addItem("Hand Selection Trackings");
//  ui.face_choice_method->addItem("Remove on LOST");
//  ui.face_choice_method->addItem("Face Recognition Method");

//  const QStandardItemModel* model = qobject_cast<const QStandardItemModel*>(ui.face_choice_method->model());
//  QStandardItem* item = model->item(2);
//
//  item->setFlags(disable ? item->flags() & ~(Qt::ItemIsSelectable|Qt::ItemIsEnabled) : Qt::ItemIsSelectable|Qt::ItemIsEnabled);
//  item->setData(disable ? ui.face_choice_method->palette().color(QPalette::Disabled, QPalette::Text)
//                      : QVariant(), // clear item data in order to use default color
//              Qt::TextColorRole);

  ui.paramsetters->addItem("Dlib-CMT Method");
  ui.paramsetters->addItem("Show Pose(dlib)");

  nh.getParam("camera_topic", subscribe_topic);
  nh.getParam("filtered_face_locations",subscribe_face);

  face_subscriber = (nh).subscribe(subscribe_face, 1, &rqt_tracker_view::tracker_plugin::list_of_faces_update, this);
  image_subscriber = it.subscribe(subscribe_topic, 1, &rqt_tracker_view::tracker_plugin::imageCb, this);
  tracked_locations = nh.subscribe("tracker_results", 1 , &rqt_tracker_view::tracker_plugin::tracker_resultsCb, this);
  temp_tracked_locations = nh.subscribe("temporary_trackers", 1 , &rqt_tracker_view::tracker_plugin::temp_tracker_resultsCb, this);

  tracker_locations_pub = (nh).advertise<cmt_tracker_msgs::Tracker>("tracking_location", 1);

  client = nh.serviceClient<cmt_tracker_msgs::Clear>("clear");
  image_client = nh.serviceClient<cmt_tracker_msgs::TrackedImages>("get_cmt_rects");
  check_update = nh.serviceClient<cmt_tracker_msgs::Update>("update");
  viewupdate_service  = nh.advertiseService("view_update", &rqt_tracker_view::tracker_plugin::updateTrackerNames,this);
  tracker_locations_sub = (nh).subscribe("tracking_location", 1 , &rqt_tracker_view::tracker_plugin::trackerCb, this);
  nh.getParam("tracking_method", tracking_method);

  ui.removeTracked->setEnabled(false);
  //connect(ui.face_choice_method, SIGNAL(currentIndexChanged(int)), this, SLOT(on_MethodChanged(int)));
  connect(ui.face_output_list, SIGNAL(itemPressed(QListWidgetItem *)), this, SLOT(on_addToTrack_clicked(QListWidgetItem *)));
  connect(ui.removeAllTracked, SIGNAL(pressed()), this, SLOT(on_removeAllTracked_clicked()));
  //connect(ui.removeTracked, SIGNAL(pressed()), this, SLOT(on_removeTracked_clicked()));
  connect(this, SIGNAL(updatefacelist()), this, SLOT(updateVisibleFaces()));
  connect(ui.paramsetters,SIGNAL(currentIndexChanged(int)), this, SLOT(on_ParamChanged(int)));

//  if (tracking_method.compare("handtracking") == 0) ui.face_choice_method->setCurrentIndex(0);
//  else if (tracking_method.compare("mustbeface") == 0) ui.face_choice_method->setCurrentIndex(1);
//  else  ui.face_choice_method->setCurrentIndex(2);



}
bool tracker_plugin::updateTrackerNames(cmt_tracker_msgs::TrackerNames::Request &req, cmt_tracker_msgs::TrackerNames::Response &res)
{
  std::cout<<"Names Updated"<<std::endl;
  nh.setParam("tracker_updated", 2);
  firstrun = true;
  return true;

}
void tracker_plugin::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  nh.getParam("tracker_updated", tracker_updated_num);
  try
  {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      }  else {
        qWarning("callback could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("callback while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      return;
    }
  }
  mat_images.clear();
  face_images.clear();
  cv::Mat image= conversion_mat_.clone();
  faces_image_information.clear();
  for (std::vector<cmt_tracker_msgs::Object>::iterator v = face_locs.objects.begin(); v != face_locs.objects.end() ; ++v)
  {
   int pose;
   nh.getParam("pose",pose);
   if(pose && (*v).feature_point.points.size() == 68)
   {

    for (size_t j = 1 ; j<=16; ++j)
    {
      cv::Point x0;
      x0.x = (*v).feature_point.points[j-1].x;
      x0.y = (*v).feature_point.points[j-1].y;
      cv::Point x1;
      x1.x = (*v).feature_point.points[j].x;
      x1.y = (*v).feature_point.points[j].y;

    cv::line(image,x1,x0,cv::Scalar(255,0,0));
    }
    for (size_t j = 28 ; j<=30; ++j)
    {
      cv::Point x0;
      x0.x = (*v).feature_point.points[j-1].x;
      x0.y = (*v).feature_point.points[j-1].y;
      cv::Point x1;
      x1.x = (*v).feature_point.points[j].x;
      x1.y = (*v).feature_point.points[j].y;

    cv::line(image,x1,x0,cv::Scalar(255,0,0));
    }
    for (size_t j = 18 ; j<=21; ++j)
    {
      cv::Point x0;
      x0.x = (*v).feature_point.points[j-1].x;
      x0.y = (*v).feature_point.points[j-1].y;
      cv::Point x1;
      x1.x = (*v).feature_point.points[j].x;
      x1.y = (*v).feature_point.points[j].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));
    }

    for (size_t j = 23 ; j<=26; ++j)
    {
      cv::Point x0;
      x0.x = (*v).feature_point.points[j-1].x;
      x0.y = (*v).feature_point.points[j-1].y;
      cv::Point x1;
      x1.x = (*v).feature_point.points[j].x;
      x1.y = (*v).feature_point.points[j].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));
    }

    for (size_t j = 31 ; j<=35; ++j)
    {
      cv::Point x0;
      x0.x = (*v).feature_point.points[j-1].x;
      x0.y = (*v).feature_point.points[j-1].y;
      cv::Point x1;
      x1.x = (*v).feature_point.points[j].x;
      x1.y = (*v).feature_point.points[j].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));
    }
     cv::Point x0;
      x0.x = (*v).feature_point.points[35].x;
      x0.y = (*v).feature_point.points[35].y;
     cv::Point x1;
      x1.x = (*v).feature_point.points[30].x;
      x1.y = (*v).feature_point.points[30].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));

    for (size_t j = 37 ; j<=41; ++j)
    {
      cv::Point x0;
      x0.x = (*v).feature_point.points[j-1].x;
      x0.y = (*v).feature_point.points[j-1].y;
      cv::Point x1;
      x1.x = (*v).feature_point.points[j].x;
      x1.y = (*v).feature_point.points[j].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));
    }

      x0.x = (*v).feature_point.points[41].x;
      x0.y = (*v).feature_point.points[41].y;

      x1.x = (*v).feature_point.points[36].x;
      x1.y = (*v).feature_point.points[36].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));

    for (size_t j = 43 ; j<=47; ++j)
    {
      cv::Point x0;
      x0.x = (*v).feature_point.points[j-1].x;
      x0.y = (*v).feature_point.points[j-1].y;
      cv::Point x1;
      x1.x = (*v).feature_point.points[j].x;
      x1.y = (*v).feature_point.points[j].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));
    }

      x0.x = (*v).feature_point.points[47].x;
      x0.y = (*v).feature_point.points[47].y;

      x1.x = (*v).feature_point.points[42].x;
      x1.y = (*v).feature_point.points[42].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));

    for (size_t j = 49 ; j<=59; ++j)
    {
      cv::Point x0;
      x0.x = (*v).feature_point.points[j-1].x;
      x0.y = (*v).feature_point.points[j-1].y;
      cv::Point x1;
      x1.x = (*v).feature_point.points[j].x;
      x1.y = (*v).feature_point.points[j].y;

    cv::line(image,x1,x0,cv::Scalar(255,0,0));
    }

      x0.x = (*v).feature_point.points[48].x;
      x0.y = (*v).feature_point.points[48].y;

      x1.x = (*v).feature_point.points[59].x;
      x1.y = (*v).feature_point.points[59].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));

    for (size_t j = 61 ; j<=67; ++j)
    {
      cv::Point x0;
      x0.x = (*v).feature_point.points[j-1].x;
      x0.y = (*v).feature_point.points[j-1].y;
      cv::Point x1;
      x1.x = (*v).feature_point.points[j].x;
      x1.y = (*v).feature_point.points[j].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));
    }

      x0.x = (*v).feature_point.points[60].x;
      x0.y = (*v).feature_point.points[60].y;

      x1.x = (*v).feature_point.points[67].x;
      x1.y = (*v).feature_point.points[67].y;
    cv::line(image,x1,x0,cv::Scalar(255,0,0));
    }
    cv::Mat mat = image(cv::Rect((*v).object.x_offset, (*v).object.y_offset,
       (*v).object.width, (*v).object.height));
    mat_images.push_back(mat);
    faces_image_information.push_back((*v).tool_used_for_detection.data);
    face_images.push_back(QImage((uchar*) mat_images.back().data, mat_images.back().cols, mat_images.back().rows,
                                 mat_images.back().step[0], QImage::Format_RGB888));
    //emotion.push_back((*v).emotion_states.data);
  }

  tracked_image_mats.clear();
  temp_tracked_image_mats.clear();

  tracked_image_results.clear();
  temp_tracked_image_results.clear();

  tracked_image_information.clear();
  temp_tracked_image_information.clear();

  for (std::vector<cmt_tracker_msgs::Tracker>::iterator v = tracking_results.tracker_results.begin(); v != tracking_results.tracker_results.end() ; ++v)
  {
    std::string previously_known;

    if ((*v).recognized.data)
    {
      previously_known = "as: " + (*v).recognized_name.data;
    }
    else {
      previously_known = "false";
    }


    double division = (double)(*v).active_points.data/(double)(*v).initial_points.data;
    std::string value = "ID-" + (*v).tracker_name.data+ "\nBF Demo: -" + SSTR((*v).before_demotion.data) +"\nOpenFace: " + previously_known +
       "\n" + "Ratio: " + SSTR(division) + "\n" + ";) " + (*v).object.obj_states.data + "\n" +"%: " + SSTR((*v).object.obj_accuracy.data);
    tracked_image_information.push_back( value );


    tracked_image_mats.push_back(image(cv::Rect((*v).object.object.x_offset, (*v).object.object.y_offset,
       (*v).object.object.width, (*v).object.object.height)).clone());


    tracked_image_results.push_back(QImage((uchar*) tracked_image_mats.back().data, tracked_image_mats.back().cols, tracked_image_mats.back().rows,
                                             tracked_image_mats.back().step[0], QImage::Format_RGB888));

  }

  for (std::vector<cmt_tracker_msgs::Tracker>::iterator v = temp_tracking_results.tracker_results.begin(); v != temp_tracking_results.tracker_results.end() ; ++v)
  {
    std::string previously_known;

    if ((*v).recognized.data)
    {
      previously_known = "as: " + (*v).recognized_name.data;
    }
    else {
      previously_known = "not recognzied";
    }

    double division = (double)(*v).active_points.data/(double)(*v).initial_points.data;
    std::string value = "ID-" + (*v).tracker_name.data+ "\nDemotion: -" + SSTR((*v).before_demotion.data) +"\nOpenFace: " + previously_known +
       "\n" + "Ratio: " + SSTR(division) + "\n" + ";) " + (*v).object.obj_states.data + "\n" +"%: " + SSTR((*v).object.obj_accuracy.data);

    temp_tracked_image_information.push_back( value );

    temp_tracked_image_mats.push_back(image(cv::Rect((*v).object.object.x_offset, (*v).object.object.y_offset,
       (*v).object.object.width, (*v).object.object.height)).clone());

    temp_tracked_image_results.push_back(QImage((uchar*) temp_tracked_image_mats.back().data, temp_tracked_image_mats.back().cols, temp_tracked_image_mats.back().rows,
                                             temp_tracked_image_mats.back().step[0], QImage::Format_RGB888));
  }

  if (firstrun)
  {
    firstrun = false;
    cmt_tracker_msgs::TrackedImages results;
    int size_value;
    nh.getParam("tracker_size",size_value);
    if (image_client.call(results))
    {
//      std::cout<<"News Changed"<<std::endl;
      tracked_images.clear();
      tracked_faces.clear();
      tracked_images_names.clear();

      for (std::vector<std::string>::iterator v = results.response.names.begin(); v != results.response.names.end(); ++v)
      {
        tracked_images_names.push_back(*v);
      }
      for (std::vector<sensor_msgs::Image>::const_iterator v = results.response.image.begin(); v != results.response.image.end(); ++v)
      {
        sensor_msgs::Image im = *v;
        sensor_msgs::ImagePtr r = boost::shared_ptr<sensor_msgs::Image>(boost::make_shared<sensor_msgs::Image>(im));
        cv_bridge::CvImageConstPtr cv_ptrs;
        cv::Mat image;
        try {
          cv_ptrs = cv_bridge::toCvShare(r);
          image = cv_ptrs->image;
          cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);
        }
        catch (cv_bridge::Exception& e)
        {
          std::cout << "Error" << std::endl;
          return;
        }
        tracked_images.push_back(image);
        tracked_faces.push_back(QImage((uchar*) tracked_images.back().data, tracked_images.back().cols,
                                       tracked_images.back().rows, tracked_images.back().step[0], QImage::Format_RGB888));
      }
      if(results.response.names.size() == size_value)
        nh.setParam("tracker_updated", 0);
    }

  }
  emit updatefacelist();
}

/**
This function is the one that update hte UI of all things related to the system.
*/
void tracker_plugin::updateVisibleFaces()
{

  ui.face_output_list->clear();
  ui.tracker_output_list->clear();
  ui.tracker_initial_list->clear();
  ui.tracker_output_list_2->clear();
  
  int count_info = 0 ;//Zip Iterator Maybe
  for (std::vector<QImage>::iterator v = face_images.begin(); v != face_images.end(); ++v)
  {
    ui.face_output_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(*v)), QString::fromStdString(faces_image_information[count_info])));
    count_info++;
  }

  count_info = 0;
  for (std::vector<QImage>::iterator v = tracked_faces.begin(); v != tracked_faces.end(); ++v)
  {
    ui.tracker_initial_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(*v)), QString::fromStdString(tracked_images_names[count_info])));
    count_info++;
  }


  count_info = 0;
  for (std::vector<QImage>::iterator v = tracked_image_results.begin(); v != tracked_image_results.end(); ++v)
  {
    ui.tracker_output_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(*v)), QString::fromStdString(tracked_image_information[count_info])));
    count_info++;
  }

  count_info = 0;
  for (std::vector<QImage>::iterator v = temp_tracked_image_results.begin(); v != temp_tracked_image_results.end(); ++v)
  {
    ui.tracker_output_list_2->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(*v)), QString::fromStdString(temp_tracked_image_information[count_info])));
    count_info++;
  }

}
void tracker_plugin::list_of_faces_update(const cmt_tracker_msgs::Objects& faces_info)
{
  face_locs.objects.clear();
  for (std::vector<cmt_tracker_msgs::Object>::const_iterator v = faces_info.objects.begin(); v != faces_info.objects.end(); ++v)
  {
    face_locs.objects.push_back((*v));
  }
}

void tracker_plugin::trackerCb(const cmt_tracker_msgs::Tracker& tracker_locs)
{
  track_published = tracker_locs;
}
void tracker_plugin::tracker_resultsCb(const cmt_tracker_msgs::Trackers& tracker_results)
{
  tracking_results.tracker_results.clear();
  for (std::vector<cmt_tracker_msgs::Tracker>::const_iterator v = tracker_results.tracker_results.begin(); v != tracker_results.tracker_results.end(); ++v)
  {
    tracking_results.tracker_results.push_back(*v);
  }
  
}

void tracker_plugin::temp_tracker_resultsCb(const cmt_tracker_msgs::Trackers& tracker_results)
{
  temp_tracking_results.tracker_results.clear();
  for (std::vector<cmt_tracker_msgs::Tracker>::const_iterator v = tracker_results.tracker_results.begin(); v != tracker_results.tracker_results.end(); ++v)
  {
    temp_tracking_results.tracker_results.push_back(*v);
  }

}

void tracker_plugin::shutdownPlugin()
{
  face_subscriber.shutdown();
  image_subscriber.shutdown();
}

//void tracker_plugin::on_MethodChanged(int index)
//{
//  nh.setParam("tracking_method", "mustbeface");
//  std::cout << "tracking method change 2" << std::endl;
//}
void tracker_plugin::on_ParamChanged(int index)
{
  if(index == 0) {
    nh.setParam("face_detection_method","dlib");

    nh.setParam("pose",0);
  }

  else if(index == 1){
    nh.setParam("pose",1);
    nh.setParam("face_detection_method","dlib");

  }

}
/**
 * @brief tracker_plugin::on_addToTrack_clicked
 * Is a function that set's the parameters for the CMT tracker and also displays it in the viewframe trackedView
 */
void tracker_plugin::on_addToTrack_clicked(QListWidgetItem *item)
{
  int last_selected_item = ui.face_output_list->currentRow();

  track_location.object = face_locs.objects[last_selected_item];

  tracker_locations_pub.publish(track_location);
}
/**
 * @brief tracker_plugin::on_removeAllTracked_clicked
 *
 * is a fucntion that removes all the CMT tracking objects and clears the tracking objects in the scene.
 *
 */
void tracker_plugin::on_removeAllTracked_clicked()
{

  cmt_tracker_msgs::Clear srv;
  client.call(srv);
  // {
     std::cout << "Cleared" << std::endl;
  // }
  // else {
  //   std::cout << "Not Cleared" << std::endl;
  // }
  // ui.tracker_initial_list->clear();
}
/**
 * @brief tracker_plugin::on_removeTracked_clicked
 *
 * Removes selected tracking elements in the trackerView ELements.
 */
void tracker_plugin::on_removeTracked_clicked()
{

}
/**
 * @brief tracker_plugin::on_removeAllElements_clicked
 *
 * This is the fucntion that is placed as inital rough draft of the funciton in the user interface.
 */
void tracker_plugin::on_removeAllElements_clicked()
{

}

}
PLUGINLIB_EXPORT_CLASS(rqt_tracker_view::tracker_plugin, rqt_gui_cpp::Plugin)
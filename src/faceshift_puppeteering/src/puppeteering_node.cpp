#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <signal.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Bool.h>
#include <pau2motors/pau.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include "fsbinarystream.h"
using boost::asio::ip::tcp;

int main(int argc, char* argv[]) {
  bool not_connected = true;
  bool firstrun = true;

  //Include values for the head pose.
//  geometry_msgs::Vector3 eye_left;
//  geometry_msgs::Vector3 eye_right;
//  geometry_msgs::Pose head_pose;
//  blender_api_msgs::FSShapekeys shapekey_pairs;

  //Pau Message to carry everything
  pau2motors::pau faceshift_values;
  //Publish on this message
//  blender_api_msgs::FSValues fs_value;
  fs::fsBinaryStream parserIn, parserOut;
  fs::fsMsgPtr msg;

  //Parameters related to accessing socket
  boost::asio::io_service io_service;
  tcp::resolver resolver(io_service);
  tcp::socket socket(io_service);

  //Parameters for message received and socket configuration.
  size_t len;
  std::string ip_num;
  std::string port_num;

  //Node initalilzation and the socket handler.
  ros::init(argc, argv, "faceshift_to_ros");
  ros::NodeHandle nh;
  ros::Rate r(0.1);

  //Publish the message recieved from the ROS interface to the system.
  ros::Publisher pub_shape = nh.advertise<pau2motors::pau>(
      "/blender_api/faceshift_values", 30);
  try {
    std::vector<std::string> blendshape_names;

    while (ros::ok()) {

      // Moved it here to always wait for connection and avoid restarting the
      // node everytime.
      boost::array<char, 128> buf;
      boost::system::error_code error;
      while (not_connected || error == boost::asio::error::eof ||
             boost::asio::error::connection_reset == error) {
        try {
          // To get the IP and port every time it's run if it didn't connect
          // again
          nh.getParam("IP", ip_num);
          nh.getParam("Port", port_num);

          tcp::resolver::query query(ip_num, port_num);
          tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

          boost::asio::connect(socket, endpoint_iterator);
          ROS_INFO_NAMED("faceshift_to_ros", "Connected");
          not_connected = false;
          firstrun = true;
          break;
        } catch (boost::system::system_error const& e) {
          ROS_WARN_NAMED("faceshift_to_ros",
                         "Error in trying to connect on IP %s and Port %s",
                         ip_num.c_str(), port_num.c_str());
          r.sleep();
        }
      }
      if (firstrun) {
        fs::fsMsgSendBlendshapeNames bln;
        std::string datatosend;
        parserOut.encode_message(datatosend, bln);
        socket.send(boost::asio::buffer(datatosend));
        firstrun = false;
      }

      if (error == boost::asio::error::eof) {
        // Now let's do some topic cleanup to handle the errors that arise in
        // the system.
        ROS_WARN_NAMED("faceshift_to_ros","Error regarding data from socket");
        not_connected = true;
        // Let's unregister publisher.
        continue;  // Connection closed cleanly by peer.
      } else if (error)
        {
        ROS_WARN_NAMED("faceshift_to_ros","Error regarding data from socket");
        not_connected= true;
        throw boost::system::system_error(error);  // Some other error.

        }

      len = socket.read_some(boost::asio::buffer(buf), error);

      parserIn.received(len,
                        buf.data());  // As it taked (long int sz, const *data)

      if(len==0)
      {
      ROS_WARN_NAMED("faceshift_to_ros","Connection Returned Empty.");
      not_connected = true;
      firstrun = true;
      }
      while(msg = parserIn.get_message()) {

        // Get Stream data
        if (dynamic_cast<fs::fsMsgTrackingState*>(msg.get())) {
          fs::fsMsgTrackingState* ts =
              dynamic_cast<fs::fsMsgTrackingState*>(msg.get());
          const fs::fsTrackingData& data = ts->tracking_data();

          // printf ("Time: %f \n", data.m_timestamp);
          // printf ("Tracking Results: %s", data.m_trackingSuccessful ? "true"
          // : "false");
//          fs_value.tracking_status.data= data.m_trackingSuccessful;
//
//          fs_value.head_pose.position.x= data.m_headTranslation.x;
//          fs_value.head_pose.position.y= data.m_headTranslation.y;
//          fs_value.head_pose.position.z= data.m_headTranslation.z;
//
//          fs_value.head_pose.orientation.x = data.m_headRotation.x;
//          fs_value.head_pose.orientation.y = data.m_headRotation.y;
//          fs_value.head_pose.orientation.z = data.m_headRotation.z;
//          fs_value.head_pose.orientation.w = data.m_headRotation.w;

          faceshift_values.m_headTranslation.x= data.m_headTranslation.x;
          faceshift_values.m_headTranslation.y= data.m_headTranslation.y;
          faceshift_values.m_headTranslation.z= data.m_headTranslation.z;

          faceshift_values.m_headRotation.x = data.m_headRotation.x;
          faceshift_values.m_headRotation.y = data.m_headRotation.y;
          faceshift_values.m_headRotation.z = data.m_headRotation.z;
          faceshift_values.m_headRotation.w = data.m_headRotation.w;


          // printf ("head translation: %f %f %f\n", data.m_headTranslation.x,
          // data.m_headTranslation.y, data.m_headTranslation.z);
          // ROS_INFO ("head rotation: %f %f %f %f \n", data.m_headRotation.x,
          // data.m_headRotation.y, data.m_headRotation.z,
          // data.m_headRotation.w);

          faceshift_values.m_eyeGazeLeftPitch= data.m_eyeGazeLeftPitch;
          faceshift_values.m_eyeGazeLeftYaw= data.m_eyeGazeLeftYaw;
          // printf ("Eye Gaze Left Pitch: %f\n", data.m_eyeGazeLeftPitch);
          // printf ("Eye Gaze Left Yaw: %f\n", data.m_eyeGazeLeftYaw);


          faceshift_values.m_eyeGazeRightPitch= data.m_eyeGazeRightPitch;
          faceshift_values.m_eyeGazeRightYaw= data.m_eyeGazeRightYaw;

          // printf ("Eye Gaze Right Pitch: %f\n", data.m_eyeGazeRightPitch);
          // printf ("Eye Gaze Right Yaw: %f\n", data.m_eyeGazeRightYaw);

          std::vector<float> blend_shape = data.m_coeffs;



          if (blendshape_names.size() == blend_shape.size()) {
              faceshift_values.m_shapekeys = blendshape_names; //as they are treated as vectors
              faceshift_values.m_coeffs = data.m_coeffs;
            }


            pub_shape.publish(faceshift_values);
//            fs_value.keys.shapekey.clear();
            //pub_shape.publish(shapekey_pairs);
          } else {
            firstrun = true;  // Cause the number of blendshapes and the rigging
                              // data have now changed in the system.
            // printf("There is a mismatch querying for blendshape names. \n");
            ROS_WARN_NAMED("faceshift_to_ros","There is change in Profile in Faceshift");
          }


        // Get Blendshapes name from the stream.
        if (dynamic_cast<fs::fsMsgBlendshapeNames*>(msg.get())) {
          fs::fsMsgBlendshapeNames* bs =
              dynamic_cast<fs::fsMsgBlendshapeNames*>(msg.get());
          blendshape_names = bs->blendshape_names();
        }
      }

      if (!parserIn.valid()) {
        // printf("parser in invalid state\n");
        parserIn.clear();
      }
      // Now include exit statment and other calibrate commands.
    }
  }
  // handle any exceptions that may have been thrown.
  catch (std::exception& e) {
    ROS_ERROR_NAMED("faceshift_to_ros", "Something Bad happened");
  }

  return 0;
}
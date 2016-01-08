#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <signal.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <blender_api_msgs/FSShapekey.h>
#include <blender_api_msgs/FSShapekeys.h>
#include <blender_api_msgs/AnimationMode.h>

#include "fsbinarystream.h"
using boost::asio::ip::tcp;

int main(int argc, char* argv[]) {
  bool not_connected = true;
  bool firstrun = true;

  blender_api_msgs::FSShapekeys shapekey_pairs;
  fs::fsBinaryStream parserIn, parserOut;
  fs::fsMsgPtr msg;

  boost::asio::io_service io_service;
  tcp::resolver resolver(io_service);
  tcp::socket socket(io_service);

  size_t len;
  std::string ip_num;
  std::string port_num;
  ros::init(argc, argv, "faceshift_to_ros");
  ros::NodeHandle nh;
  ros::Rate r(0.1);
  // TODO
  // Why doesn't unregistered topics don't resolve in the blender API this is
  // just to resolve the problem.
  ros::Publisher pub = nh.advertise<blender_api_msgs::AnimationMode>(
      "/blender_api/set_animation_mode", 30, true);

  ros::Publisher pub_shape = nh.advertise<blender_api_msgs::FSShapekeys>(
      "/blender_api/faceshift_blendshapes_values", 30);
  try {
    std::vector<std::string> blendshape_names;

    // Here is the code that does register the handle to the system to avoid
    // socket terminated without getting any data in ROS.
    blender_api_msgs::AnimationMode mode;
    mode.value = 0;
    pub.publish(mode);

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
          // printf ("head translation: %f %f %f\n", data.m_headTranslation.x,
          // data.m_headTranslation.y, data.m_headTranslation.z);
          // ROS_INFO ("head rotation: %f %f %f %f \n", data.m_headRotation.x,
          // data.m_headRotation.y, data.m_headRotation.z,
          // data.m_headRotation.w);
          // printf ("Eye Gaze Left Pitch: %f\n", data.m_eyeGazeLeftPitch);
          // printf ("Eye Gaze Left Pitch: %f\n", data.m_eyeGazeLeftYaw);
          // printf ("Eye Gaze Left Pitch: %f\n", data.m_eyeGazeRightPitch);
          // printf ("Eye Gaze Left Pitch: %f\n", data.m_eyeGazeRightYaw);

          std::vector<float> blend_shape = data.m_coeffs;

          int counter = 0;

          if (blendshape_names.size() == blend_shape.size()) {
            shapekey_pairs.shapekey.clear();
            for (std::vector<float>::iterator i = blend_shape.begin();
                 i != blend_shape.end(); ++i) {
              blender_api_msgs::FSShapekey skey;
              skey.name = blendshape_names[counter];
              skey.value = *i;
              shapekey_pairs.shapekey.push_back(skey);
              // printf("Blendshape %s : %f\n",
              // blendshape_names[counter].c_str(),  *i);
              counter++;
            }
            pub_shape.publish(shapekey_pairs);
          } else {
            firstrun = true;  // Cause the number of blendshapes and the rigging
                              // data have now changed in the system.
            // printf("There is a mismatch querying for blendshape names. \n");
            ROS_WARN_NAMED("faceshift_to_ros","There is change in Profile in Faceshift");
          }
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
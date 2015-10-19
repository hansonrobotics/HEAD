#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <icog_face_tracker/facebox.h>
#include <icog_face_tracker/faces.h>

using namespace std;
using namespace cv;

CascadeClassifier face_cascade;
ros::Publisher pub;
bool showPreview;

vector<Rect> detectFaces(Mat frame) {
    vector<Rect> faces;
    Mat bufferMat;
    cvtColor(frame, bufferMat, COLOR_BGR2GRAY);
    equalizeHist(bufferMat, bufferMat);
    face_cascade.detectMultiScale(bufferMat, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));
    return faces;
}

void imageCB(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cvPtr;
    try {
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    vector<Rect> faces = detectFaces(cvPtr->image);
    icog_face_tracker::faces faces_msg;
    icog_face_tracker::facebox _facebox;
    faces_msg.image_width = cvPtr->image.cols;
    faces_msg.image_height = cvPtr->image.rows;

    for (int i = 0; i < faces.size(); i++) {
        _facebox.top = faces[i].y;
        _facebox.left = faces[i].x;
        _facebox.width = faces[i].width;
        _facebox.height = faces[i].height;
        faces_msg.face_boxes.push_back(_facebox);
        if (showPreview)
            rectangle(cvPtr->image, faces[i], CV_RGB(100, 100, 255), 1);
    }
    pub.publish(faces_msg);
    if (showPreview) {
        imshow("Live Feed", cvPtr->image);
        waitKey(3);
    }
}

int main(int argc, char **argv) {
    face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml");
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;
    nh.param("/tracker_node/show_preview", showPreview, false);
    pub = nh.advertise<icog_face_tracker::faces>("/faces", 5);
    ros::Subscriber sub = nh.subscribe("/usb_cam_node/image_raw", 1, imageCB);
    ros::spin();
}


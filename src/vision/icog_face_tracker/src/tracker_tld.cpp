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
#include <iostream>
#include <string>
#include "TLD.h"

using namespace std;
using namespace cv;

/* OpenCV Global Vars */
CascadeClassifier face_cascade;

/* ROS Global Vars */
ros::Publisher pub;
bool showPreview;

/* OpenTLD Global Vars */
vector<tld::TLD *> tld_trackers;
int newFaceFrameCounter = 0;

/* Prototypes */
vector<Rect> detectHaarFaces(Mat frame);

vector<Rect> detectTLDFaces(Mat frame);

int getFaceIndex(Rect face, vector<Rect> tldFaces);

void frameCaptured_CB(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv) {
    face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml");
    ros::init(argc, argv, "tracker_tld_node");
    ros::NodeHandle nh;
    nh.param("/tracker_tld_node/show_preview", showPreview, false);
    pub = nh.advertise<icog_face_tracker::faces>("/faces", 5);
    ros::Subscriber sub = nh.subscribe("/usb_cam_node/image_raw", 1, frameCaptured_CB);
    ros::spin();
}

// Detect faces in frame using HaarCascades
vector<Rect> detectHaarFaces(Mat frame) {
    vector<Rect> faces;
    Mat bufferMat;
    cvtColor(frame, bufferMat, COLOR_BGR2GRAY);
    equalizeHist(bufferMat, bufferMat);
    face_cascade.detectMultiScale(bufferMat, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));
    return faces;
}

// Detect faces in current frame based on results from previous frames
vector<Rect> detectTLDFaces(Mat frame) {
    vector<Rect> faces;
    for (int i = 0; i < tld_trackers.size(); i++) {
        tld_trackers[i]->processImage(frame);
        if (tld_trackers[i]->currBB != NULL)
            faces.push_back(*tld_trackers[i]->currBB);
        else
            faces.push_back(Rect(0, 0, 0, 0));
    }
    return faces;
}

// Uses maximum area overlap to determine the closest rectangle
int getFaceIndex(Rect face, vector<Rect> tldFaces) {
    int tldIndex = -1, maxIntersectionArea = 0;
    for (int i = 0; i < tldFaces.size(); i++) {
        int intersectionArea = (face & tldFaces[i]).area();
        if (intersectionArea > maxIntersectionArea) {
            maxIntersectionArea = intersectionArea;
            tldIndex = i;
        }
    }
    return tldIndex;
}

void frameCaptured_CB(const sensor_msgs::ImageConstPtr &msg) {
    // Convert ROS Image to OpenCV Image
    cv_bridge::CvImagePtr cvPtr;
    try {
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Detect faces using HaarCascade and OpenTLD
    vector<Rect> haarFaces = detectHaarFaces(cvPtr->image);
    vector<Rect> tldFaces = detectTLDFaces(cvPtr->image);


    icog_face_tracker::faces faces_msg;
    icog_face_tracker::facebox facebox_msg;
    faces_msg.image_width = cvPtr->image.cols;
    faces_msg.image_height = cvPtr->image.rows;

    Mat frame_gray;
    cvtColor(cvPtr->image, frame_gray, COLOR_BGR2GRAY);

    bool newFaceFound = false;
    // Decide which faces are new
    for (int i = 0; i < haarFaces.size(); i++) {
        int faceIndex = getFaceIndex(haarFaces[i], tldFaces);
        // New face
        if (faceIndex == -1) {
            newFaceFound = true;
            // Threshold of 10 frames to decide if a face is new
            if (newFaceFrameCounter > 10) {
                // Create a new TLD tracker instance
                tld_trackers.push_back(new tld::TLD());
                unsigned long tld_index = tld_trackers.size() - 1;
                tld_trackers.at(tld_index)->detectorCascade->imgWidth = cvPtr->image.cols;
                tld_trackers.at(tld_index)->detectorCascade->imgHeight = cvPtr->image.rows;
                tld_trackers.at(tld_index)->detectorCascade->imgWidthStep = cvPtr->image.rows;
                tld_trackers.at(tld_index)->selectObject(frame_gray, &haarFaces[i]);
                tld_trackers.at(tld_index)->learningEnabled = true;
                faceIndex = tld_trackers.size() - 1;

                // TODO: Remove older tld trackers
            }
        }
        facebox_msg.id = faceIndex;
        facebox_msg.top = haarFaces[i].y;
        facebox_msg.left = haarFaces[i].x;
        facebox_msg.width = haarFaces[i].width;
        facebox_msg.height = haarFaces[i].height;
        faces_msg.face_boxes.push_back(facebox_msg);
        if (showPreview) {
            rectangle(cvPtr->image, haarFaces[i], CV_RGB(100, 100, 255), 1);
            if (facebox_msg.id > -1) {
                stringstream temp;
                temp << "ID: " << facebox_msg.id;
                putText(cvPtr->image, temp.str(), Point(facebox_msg.left, facebox_msg.top),
                        FONT_HERSHEY_COMPLEX_SMALL,
                        0.8, cvScalar(200, 200, 250), 1, CV_AA);
            }
        }

    }

    if (newFaceFound)
        newFaceFrameCounter += 1;
    else
        newFaceFrameCounter = 0;

    pub.publish(faces_msg);

    if (showPreview) {
        imshow("Live Feed", cvPtr->image);
        waitKey(3);
    }
}



#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <math.h>

#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <unistd.h>
#include <vector>
#include <cstdlib>
#include <facial_puppetry/land_marks.h>

using namespace std;
using namespace cv;
using namespace dlib;

//function declaration
std::vector < float >  get_distance(double dP[70][2]);

//holds dlib values including face TOP,LEFT, BOTTOM, RIGHT
double FlandMark[70][2];
char k;
//holds relative LM's values: distance
std::vector<float> distXY;
//holds the maximum possible variation of LM's position
std::vector<float> maxXY;

//custom message that holds dlib values, maximum values, and face width and hight
facial_puppetry::land_marks dlibRaw;


dlib::image_window win;
dlib::frontal_face_detector detector;
dlib::shape_predictor pose_model;
ros::Publisher pub;
//ros::Publisher pub_anim_mode;


// 1st pass normalization and shapekey mapping
std::vector <float> get_distance(double dP[72][2])
{
// Optimization is Required Here: TBD
distXY.push_back(dP[27][1] - (dP[20][1] +  dP[23][1])/2);// 0 . brow center upt
maxXY.push_back((dP[27][1] - (dP[20][1] +  dP[23][1])/2)/2);

distXY.push_back(0.0);   maxXY.push_back(1.0);  //brow center down

distXY.push_back(dP[27][1] - dP[24][1]);//brow_inner_UP.L 2
maxXY.push_back((dP[27][1] - dP[24][1])/3);

distXY.push_back(0.0);   maxXY.push_back(1.0);//'brow_inner_DN.L

distXY.push_back(dP[27][1] - dP[19][1]);//brow_inner_UP.R 4
maxXY.push_back((dP[27][1] - dP[19][1])/3);

distXY.push_back(0.0);   maxXY.push_back(1.0);//brow_inner_DN.R

distXY.push_back(dP[27][1] - dP[24][1]);//brow_outer_UP.L 6
maxXY.push_back((dP[27][1] - dP[24][1])/3);

distXY.push_back(0.0);   maxXY.push_back(1.0);//brow_outer_DN.L

distXY.push_back(dP[27][1] - dP[19][1]);//brow_outer_up.R 8
maxXY.push_back((dP[27][1] - dP[19][1])/3);

distXY.push_back(0.0);   maxXY.push_back(1.0);//brow_outer_DN.R

distXY.push_back((dP[46][1]+dP[47][1])/2 - (dP[43][1]+dP[44][1])/2);//eye-flare.UP.L 10
maxXY.push_back((dP[46][1]+dP[47][1])/2 - (dP[43][1]+dP[44][1])/2);


distXY.push_back((1 / ((dP[46][1]+dP[47][1])/2 - (dP[43][1]+dP[44][1])/2))*100);//eye-blink.UP.L  11      ????
maxXY.push_back(((dP[46][1]+dP[47][1])/2 - (dP[43][1]+dP[44][1])/2)*2.5);


distXY.push_back(((dP[40][1]+dP[41][1])/2 - (dP[37][1]+dP[37][1])/2));//eye-flare.UP.R' 12
maxXY.push_back((dP[40][1]+dP[41][1])/2 - (dP[37][1]+dP[38][1])/2);

distXY.push_back((1 / ((dP[40][1]+dP[41][1])/2 - (dP[37][1]+dP[37][1])/2))*100);//eye-blink.UP.R 13    ???
maxXY.push_back(((dP[40][1]+dP[41][1])/2 - (dP[37][1]+dP[38][1])/2)*2.5);


distXY.push_back((1 / ((dP[46][1]+dP[47][1])/2 - (dP[43][1]+dP[44][1])/2))*100);//eye-blink.LO.L 14    ????
maxXY.push_back(((dP[46][1]+dP[47][1])/2 - (dP[43][1]+dP[44][1])/2)*2.5);


distXY.push_back((dP[46][1]+dP[47][1])/2 - (dP[43][1]+dP[44][1])/2);//eye-flare.LO.L 15
maxXY.push_back(((dP[46][1]+dP[47][1])/2 - (dP[43][1]+dP[44][1])/2)*2);



distXY.push_back((1 / ((dP[40][1]+dP[41][1])/2 - (dP[37][1]+dP[37][1])/2))*100);//eye-blink.LO.R 16     ????
maxXY.push_back(((dP[40][1]+dP[41][1])/2 - (dP[37][1]+dP[38][1])/2)*2.5);


distXY.push_back(((dP[40][1]+dP[41][1])/2 - (dP[37][1]+dP[37][1])/2));//eye-flare.LO.R   17
maxXY.push_back(((dP[46][1]+dP[47][1])/2 - (dP[43][1]+dP[44][1])/2)*2);

//hmmm, odd animation: jumped
distXY.push_back(0.0);   maxXY.push_back(1.0); //wince.L         18
distXY.push_back(0.0);   maxXY.push_back(1.0); //wince.R         19

distXY.push_back((1/(dP[54][1] - dP[45][1])));//sneer.L     20
maxXY.push_back(dP[33][1] - dP[29][1]);

distXY.push_back((1/(dP[48][1] - dP[36][1])));//sneer.R     21
maxXY.push_back(dP[33][1] - dP[29][1]);

/*
pass zero to unmapped shapekeys: eyes-look.dn   eyes-look.dn lip-UP.C.UP lip-UP.C.DN lip-UP.L.UP
                                 lip-UP.L.DN  lip-UP.R.UP  lip-UP.R.DN
*/

for(int lm=0;lm<8;lm++){  distXY.push_back(0.0); maxXY.push_back(1.0);}


distXY.push_back(dP[14][0] - dP[51][0]);//lips-smile.L 30    ????
maxXY.push_back((dP[54][0] - dP[48][0])/2);

distXY.push_back(dP[51][0] - dP[2][0]);//lips-smile.R  31     ???
maxXY.push_back((dP[54][0] - dP[48][0])/2);

distXY.push_back(dP[54][0]-dP[27][0]);//lips-wide.L 32
maxXY.push_back((dP[54][0]-dP[27][0])/3);

distXY.push_back(0.0);   maxXY.push_back(1.0);  //lips-narrow.L


distXY.push_back(dP[27][0]-dP[48][0]);//lips-wide.R 34
maxXY.push_back((dP[27][0]-dP[48][0])/3);


/*
pass zero to unmapped shapekeys: lips-narrow.R    lip-DN.C.DN     lip-DN.L.DN       lip-DN.L.UP
                                 lip-DN.R.DN   lip-DN.R.UP'    lips-frown.L'   lips-frown.R
*/
for(int lm2=0;lm2<9;lm2++){  distXY.push_back(0.0); maxXY.push_back(1.0);}


distXY.push_back((dP[65][1]+dP[66][1] +dP[67][1])/3 - (dP[61][1]+dP[62][1] +dP[63][1])/3);//lip-JAW.DN    ????? 44
maxXY.push_back((dP[57][1] - dP[52][1])*2);

return distXY;
}


//callback function
void dlib_callback(const sensor_msgs::ImageConstPtr& msg) {
//activate pau mode every time this func. is called: shall be handled in a shell script- TBD
//pub_anim_mode.publish(16); //complete facial control

//event handling
//k=cvWaitKey(0);
//if(k == 'A')
//{
//cout<<"Staring ...";
//
//}

cv_bridge::CvImagePtr cvPtr;
try
{
//ros image -> opencv image conversion
cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
} catch (cv_bridge::Exception& e) {
ROS_ERROR("cv_bridge exception: %s", e.what());
return;
}

cv::Mat temp;
temp = cvPtr->image;
dlib::cv_image<dlib::bgr_pixel> cimg(temp);

// Detect Faces
std::vector<dlib::rectangle> faces = detector(cimg);

// Find the pose of each face.
std::vector<dlib::full_object_detection> shapes;

//get pose
for (unsigned long i = 0; i < faces.size(); ++i)
{shapes.push_back(pose_model(cimg, faces[i]));}

// if there is atleast a face
if(shapes.size()>0)
{
for (unsigned long j = 0; j < shapes.size(); ++j) // Iterate through faces
{
const full_object_detection& d = shapes[j]; //get/store face

for (unsigned long k = 0; k < 68; ++k) // get LMs
{
FlandMark[k][0] = (double)d.part(k).x();
FlandMark[k][1] = (double)d.part(k).y();
}
}

//get face coordinates: used to handle change in face position(in z dim: distance of ROI)
//must find a way to handle Rigid motion (Face rotation on y axis)  TBD
FlandMark[68][0] = faces[0].left();
FlandMark[68][1] = faces[0].top();
FlandMark[69][0] = faces[0].right();
FlandMark[69][1] = faces[0].bottom();


//assign values to custom msg attributes
dlibRaw.dlib_val = get_distance(FlandMark); //relative distance
dlibRaw.max_ref = maxXY; //maximum possible change per LM
dlibRaw.distX = FlandMark[68][0]; //face Left
dlibRaw.distY = FlandMark[68][1]; //face Top
dlibRaw.distW = FlandMark[69][0]; //face width
dlibRaw.distH = FlandMark[69][1]; //face bottom

//publish Dlib Data
pub.publish(dlibRaw);

//Clear vcs for the upcoming computation
distXY.clear();
maxXY.clear();
}

//get each face data: will be used during DLIB improvement phase
dlib::array<array2d<rgb_pixel> > face_chips;
extract_image_chips(cimg, get_face_chip_details(shapes), face_chips);

// Camera and Blender Viewer: get this directly inside blender UI or stream the 2d euivalent of 3D model into browser. TBD
win.clear_overlay();
win.set_image(cimg);
win.add_overlay(render_face_detections(shapes));
}


//entry point
int main(int argc, char **argv)
{
detector = dlib::get_frontal_face_detector();
dlib::deserialize("/opt/hansonrobotics/vision/openface/models/dlib/shape_predictor_68_face_landmarks.dat") >> pose_model;
ros::init(argc, argv, "dlib_core_node");
ros::NodeHandle nh;
pub = nh.advertise<facial_puppetry::land_marks>("/dlib_values", 1000);
//pub_anim_mode = nh.advertise<std_msgs::UInt8>("/blender_api/set_animation_mode", 10);
ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, dlib_callback);
ros::spin();
}

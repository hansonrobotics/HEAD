//OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pi_face_tracker/Face.h>
#include <pi_face_tracker/Faces.h>
#include <pi_face_tracker/FaceEvent.h>//
#include <face_id/f_id.h>
#include <face_id/faces_ids.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
//openbr
#include <openbr/openbr_plugin.h>
#include <QMutex>
#include <QString>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <mutex>
using namespace std;
//using namespace cv;

//ASSUMES SAME IMAGE RESOLUTION IMAGE REFERENCE AS CMT TRACKER

//load gallery txt file with file,person_name strings to create a mapping
//subscribe to camera and tracked faces
//if recognized faces in camera image overlap with tracked faces then map them in ros msg

QSharedPointer<br::Transform> transformb;// = br::Transform::fromAlgorithm("FaceRecognition");
QSharedPointer<br::Distance> distanceb;// = br::Distance::fromAlgorithm("FaceRecognition");
std::mutex z_mutex;
std::string gDir,gInfo;

ros::Publisher faces_pub;
br::TemplateList target;

string face_cascade_name = "haarcascade_frontalface_alt.xml";
string gallery_path;
std::map<string,string> file2name;
//String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
cv::CascadeClassifier face_cascade;
//CascadeClassifier eyes_cascade;
std::vector<cv::Rect> Rfaces;
struct face_pix{
  int id;
  int fx;
  int fy;
};
std::vector<face_pix>frv;

class face_obs
{
  const int max_obs=255;
  std::map<int,std::map<string,int>> face_id_score;
  std::map<int,std::map<string,int>>::iterator it;
  std::map<string,int>::iterator it2;
public:
  void remove_id(int fid)
  {
    it=face_id_score.find(fid);
    if (it!=face_id_score.end()){
      face_id_score.erase(it);
    }
  }
  void observed(int fid,string name)
  {
    it=face_id_score.find(fid);
    if (it!=face_id_score.end()){
      it2=face_id_score[fid].find(name);
      if (it2!=face_id_score[fid].end()){
        if (face_id_score[fid][name]<1){
          return;
        }
        face_id_score[fid][name]++;
        if (face_id_score[fid][name]>=max_obs){
          face_id_score[fid][name]=max_obs;
          //turn all other names to 0
          for (it2=face_id_score[fid].begin();it2!=face_id_score[fid].end();++it2)
          {
            if (it2->first!=name)it2->second=0;
          }
        }
      }else{
        face_id_score[fid][name]=1;
      }
    }else{
      std::map<string,int>dat;
      dat[name]=1;
      face_id_score[fid]=dat;
    }
  }//observed
  string get_name_from_id(int fid)
  {
    //loop and get highest scoring name
    int score=0;
    string name="";
    it=face_id_score.find(fid);
    if (it!=face_id_score.end()){
      for (it2=face_id_score[fid].begin();it2!=face_id_score[fid].end();++it2)
      {
        if (it2->second>score){
          score=it2->second;
          name=it2->first;
        }
      }
    }
    return name;
  }
}face_ob;

std::vector<cv::Mat> getFaces(cv::Mat frame_gray)
{
  //
  std::vector<cv::Mat> fcs;
  std::vector<cv::Rect>facer;
  cv::equalizeHist( frame_gray, frame_gray );
  Rfaces.clear();
  facer.clear();
  face_cascade.detectMultiScale( frame_gray, facer, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(64, 64) );//30,30
  for( size_t i = 0; i < facer.size(); i++ )
  {
    cv::Rect rct(facer[i].x-facer[i].width/4,facer[i].y-facer[i].height/4,
      facer[i].width+facer[i].width/2,facer[i].height+facer[i].height/2);
    if (rct.x<0.0)rct.x=0;
    if (rct.y<0.0)rct.y=0;
    if (rct.x+rct.width>639)rct.width=639-rct.x;
    if (rct.y+rct.height>479)rct.height=479-rct.y;
    Rfaces.push_back(rct);

    //cout<<"roi rx: "<<rct.x<<",ry: "<<rct.y<<",rw: "<<rct.width<<", rh: "<<rct.height<<"\n";
    cv::Mat faceROI = frame_gray( rct );
    fcs.push_back(faceROI);
  }
  return fcs;
}

bool is_point_in_rect(int x,int y,cv::Rect rct)
{
  return ((x>=rct.x) && (x<=rct.x+rct.width) && (y>=rct.y) && (y<=rct.y+rct.height));
}

int get_overlap_id(cv::Rect rct)
{
  std::lock_guard<std::mutex> guard(z_mutex);
  for (int  i=0;i<frv.size();i++)
  {
    /*//debug
    std::cout<<"\nfrv index:"<<i<<"x: "<<frv[i].fx<<",y: "<<frv[i].fy<<"\n";
    std::cout<<"rx: "<<rct.x<<",ry: "<<rct.y<<",rw: "<<rct.width<<", rh: "<<rct.height<<"\n";
    std::cout<<"bool: "<<is_point_in_rect(frv[i].fx,frv[i].fy,rct)<<"\n";
    */
    if (is_point_in_rect(frv[i].fx,frv[i].fy,rct))
    {
      return frv[i].id;
    }
  }
  return 0;
}

std::string get_name_from_gallery(int idx)
{
  //check if file.name is same as name in gallery
  std::map<string,string>::iterator it;
  it=file2name.find(target[idx].file.name.toStdString());
  if (it != file2name.end())
    return file2name[target[idx].file.name.toStdString()];
  return "stranger";
}

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
  cv::Mat img = cv_ptr->image;
  cv::Mat img_gray;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  //find faces and pass each face with co-ordinates for recognition
  cv::cvtColor(img, img_gray, CV_BGR2GRAY);
  std::vector<cv::Mat> fcs=getFaces(img_gray);//set faces[index]?
  face_id::faces_ids fc_ids;
  //std::vector<face_id::face_id> fc_ids;
/*
  if (fcs.size()<1){cout<<"no image...\n";return;}
  cout<<"image..."<<fcs.size()<<"\n";
  cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::imshow( "Display window", fcs[0] );                   // Show our image inside it.
  cv::waitKey(1);
  return;
*/
  for( size_t i = 0; i < fcs.size(); i++ )
  {
    cv::Mat mt=fcs[i].clone();//mat needs to be continuous
    br::Template query(mt);
    query >> *transformb;//why error here?
    // Compare templates
    QList<float> scores = distanceb->compare(target, query);
    //FORM ROS MSG
    float score=scores[0];
    int index=0;
    for (int a=1;a<scores.size();a++)
    {
      if (scores[a]>score)
      {
        score=scores[a];
        index=a;
      }
    }
    //update message to send
    face_id::f_id fid;
    //std::cout<<"compare: Rfaces="<<Rfaces.size()<<" Scores="<<scores.size()<<" mats="<<fcs.size()<<"\n";
    fid.id=get_overlap_id(Rfaces[i]);//Rfaces[index] wrong
    fid.name=((score>0.8)?get_name_from_gallery(index):"stranger");
    fid.confidence=score;
    if ((score>0.8) && (fid.id!=0)){
      face_ob.observed(fid.id,fid.name);
      fid.name=face_ob.get_name_from_id(fid.id);
      fc_ids.faces.push_back(fid);
    }
    //std::cout<<"\n"<<target[index].file.name.toStdString()<<"\n";
  }
  //publish message
  if (fc_ids.faces.size()<1) return;
  faces_pub.publish(fc_ids);
  /*
  face_id::faces_ids fi;
  fi.faces=fc_ids; wrong
  faces_pub.publish(fi);
  */
}

string trim(const string& str)
{
    size_t first = str.find_first_not_of(' ');
    if (string::npos == first)
    {
        return str;
    }
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}

void map_image2name(string line)
{
  //split line in 2
  string file,name;
  unsigned int pos = line.find( "," );
  if (pos<1) return;
  file= gDir+"/"+trim(line.substr( 0, pos ));
  name= trim(line.substr(pos+1,line.length()-pos-1));
  file2name[file]=name;
}
void face_event_cb(const pi_face_tracker::FaceEventConstPtr& msg)
{
  if (msg->face_event=="lost_face")
  {
    face_ob.remove_id(msg->face_id);
  }
}
void faces_cb(const pi_face_tracker::FacesConstPtr& msg)
{
  const float pic_width=640.0;//pixels
  const float pic_height=480.0;
  const float fov=1.42;//radians
  //const float fsz=0.17;//meters face width
  const double k_const = (double)pic_width / (double) (2.0*(tan(fov/2.0)));
  float xx,yy,zz;
  face_pix fr;
  //need tracking id and rect
  std::lock_guard<std::mutex> guard(z_mutex);
  frv.clear();
  for (int i=0;i<msg->faces.size();i++)
  {
    //
    fr.id=msg->faces[i].id;
    //std::cout<<"id: "<<fr.id<<"\n";
    //convert x,y to pixels from 3d xyz
    xx=msg->faces[i].point.x;//z
    yy=msg->faces[i].point.y;//x
    zz=msg->faces[i].point.z;//y
    double dp=xx/k_const;
    fr.fy=(pic_height/2.0)-(zz/dp);
    fr.fx=(pic_width/2.0)-(yy/dp);
    //std::cout<<"fx: "<<fr.fx<<", fy: "<<fr.fy<<"\n";
    frv.push_back(fr);
  }
}

void load_name_image_map(string gfile)
{
  //open gallery file from path
  string line;
  ifstream gallery_file(gfile);
  //gallery_file.open();
  while ( getline (gallery_file,line) )
  {
    map_image2name(line);
  }
  gallery_file.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "faceid");
  ros::NodeHandle n;
  Rfaces.clear();
  //param to know path of gallery and gallery description file
  n.param<string>("gallery_dir",gDir,"gallery_dir");
  n.param<string>("gallery_info_file",gInfo,"gallery_dir/gallery_info.txt");
  n.param<string>("haarcascade_frontalface_alt",face_cascade_name,"haarcascade_frontalface_alt.xml");
  if( !face_cascade.load( face_cascade_name ) ){ cout<<"--(!)Error loading face cascade\n"; return -1; };
  //if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };

  //Load image name map
  load_name_image_map(gInfo);
  if (file2name.size()<1){cout<<"--(!)Error loading Gallery Info file.\n"; return -1;};
  //
  br::Context::initialize(argc, argv);

  // Retrieve classes for enrolling and comparing templates using the FaceRecognition algorithm
  transformb = br::Transform::fromAlgorithm("FaceRecognition");
  distanceb = br::Distance::fromAlgorithm("FaceRecognition");

  // Initialize templates
  target = br::TemplateList::fromGallery(gDir.c_str());
  // Enroll templates
  br::Globals->enrollAll = true; // Enroll 0 or more faces per image
  target >> *transformb;
  br::Globals->enrollAll = false;
  //
  ros::Subscriber sub = n.subscribe("/camera/image_raw", 2, image_cb);
  ros::Subscriber sub_face = n.subscribe("/camera/face_locations", 1, &faces_cb);
  //subscribe to face lost
  ros::Subscriber sub_face_events = n.subscribe("/camera/face_event", 50, &face_event_cb);
  //need rect of face
  faces_pub = n.advertise<face_id::faces_ids>("/camera/face_recognition", 1);
  ros::spin();
  br::Context::finalize();
  return 0;
}

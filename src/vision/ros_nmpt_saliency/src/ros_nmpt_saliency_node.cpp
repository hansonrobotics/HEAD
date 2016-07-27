#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <sstream>
#include "BlockTimer.h"
#include "FastSalience.h"
#include "LQRPointTracker.h"
#include "NMPTUtils.h"

#include "geometry_msgs/Point.h"
#include "ros_nmpt_saliency/targets.h"

#include <vector>


using namespace std;
using namespace cv;

	Size imSize(320,240);
	BlockTimer bt;
	FastSalience salTracker;
	LQRPointTracker salientSpot(2);
	vector<double> lqrpt(2,.5);
	Mat im, im2, viz, sal ;
	ros::Publisher pub;
	geometry_msgs::Point pt;
	bool debug_mode;

    /* will put this under  degree.h file */
	double getdegree(double x, double y);//function declaration for degree of saliency


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	double saltime, tottime, degree;
	//	capture >> im2;
   	cv_bridge::CvImagePtr cv_ptr;
	//sensor_msgs::Image salmap_;
   //CvBridge bridge;
   try
   {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        im2=cv_ptr->image;
        /* Transforming down the image*/
        double ratio = imSize.width * 1. / im2.cols;
        resize(im2, im, Size(0,0), ratio, ratio, INTER_NEAREST);

        viz.create(im.rows, im.cols*2, CV_32FC3);

        bt.blockRestart(0);
        vector<KeyPoint> pts;
        salTracker.detect(im, pts);
        saltime = bt.getCurrTime(0) ;

        salTracker.getSalImage(sal);

        double min, max;
        Point minloc, maxloc;
        minMaxLoc(sal, &min, &max, &minloc, &maxloc);

        lqrpt[0] = maxloc.x*1.0 / sal.cols;
        lqrpt[1] = maxloc.y*1.0 / sal.rows;

        salientSpot.setTrackerTarget(lqrpt);

        Mat vizRect = viz(Rect(im.cols,0,im.cols, im.rows));
        cvtColor(sal, vizRect, CV_GRAY2BGR);

        vizRect = viz(Rect(0, 0, im.cols, im.rows));
        im.convertTo(vizRect,CV_32F, 1./256.);

        salientSpot.updateTrackerPosition();
        lqrpt = salientSpot.getCurrentPosition();

        /* assign the geometric coordinates of points identified as salient to the var. pt(Point type)*/
        pt.x=lqrpt[0];
        pt.y=lqrpt[1];
        pt.z=0;
        /* call a function to get the degree of saliency of the current point*/
        degree = getdegree(pt.x, pt.y);


        /* for the purpose of visualizing points above degree 7:the most certain
        green for degree >=7 features, yellow for degree <7 and >=4 ...
         */
        double realx = 320*pt.x;
        double realy = 240*pt.y;

        if(degree >= 7) {       circle(vizRect,cvPoint(realx,realy),5,CV_RGB(0,255,0),-1);        }
        else if(degree >= 4) {       circle(vizRect,cvPoint(realx,realy),5,CV_RGB(128,128,0),-1);        }
        else {       circle(vizRect,cvPoint(realx,realy),5,CV_RGB(255,0,0),-1);        }

        //end: discard me

        ros_nmpt_saliency::targets trg;
        trg.positions.push_back(pt);
        trg.degree=degree;
        pub.publish(trg);
        vizRect = viz(Rect(im.cols,0,im.cols, im.rows));
        cvtColor(sal, vizRect, CV_GRAY2BGR);

        tottime = bt.getCurrTime(1);
        bt.blockRestart(1);

        if (debug_mode){
            imshow("view",viz);
            waitKey(1);
        }

   }
   catch (...)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
}



/* Declared the following variables to handle the values in the degree calculation */
double L,R, T,B; //Bounding Box: point relativeness
double timel =0.0; //Current Time Handler
int counter = 0; //Keeps Track of Newly Identified Salient Point
vector< vector<double> > degreeHandler(10000, vector<double>(5,0)); //Salient Points
double DegreeVal, defaultMax; // holds random degree value during init
double turn_around_time=1;

/* This method computes the degree of salient points identified through time.
- It computes the frequency of identified points over a fixed time interval t.
- The maximum possible freq. per t is used to normzlie the frequency.
- Freq. Values are further normalized b/n 1 and 10 w/ the intetion of putting their degree.
*/
/* put this under degree.cpp and include degree.h here*/
double getdegree(double x, double y)
{
    /*   */
    timel+=bt.getCurrTime(0);
    int flag=0; //for the purpose of switchin to "Add as unique salient point mode"
    double delay; // interval between the last change at a point and now
    double Lturnaround; // time to trigger reconfiguration per each salient point
    double DegreeVal; // to store/pass the final degree value

    /* .
    Take Degree one along w/ points as newly coming salient point if it is first time func. call
    */
    if (not counter)
    {
        degreeHandler[counter][0]=x;
        degreeHandler[counter][1]=y;
        degreeHandler[counter][2]=timel;

        DegreeVal = degreeHandler[counter][3] = 1;
        // Reference for Turn_Around
        degreeHandler[counter][4] = timel;
        degreeHandler[counter][5]=timel;

        ++counter;
    }
    else
    {
        /* In each function call we have to check the belongingness of
        of salient point to the exsiting point                      */
        for(int i=0;i<counter;i++)
        {
            /* creating a bounding rectangle for the upcoming salient point.
            This helps to consider  close enough salient points as one.
            */
            //27 X 21 Pixel Bounding box
         L=degreeHandler[i][0]- 0.09; R=degreeHandler[i][0]+ 0.09;
         T=degreeHandler[i][1]- 0.09; B=degreeHandler[i][1]+ 0.09;
       /* //max degree checking point
            L=degreeHandler[i][0]- 1.0; R=degreeHandler[i][0]+ 1.0;
            T=degreeHandler[i][1]- 1.0; B=degreeHandler[i][1]+ 1.0;
       */


            /* check whether this point relys in the exsiting group or not */
            /* uncomment these two lines and comment the following two lines
            to make per point degree checkup */

            /*if(degreeHandler[i][0] == x  && degreeHandler[i][1] == y) //use this for default comparison for "per point comparision"
            {
            */
            /* Checking the belonginingess of the point*/
            if ((x>=L && y>=T) && (x<=R && y>=T) && (x>=L && y<=B) && (x<=R && y<=B))
            {
                flag=1;
                Lturnaround = timel - degreeHandler[i][4];
                delay = timel - degreeHandler[i][2] ;
                defaultMax = 9.5 * Lturnaround; //considering the performance of core i7 machinew/ 2.34 ghz and w/ pyface tracker : cmt take much resource so better to deduct the value
                /* How long the point identified as salient?  */
                if  (Lturnaround < turn_around_time)
                {
                    /* add the naromalized salience value (will be one if it has high freq. of occurance)*/
                    DegreeVal = (degreeHandler[i][3] += (0.02/delay))* Lturnaround;// maximum possible freq. in time.

                 }
                else
                {

                    DegreeVal=degreeHandler[i][3] =1;
                    degreeHandler[i][4] = timel;
                }
                degreeHandler[i][2] = timel;

                break; // jump out from the loop given the point group is found
            }
            else { continue;  } // loop till it get the end or find the group
        }

        /* New point identified*/
        if (not flag)
        {
            degreeHandler[counter][0]=x;
            degreeHandler[counter][1]=y;
            delay = timel - degreeHandler[counter][2];
            degreeHandler[counter][2]=timel;
            DegreeVal = degreeHandler[counter][3]=0.1;
            degreeHandler[counter][4] = timel;
            degreeHandler[counter][5]=timel;
            ++counter;
        }
    }


    /* put the number in between 1 and 10 */

    int roundD =  ((DegreeVal*10)/defaultMax) + 0.5;
    //cout<<"Degree Raw: "<<DegreeVal<<" Degree Tenth  "<<roundD<<" Possible Raw "<<Lturnaround * 7.5<<endl;
    DegreeVal = roundD;
    return DegreeVal;
}



int main(int argc, char **argv)
   {
        ros::init(argc, argv, "image_listener");
        ros::NodeHandle nh;
        nh.getParam("debug",debug_mode);
        if (debug_mode) cvNamedWindow("view");
        cvStartWindowThread();
        image_transport::ImageTransport it(nh);

        image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);
        pub = nh.advertise<ros_nmpt_saliency::targets>("/nmpt_saliency_point", 50);

        salientSpot.setTrackerTarget(lqrpt);
        bt.blockRestart(1);

        ros::spin();
   }

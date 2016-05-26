/*
 *  OpenCVHaarDetector.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "OpenCVHaarDetector.h"

using namespace cv; 
using namespace std; 

OpenCVHaarDetector::OpenCVHaarDetector(const char* filename) {
	this->filename = filename;  
	//strncpy(this->filename, filename, 5000); 
	//cascade = NULL; 
	//storage = NULL; 
	reInitialize(); 
}

OpenCVHaarDetector::OpenCVHaarDetector(OpenCVHaarDetector* detectorToCopy) {
	this->filename = detectorToCopy->filename; //(char*)malloc(5000); 
	//strncpy(this->filename, detectorToCopy->filename, 5000); 
	//cascade = NULL; 
	//storage = NULL; 
	reInitialize(); 
	
	vjScale = detectorToCopy->vjScale; 
	vjMinSize = detectorToCopy->vjMinSize; 
}

OpenCVHaarDetector::OpenCVHaarDetector() {
	//this->filename = (char*)malloc(5000); 
	//cascade = NULL; 
	//storage = NULL; 
}


void OpenCVHaarDetector::setDetectorSource(string newFileName)  {
	filename = newFileName; 
	if (!cascade.load(filename)) {
		cout << "WARNING: Could not find object detector file " << newFileName << ".\n--->Call setDetectorSource() with a new cascade." << endl; 
	}
}

void OpenCVHaarDetector::reInitialize() {
	
	//cvReleaseHaarClassifierCascade(&(this->cascade)); 
	//cvReleaseMemStorage(&(this->storage)); 
	
	//Get face finder from the application bundle
	//cascade.load(filename); 
	setDetectorSource(filename); 
    //cascade = (CvHaarClassifierCascade*) cvLoad (filename, 0, 0, 0);
	
	//Get scratch memory for face finding. 
    //storage = cvCreateMemStorage(0);
	
	vjScale = 1.1; 
	vjMinSize = 0; 
}

OpenCVHaarDetector::~OpenCVHaarDetector() {
	//cvReleaseHaarClassifierCascade(&(this->cascade)); 
	//cvReleaseMemStorage(&(this->storage)); 
}

//CvSeq* OpenCVHaarDetector::detectObjects(IplImage* image) {
vector<Rect> OpenCVHaarDetector::detectObjects(IplImage* image) {
	vector<Rect> objects; 
	Size minsize(vjMinSize, vjMinSize); 
	
	//Size minsize(30, 30); 
	//minsize.width = vjMinSize; 
	//minsize.height = vjMinSize; 
	cascade.detectMultiScale(image, objects, vjScale, 0, 0, minsize); 
//	return cvHaarDetectObjects( image, cascade, storage,
//						vjScale, 0, 0,  cvSize(vjMinSize, vjMinSize) );
	return objects; 
}

void OpenCVHaarDetector::setHaarCascadeScaleFactor(double factor) {
	vjScale = factor; 
}

void OpenCVHaarDetector::setHaarCascadeMinSize(int size) {
	vjMinSize = size;
}


void OpenCVHaarDetector::readFromStream(istream& in) {
	in >> filename; 
	reInitialize(); 
}

void OpenCVHaarDetector::addToStream(ostream& out) {
	out << filename << endl; 
}
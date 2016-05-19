/*
 *  ImagePatchPyramid.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "ImagePatchPyramid.h"
#include "DebugGlobals.h"
#include <fstream>
#include <stdio.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

using namespace std; 
using namespace cv; 

ImagePatchPyramid::ImagePatchPyramid() {
	minw = 60; 
	minh = 45; 
	
	subImageGridPoints = NULL; 
	actXtoGridCornerX=NULL; 
	actYtoGridCornerY=NULL; 
	pixelXFromGridCornerX=NULL; 
	pixelYFromGridCornerY=NULL; 
	objectCount=NULL;
	objectCountFirstScale=NULL;
	foveaRepresentation=NULL;
	currentScaledImage=NULL;
	detector = NULL; 
}


/**
 * \brief Deep Copy Constructor: Create an IPP that is identical to 
 * the one copied.
 **/
ImagePatchPyramid::ImagePatchPyramid(ImagePatchPyramid* ippToCopy) {
	
	minw = 60; 
	minh = 45; 
	
	this->subImageGridPoints = NULL; 
	this->actXtoGridCornerX=NULL; 
	this->actYtoGridCornerY=NULL; 
	this->pixelXFromGridCornerX=NULL; 
	this->pixelYFromGridCornerY=NULL; 
	this->objectCount=NULL;
	this->objectCountFirstScale=NULL;
	this->foveaRepresentation=NULL;
	this->currentScaledImage=NULL;
	this->detector = NULL; 
	
	this->newImage = ippToCopy->newImage; 
	this->inputImageSize = ippToCopy->inputImageSize; 
	this->subImageSize = ippToCopy->subImageSize; 
	this->gridSize = ippToCopy->gridSize; 
	
	this->repeatFirstScale = ippToCopy->repeatFirstScale; 
	this->observeOff = ippToCopy->observeOff; 
	
	this->minw = ippToCopy->minw; 
	this->minh = ippToCopy->minh; 
	this->numSubImages = ippToCopy->numSubImages; 
	
	if (ippToCopy->detector)
		this->detector = new OpenCVHaarDetector(ippToCopy->detector); 
	
	if (ippToCopy->objectCount)
		this->objectCount = cvCloneImage(ippToCopy->objectCount); 
	
	if (ippToCopy->foveaRepresentation)
		this->foveaRepresentation = cvCloneImage(ippToCopy->foveaRepresentation); //has size of full gray image
	
	if (ippToCopy->currentScaledImage)
		this->currentScaledImage = cvCloneImage(ippToCopy->currentScaledImage); //has size of scaled down image
	
	if (ippToCopy->objectCountFirstScale)
	this->objectCountFirstScale = cvCloneImage(ippToCopy->objectCountFirstScale) ; //has size of grid
	
	
	if (ippToCopy->actXtoGridCornerX)
	this->actXtoGridCornerX = cvCloneMat(ippToCopy->actXtoGridCornerX);    //If a subview is centered at an X/Y location, where does its corner start? 		
	
	if (ippToCopy->actYtoGridCornerY)
		this->actYtoGridCornerY = cvCloneMat(ippToCopy->actYtoGridCornerY);    //If a subview is centered at an X/Y location, where does its corner start? 
	
	if (ippToCopy->pixelXFromGridCornerX)
		this->pixelXFromGridCornerX = cvCloneMat(ippToCopy->pixelXFromGridCornerX);    //[Nx2] matrix maps to [x-corner-start x-corner-end] of grid column x 
	
	if (ippToCopy->pixelYFromGridCornerY)
	this->pixelYFromGridCornerY = cvCloneMat(ippToCopy->pixelYFromGridCornerY);    //[Mx2] matrix maps to [y-corner-start y-corner-end] of grid row y 
	
	if (ippToCopy->subImageGridPoints)
	this->subImageGridPoints = cvCloneMat(ippToCopy->subImageGridPoints);   // [numSubImages x 2] What is the size (width / height) of each scale in terms of grid points? 
	
}

ImagePatchPyramid::ImagePatchPyramid(CvSize inputImageSize, CvSize subImageSize, CvSize gridSize, 
									 int numSubImages, CvMat* subImageGridPoints, OpenCVHaarDetector* detector){
	
	minw = 60;
	minh = 45; 
	
	
	this->subImageGridPoints = NULL; 
	this->actXtoGridCornerX=NULL; 
	this->actYtoGridCornerY=NULL; 
	this->pixelXFromGridCornerX=NULL; 
	this->pixelYFromGridCornerY=NULL; 
	this->objectCount=NULL;
	this->objectCountFirstScale=NULL;
	this->foveaRepresentation=NULL;
	this->currentScaledImage=NULL;
	
	this->inputImageSize = inputImageSize; 
	this->subImageSize = subImageSize; 
	this->gridSize = gridSize; 
	this->numSubImages = numSubImages; 
	this->subImageGridPoints = cvCloneMat(subImageGridPoints); 
	this->detector = detector; 
	
	reInitialize(); 
	
}

void ImagePatchPyramid::reInitialize() {
	cvReleaseMat(&(this->actXtoGridCornerX)); 
	cvReleaseMat(&(this->actYtoGridCornerY)); 
	cvReleaseMat(&(this->pixelXFromGridCornerX)); 
	cvReleaseMat(&(this->pixelYFromGridCornerY)); 
	cvReleaseImage(&(this->objectCount));
	cvReleaseImage(&(this->objectCountFirstScale));
	cvReleaseImage(&(this->foveaRepresentation));
	cvReleaseImage(&(this->currentScaledImage)); 
	
	actXtoGridCornerX = cvCreateMat(gridSize.width, numSubImages, CV_32SC1);   	
	actYtoGridCornerY = cvCreateMat(gridSize.height, numSubImages, CV_32SC1);  
	pixelXFromGridCornerX = cvCreateMat(gridSize.width, 2, CV_32SC1);    
	pixelYFromGridCornerY = cvCreateMat(gridSize.height, 2, CV_32SC1);  
	
    //If a subview is centered at an X/Y location, where does its corner start? 
	for (int scale = 0; scale < numSubImages; scale++) {
		int left = cvGetReal2D(subImageGridPoints, scale, 0)/2;
		int right = cvGetReal2D(subImageGridPoints, scale, 0) - left - 1; 
		for (int pos = 0; pos < gridSize.width; pos++) {
			if (pos-left < 0) { //overflows to left
				cvSetReal2D(actXtoGridCornerX, pos, scale, 0); 
			} else if (pos+right > gridSize.width-1) { //overflows to right
				cvSetReal2D(actXtoGridCornerX, pos, scale, gridSize.width-right-left-1); 
			} else {
				cvSetReal2D(actXtoGridCornerX, pos, scale, pos-left); 
			}			
		}
	}
	
	//If a subview is centered at an X/Y location, where does its corner start?
	for (int scale = 0; scale < numSubImages; scale++) {
		int left = cvGetReal2D(subImageGridPoints, scale, 1)/2;
		int right = cvGetReal2D(subImageGridPoints, scale, 1) - left - 1; 
		for (int pos = 0; pos < gridSize.height; pos++) {
			if (pos-left < 0) { //overflows to left
				cvSetReal2D(actYtoGridCornerY, pos, scale, 0); 
			} else if (pos+right > gridSize.height-1) { //overflows to right
				cvSetReal2D(actYtoGridCornerY, pos, scale, gridSize.height-right-left-1); 
			} else {
				cvSetReal2D(actYtoGridCornerY, pos, scale, pos-left); 
			}			
		}
	}
	
	//Allocate image size, type, memory	
	objectCount = cvCreateImage(cvSize(gridSize.width, gridSize.height), IPL_DEPTH_8U, 1);				
	objectCountFirstScale = cvCreateImage(cvSize(gridSize.width, gridSize.height), IPL_DEPTH_8U, 1);	
	
	foveaRepresentation = NULL; 
	currentScaledImage = NULL; 
	
	//This takes care of a lot of initialization. 
	changeInputImageSize(inputImageSize, subImageSize); 
	
	observeOff = 0; 
	
	repeatFirstScale = (cvGetReal2D(subImageGridPoints,0,0)==gridSize.width && 
						cvGetReal2D(subImageGridPoints,0,1)==gridSize.height); 	
	newImage = 1; 
	
}

ImagePatchPyramid::~ImagePatchPyramid() {
	cvReleaseMat(&(this->subImageGridPoints)); 
	cvReleaseMat(&(this->actXtoGridCornerX)); 
	cvReleaseMat(&(this->actYtoGridCornerY)); 
	cvReleaseMat(&(this->pixelXFromGridCornerX)); 
	cvReleaseMat(&(this->pixelYFromGridCornerY)); 
	cvReleaseImage(&(this->objectCount));
	cvReleaseImage(&(this->objectCountFirstScale));
	cvReleaseImage(&(this->foveaRepresentation));
	cvReleaseImage(&(this->currentScaledImage));  
}

void ImagePatchPyramid::readFromStream(istream& in) {
		
	int rows, cols, type; 
	int matEntry; 
	//double matFloat; 
	
	in >> inputImageSize.width >> inputImageSize.height; 
	in >> subImageSize.width >> subImageSize.height; 
	in >> gridSize.width >> gridSize.height; 
	in >> numSubImages; 
	in >> rows >> cols >> type; 
	
	subImageGridPoints = cvCreateMat(rows, cols, type); 
	for (int y = 0; y < subImageGridPoints->rows; y++) {
		for (int x = 0; x < subImageGridPoints->cols; x++) {
			in >> matEntry; 
			cvSetReal2D(subImageGridPoints, y, x, matEntry) ; 
		}
	}
	
	detector = new OpenCVHaarDetector(); 
	
	//cout << "Read in Haar detector IPP" << endl; 
	in >> detector; 
	//cout << "ReInitialize IPP" << endl; 
	reInitialize();
}

void ImagePatchPyramid::addToStream(ostream& out) {
	out << inputImageSize.width << " " << inputImageSize.height << endl; 
	out << subImageSize.width << " " << subImageSize.height << endl; 
	out << gridSize.width << " " << gridSize.height << endl; 
	out << getNumScales() << endl; 
	out << subImageGridPoints->rows << " " << subImageGridPoints->cols << " " << subImageGridPoints->type << endl; 
	for (int y = 0; y < subImageGridPoints->rows; y++) {
		for (int x = 0; x < subImageGridPoints->cols; x++) {
			out << cvGetReal2D(subImageGridPoints, y, x) << " "; 
		}
		out << endl; 
	}	
	
	out << detector; 
}

void ImagePatchPyramid::changeInputImageSize(CvSize newInputSize) {
	int w1 = newInputSize.width; 
	int h1 = newInputSize.height; 
	
	//This calculation makes it so that the smallest scale has 1:1 mapping of pixels. 
	int numScales = getNumScales(); 
	int smallWidth = getGridCellWidthOfScale(numScales-1);
	int smallHeight = getGridCellHeightOfScale(numScales-1); 
	int w2 = w1 * smallWidth / gridSize.width; //e.g. w1*3/21
	int h2 = h1 * smallHeight / gridSize.height; //e.g. h1*3/21			
	
	changeInputImageSize(cvSize(w1, h1), cvSize(w2,h2)); 
}

void ImagePatchPyramid::changeInputImageSize(CvSize newInputSize, CvSize newSubImageSize){
	inputImageSize = newInputSize; 
	//int w1 = newInputSize.width; 
	//int h1 = newInputSize.height; 
	int w2 = newSubImageSize.width; 
	int h2 = newSubImageSize.height; 
	/*
	if (w2 < h2) {
		int temp = minw; 
		minw = minh; 
		minh = temp; 
	}*/
	
	if ((w2 < h2 && minh < minw) || (h2<w2 && minw<minh)) {
		int temp = minw; 
		minw = minh; 
		minh = temp; 
	}
	
	numSubImages = subImageGridPoints->rows; 
	
	//cerr << "----" << numSubImages << ", (" << w2 << ", " << h2 
	//<< ") / (" << newInputSize.width << ", " << newInputSize.height << ")" << endl; 
	while (numSubImages > 1 && (w2<minw || h2 < minh)) {
		int thiswidth = cvGetReal2D(subImageGridPoints, numSubImages-1, 0); 
		int nextwidth = cvGetReal2D(subImageGridPoints, numSubImages-2, 0); 
		int thisheight = cvGetReal2D(subImageGridPoints, numSubImages-1, 1); 
		int nextheight = cvGetReal2D(subImageGridPoints, numSubImages-2, 1); 
		double wratio = 1.0*nextwidth/thiswidth; 
		double hratio = 1.0*nextheight/thisheight; 
		w2 = ceil(w2*wratio); 
		h2 = ceil(h2*hratio); 
		numSubImages--; 
		//cerr << "----" << numSubImages << ", (" << w2 << ", " << h2 << ")" <<  endl; 
	}
	
	subImageSize = cvSize(w2,h2); 
	
	
	
	/*Reset Pixel-Grid Mapping*/
	//[Nx2] matrix maps to [x-corner-start x-corner-end] of grid column x 
	for (int i = 0; i < gridSize.width; i++) {
		//left corner pixel of grid cell
		cvSetReal2D(pixelXFromGridCornerX, i, 0, inputImageSize.width * i / gridSize.width); 
		//right corner pixel of grid cell is just previous to the next cell
		if (i > 0) 
			cvSetReal2D(pixelXFromGridCornerX, i-1, 1, cvGetReal2D(pixelXFromGridCornerX, i, 0)-1); 
	}
	//last grid cell end is at end of image
	cvSetReal2D(pixelXFromGridCornerX, gridSize.width-1, 1, inputImageSize.width-1); 	
	
	//[Mx2] matrix maps to [y-corner-start y-corner-end] of grid row y 
	for (int i = 0; i < gridSize.height; i++) {
		//left corner pixel of grid cell
		cvSetReal2D(pixelYFromGridCornerY, i, 0, inputImageSize.height * i / gridSize.height); 
		//right corner pixel of grid cell is just previous to the next cell
		if (i > 0) 
			cvSetReal2D(pixelYFromGridCornerY, i-1, 1, cvGetReal2D(pixelYFromGridCornerY, i, 0)-1); 
	}
	//last grid cell end is at end of image
	cvSetReal2D(pixelYFromGridCornerY, gridSize.height-1, 1, inputImageSize.height-1); 	
	
	
	//Allocate image size, type, memory
	if (foveaRepresentation != NULL) 
		cvReleaseImage(&foveaRepresentation);
	
	if (currentScaledImage != NULL)
		cvReleaseImage(&currentScaledImage); 
	
	//cvReleaseImage(&obsProbScratchPad); 
	
	foveaRepresentation = cvCreateImage(cvSize(inputImageSize.width, inputImageSize.height), IPL_DEPTH_8U, 1);
	currentScaledImage = cvCreateImage(cvSize(subImageSize.width, subImageSize.height), IPL_DEPTH_8U, 1);
	
	//Set others to zero just to be clean
	cvSetZero(foveaRepresentation); 
	cvSetZero(currentScaledImage); 	
	cvSetZero(objectCount); 
	cvSetZero(objectCountFirstScale); 
	
	newImage = 1; 
}

CvPoint ImagePatchPyramid::searchHighResImage(IplImage* grayFrame){
	//CvSeq* objects = detector->detectObjects(grayFrame); 
	vector<Rect> objects = detector->detectObjects(grayFrame); 
	
	cvSetZero(objectCount); 
	
	//for (int j = 0; j < (objects ? objects->total : 0); j++){
	
	for(vector<Rect>::const_iterator r = objects.begin(); r != objects.end(); r++)
	{
		//CvRect* r = (CvRect*) cvGetSeqElem (objects, j);
		
		int x = (r->x +r->width / 2) * gridSize.width / inputImageSize.width ; 
		int y = (r->y +r->height /2) * gridSize.height / inputImageSize.height; 
		int currCount = cvGetReal2D(objectCount, y, x); 
		cvSetReal2D(objectCount, y, x, currCount+1); 
	}
	
	double min, max; 
	CvPoint min_loc, max_loc; 
	cvMinMaxLoc( objectCount, &min, &max, &min_loc, &max_loc );
	
	if (!observeOff) {
		cvCopy(grayFrame, foveaRepresentation); 
		for (int i = 0; i < gridSize.width; i++) {
			int first = cvGetReal2D(pixelXFromGridCornerX, i, 0); 
			int second = cvGetReal2D(pixelXFromGridCornerX, i, 1); 
			cvRectangle(foveaRepresentation, //draw a box around this scale
						cvPoint(first, 0), 	cvPoint(first, inputImageSize.height-1), 
						cvRealScalar(0), 1 ); 			
			cvRectangle(foveaRepresentation, //draw a box around this scale
						cvPoint(second, 0), 	cvPoint(second, inputImageSize.height-1), 
						cvRealScalar(0), 1 ); 			
		}
		
		for (int i = 0; i < gridSize.height; i++) {
			int first = cvGetReal2D(pixelYFromGridCornerY, i, 0); 
			int second = cvGetReal2D(pixelYFromGridCornerY, i, 1); 
			cvRectangle(foveaRepresentation, //draw a box around this scale
						cvPoint(0, first), 	cvPoint(inputImageSize.width-1,first), 
						cvRealScalar(0), 1 ); 			
			cvRectangle(foveaRepresentation, //draw a box around this scale
						cvPoint(0, second), cvPoint(inputImageSize.width-1, second), 
						cvRealScalar(0), 1 ); 			
		}
	}
	
	return pixelForGridPoint(max_loc); 
} 


CvPoint ImagePatchPyramid::gridPointForPixel(CvPoint pixel){ 
	CvPoint retval; 
	retval.x = pixel.x * gridSize.width / inputImageSize.width; 
	retval.y = pixel.y * gridSize.height / inputImageSize.height; 
	return retval; 
}

CvPoint ImagePatchPyramid::pixelForGridPoint(CvPoint gridPoint) {
	CvPoint retval; 
	retval.x = (gridPoint.x + .5)*inputImageSize.width / gridSize.width; 
	retval.y = (gridPoint.y + .5)*inputImageSize.height / gridSize.height; 
	return retval; 
}

CvRect ImagePatchPyramid::getVisibleRegion(CvPoint searchPoint) {
	int scale = 0; 
	int gridStartX = cvGetReal2D( actXtoGridCornerX, searchPoint.x, scale); 
	int gridStartY = cvGetReal2D( actYtoGridCornerY, searchPoint.y, scale); 
	return cvRect(gridStartX, gridStartY, 
								 cvGetReal2D(subImageGridPoints, scale, 0), 
								 cvGetReal2D(subImageGridPoints, scale, 1)); 
}

//void ImagePatchPyramid::countObjects(CvSeq* objects, CvPoint searchPoint, int scale) {

void ImagePatchPyramid::countObjects(vector<Rect> objects, CvPoint searchPoint, int scale) {
	
	if (searchPoint.x < 0 || searchPoint.y < 0) 
		cout << "Bad Search Point: " << searchPoint.x << ", " <<searchPoint.y << endl; 
	
	int gridStartX = cvGetReal2D( actXtoGridCornerX, searchPoint.x, scale); 
	int gridStartY = cvGetReal2D( actYtoGridCornerY, searchPoint.y, scale); 
	CvRect gridImageROI = cvRect(gridStartX, gridStartY, 
								 cvGetReal2D(subImageGridPoints, scale, 0), 
								 cvGetReal2D(subImageGridPoints, scale, 1)); 
	
	//for (int j = 0; j < (objects ? objects->total : 0); j++)
	for(vector<Rect>::const_iterator r = objects.begin(); r != objects.end(); r++)
	{
		//CvRect* r = (CvRect*) cvGetSeqElem (objects, j);
		
		int x = (r->x +r->width / 2) * gridImageROI.width / subImageSize.width +gridStartX; 
		int y = (r->y +r->height /2) * gridImageROI.height / subImageSize.height+gridStartY; 
		int currCount = cvGetReal2D(objectCount, y, x); 
		cvSetReal2D(objectCount, y, x, currCount+1); 
	}	
}


void ImagePatchPyramid::setObjectDetectorSource(string newFileName) {
	detector->setDetectorSource(newFileName); 
}

CvRect ImagePatchPyramid::getFullImageROIForPointAtScale(CvPoint searchPoint, int scale) {
	int gridStartX = cvGetReal2D( actXtoGridCornerX, searchPoint.x, scale); 
	int gridStartY = cvGetReal2D( actYtoGridCornerY, searchPoint.y, scale); 
	int gridEndX = gridStartX + cvGetReal2D(subImageGridPoints, scale, 0) - 1; 
	int gridEndY = gridStartY + cvGetReal2D(subImageGridPoints, scale, 1) - 1; 
	
	int pixelStartX = cvGetReal2D(pixelXFromGridCornerX, gridStartX, 0); 
	int pixelStartY = cvGetReal2D(pixelYFromGridCornerY, gridStartY, 0); 
	int pixelEndX = cvGetReal2D(pixelXFromGridCornerX, gridEndX, 1); 
	int pixelEndY = cvGetReal2D(pixelYFromGridCornerY, gridEndY, 1); 
	
	return cvRect(pixelStartX, pixelStartY, pixelEndX - pixelStartX, pixelEndY - pixelStartY); 
}

void ImagePatchPyramid::setNewImage() {
	newImage = 1; 
	//cvSetZero(objectCountFirstScale); 
}

void ImagePatchPyramid::searchFrameAtGridPoint(IplImage* grayFrame, CvPoint searchPoint) {

	if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: Beginning search of frame at grid point " << searchPoint.x << ", " << searchPoint.y << endl; 
	//If we are searching the same frame as before...
	int start = 0; 
	if (!newImage && repeatFirstScale) {
		if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: copying first scale from previous search." << endl; 
		
		cvCopy(objectCountFirstScale, objectCount); 
		start = 1; 
	}
	else {
		
		if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: Setting object counts to zero." << endl;
		cvSetZero(objectCount); 
	}
	
	if (!observeOff) {		
		
		if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: clearing visualization image." << endl;
		cvSetZero(foveaRepresentation); 
	}
	
	for (int scale = start; scale < numSubImages; scale++) {
		//Scale down image region of interest at the current scale
		
		if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: getting roi for search point at scale " << scale << "." << endl;
		CvRect fullImageROI = getFullImageROIForPointAtScale(searchPoint, scale); 
		
		cvSetImageROI( grayFrame,  fullImageROI ); 
		
		
		if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: Scaling down image." << endl;
		cvResize (grayFrame, currentScaledImage, CV_INTER_NN);  //scale foveated region of image down to this scale		
		cvResetImageROI(grayFrame); 
		if (!observeOff) {
			
			if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: scaling image back up for fovea representation." << endl;
			cvSetImageROI( foveaRepresentation,  fullImageROI ); 
			cvResize(currentScaledImage, foveaRepresentation, CV_INTER_NN); //scale it back up to show reduced resolution
			
			cvResetImageROI(foveaRepresentation); 
			
			cvRectangle(foveaRepresentation, //draw a box around this scale
						cvPoint(fullImageROI.x-2, fullImageROI.y-2), 
						cvPoint(fullImageROI.x+2+fullImageROI.width, fullImageROI.y+2+fullImageROI.height), 
						cvRealScalar(255), 5 ); 	
		}
		
		
		if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: applying object detector to the current image." << endl;
		//CvSeq* objects = detector->detectObjects( currentScaledImage);
		vector<Rect> objects = detector->detectObjects( currentScaledImage);
		
		if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: counting detector outputs in each grid cell." << endl;
		countObjects(objects, searchPoint, scale); 	//Fill "objectCount", the basic I-POMDP observation vector.
		
		if (scale==0 && repeatFirstScale) {
			
			if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: caching counts of first scale for repeated search." << endl;
			cvCopy(objectCount, objectCountFirstScale); 
			newImage = 0; 
		}
		
		//cvReleaseSeq(&objects); 
		
		if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: clearing face search sequence." << endl;
		//cvClearSeq(objects); 
		
	}
	
	if (!observeOff) {
		
		if (_IPP_DEBUG) cout << "searchFrameAtGridPoint: drawing grid over fovea representation." << endl;
		for (int i = 0; i < gridSize.width; i++) {
			int first = cvGetReal2D(pixelXFromGridCornerX, i, 0); 
			int second = cvGetReal2D(pixelXFromGridCornerX, i, 1); 
			cvRectangle(foveaRepresentation, //draw a box around this scale
						cvPoint(first, 0), 	cvPoint(first, inputImageSize.height-1), 
						cvRealScalar(0), 1 ); 			
			cvRectangle(foveaRepresentation, //draw a box around this scale
						cvPoint(second, 0), 	cvPoint(second, inputImageSize.height-1), 
						cvRealScalar(0), 1 ); 			
		}
		
		for (int i = 0; i < gridSize.height; i++) {
			int first = cvGetReal2D(pixelYFromGridCornerY, i, 0); 
			int second = cvGetReal2D(pixelYFromGridCornerY, i, 1); 
			cvRectangle(foveaRepresentation, //draw a box around this scale
						cvPoint(0, first), 	cvPoint(inputImageSize.width-1,first), 
						cvRealScalar(0), 1 ); 			
			cvRectangle(foveaRepresentation, //draw a box around this scale
						cvPoint(0, second), cvPoint(inputImageSize.width-1, second), 
						cvRealScalar(0), 1 ); 			
		}
	}
}



void ImagePatchPyramid::saveVisualization(IplImage* grayFrame, CvPoint searchPoint, const char* base_filename) { 	
		
	char filename[5000]; 
	snprintf(filename, 5000*sizeof(char), "%s-FullInputImage.png", base_filename); 
	cvSaveImage(filename, grayFrame); 
	
	cvSetZero(objectCount); 
	cvSetZero(foveaRepresentation); 
	
	for (int scale = 0; scale < numSubImages; scale++) {
		//Scale down image region of interest at the current scale
		CvRect fullImageROI = getFullImageROIForPointAtScale(searchPoint, scale); 
		
		cvSetImageROI( grayFrame,  fullImageROI ); 
		cvSetImageROI( foveaRepresentation,  fullImageROI ); 
		
		cvResize (grayFrame, currentScaledImage, CV_INTER_LINEAR);  //scale foveated region of image down to this scale
		cvResize(currentScaledImage, foveaRepresentation, CV_INTER_NN); //scale it back up to show reduced resolution
		
		snprintf(filename, 5000*sizeof(char), "%s-Scale-%d.png", base_filename, scale); 
		cvSaveImage(filename, currentScaledImage); 
		
		cvResetImageROI(grayFrame); 
		
		cvResetImageROI(foveaRepresentation); 
		
		//CvSeq* objects = detector->detectObjects( currentScaledImage);
		vector<Rect> objects = detector->detectObjects(currentScaledImage); 
		
		countObjects(objects, searchPoint, scale); 	//Fill "objectCount", the basic I-POMDP observation vector.
		
		
		//cvReleaseSeq(&objects); 
		//cvClearSeq(objects); 
		
	}
	
	snprintf(filename, 5000*sizeof(char), "%s-FoveatedInputImage.png", base_filename); 
	cvSaveImage(filename, foveaRepresentation); 
	
	
	for (int scale = 0; scale < numSubImages; scale++) {
		//Scale down image region of interest at the current scale
		CvRect fullImageROI = getFullImageROIForPointAtScale(searchPoint, scale); 
		
		
		cvRectangle(foveaRepresentation, //draw a box around this scale
					cvPoint(fullImageROI.x-2, fullImageROI.y-2), 
					cvPoint(fullImageROI.x+2+fullImageROI.width, fullImageROI.y+2+fullImageROI.height), 
					cvRealScalar(255), 5 ); 	
		
	}
	
	snprintf(filename, 5000*sizeof(char), "%s-FoveatedInputImageWithLooking.png", base_filename); 
	cvSaveImage(filename, foveaRepresentation); 
	
	snprintf(filename, 5000*sizeof(char), "%s-FaceCounts.csv", base_filename); 
	ofstream out; 
	out.open(filename); 
	for (int i = 0; i < objectCount->height; i++) {
		for (int j = 0; j < objectCount->width; j++) {
			out << cvGetReal2D(objectCount, i, j) << ", "; 
		}
		out << endl; 
	}
	out.close(); 
		
		
	for (int i = 0; i < gridSize.width; i++) {
		int first = cvGetReal2D(pixelXFromGridCornerX, i, 0); 
		int second = cvGetReal2D(pixelXFromGridCornerX, i, 1); 
		cvRectangle(foveaRepresentation, //draw a box around this scale
					cvPoint(first, 0), 	cvPoint(first, inputImageSize.height-1), 
					cvRealScalar(0), 1 ); 			
		cvRectangle(foveaRepresentation, //draw a box around this scale
					cvPoint(second, 0), 	cvPoint(second, inputImageSize.height-1), 
					cvRealScalar(0), 1 ); 			
	}
	
	for (int i = 0; i < gridSize.height; i++) {
		int first = cvGetReal2D(pixelYFromGridCornerY, i, 0); 
		int second = cvGetReal2D(pixelYFromGridCornerY, i, 1); 
		cvRectangle(foveaRepresentation, //draw a box around this scale
					cvPoint(0, first), 	cvPoint(inputImageSize.width-1,first), 
					cvRealScalar(0), 1 ); 			
		cvRectangle(foveaRepresentation, //draw a box around this scale
					cvPoint(0, second), cvPoint(inputImageSize.width-1, second), 
					cvRealScalar(0), 1 ); 			
	}
	
	
	snprintf(filename, 5000*sizeof(char), "%s-FoveatedInputImageWithGrid.png", base_filename); 
	cvSaveImage(filename, foveaRepresentation); 
	
	cvCopy(grayFrame, foveaRepresentation); 
	
	for (int i = 0; i < gridSize.width; i++) {
		int first = cvGetReal2D(pixelXFromGridCornerX, i, 0); 
		int second = cvGetReal2D(pixelXFromGridCornerX, i, 1); 
		cvRectangle(foveaRepresentation, //draw a box around this scale
					cvPoint(first, 0), 	cvPoint(first, inputImageSize.height-1), 
					cvRealScalar(0), 1 ); 			
		cvRectangle(foveaRepresentation, //draw a box around this scale
					cvPoint(second, 0), 	cvPoint(second, inputImageSize.height-1), 
					cvRealScalar(0), 1 ); 			
	}
	
	for (int i = 0; i < gridSize.height; i++) {
		int first = cvGetReal2D(pixelYFromGridCornerY, i, 0); 
		int second = cvGetReal2D(pixelYFromGridCornerY, i, 1); 
		cvRectangle(foveaRepresentation, //draw a box around this scale
					cvPoint(0, first), 	cvPoint(inputImageSize.width-1,first), 
					cvRealScalar(0), 1 ); 			
		cvRectangle(foveaRepresentation, //draw a box around this scale
					cvPoint(0, second), cvPoint(inputImageSize.width-1, second), 
					cvRealScalar(0), 1 ); 			
	}
	
	
	snprintf(filename, 5000*sizeof(char), "%s-FullInputImageWithGrid.png", base_filename); 
	cvSaveImage(filename, foveaRepresentation); 
	
	cvCopy(grayFrame, foveaRepresentation); 
	
	for (int scale = 0; scale < numSubImages; scale++) {
		//Scale down image region of interest at the current scale
		CvRect fullImageROI = getFullImageROIForPointAtScale(searchPoint, scale); 
		
		
		cvRectangle(foveaRepresentation, //draw a box around this scale
					cvPoint(fullImageROI.x-2, fullImageROI.y-2), 
					cvPoint(fullImageROI.x+2+fullImageROI.width, fullImageROI.y+2+fullImageROI.height), 
					cvRealScalar(255), 5 ); 	
		
	}
	
	
	snprintf(filename, 5000*sizeof(char), "%s-FullInputImageWithLooking.png", base_filename); 
	cvSaveImage(filename, foveaRepresentation); 
}

void ImagePatchPyramid::setGeneratePreview(int flag) {observeOff = !flag;} 
int ImagePatchPyramid::getGeneratePreview() {return !observeOff;} 
void ImagePatchPyramid::setMinSize(CvSize minsize) {
	minw = minsize.width; 
	minh = minsize.height;
}
void ImagePatchPyramid::useSameFrameOptimizations(int flag) {repeatFirstScale = flag; }
int ImagePatchPyramid::getSameFrameOptimizations() {return repeatFirstScale;}


CvSize ImagePatchPyramid::getInputImageSize() {return inputImageSize;}
CvSize ImagePatchPyramid::getSubImageSize(){return subImageSize; }

int ImagePatchPyramid::getNumScales(){return  subImageGridPoints->rows; } 
int ImagePatchPyramid::getUsedScales(){return  numSubImages; } 
int ImagePatchPyramid::getGridCellHeightOfScale(int i) {return (int)cvGetReal2D(subImageGridPoints, i, 1);} 
int ImagePatchPyramid::getGridCellWidthOfScale(int i) {return (int)cvGetReal2D(subImageGridPoints, i, 0);} 


ostream& operator<< (ostream& ofs, ImagePatchPyramid* model) {
	model->addToStream(ofs); 
	return ofs; 
}
istream& operator>> (istream& ifs, ImagePatchPyramid* model) {
	model->readFromStream(ifs); 
	return ifs; 
}
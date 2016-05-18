/*
 *  OpenCV2BoxFilter.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/22/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "OpenCV2BoxFilter.h"

using namespace std; 
using namespace cv; 

OpenCV2BoxFilter::OpenCV2BoxFilter(int maxPaddingRequired, int filterType){
	this->maxPaddingRequired = maxPaddingRequired; 
	this->filterType = filterType; 
}

OpenCV2BoxFilter::OpenCV2BoxFilter(){
	this->maxPaddingRequired = 0; 
	this->filterType = CV_32F; 
}


OpenCV2BoxFilter & OpenCV2BoxFilter::operator=(const OpenCV2BoxFilter &rhs) {
	if (this != &rhs) {
		maxPaddingRequired = rhs.maxPaddingRequired; 
		filterType = rhs.filterType; 
		integralImage = rhs.integralImage.clone(); 
		zeroPaddedScratchImage = rhs.zeroPaddedScratchImage.clone(); 
	}
	return *this; 
}


OpenCV2BoxFilter::OpenCV2BoxFilter(const OpenCV2BoxFilter &rhs) {
	if (this != &rhs) {
		maxPaddingRequired = rhs.maxPaddingRequired; 
		filterType = rhs.filterType; 
		integralImage = rhs.integralImage.clone(); 
		zeroPaddedScratchImage = rhs.zeroPaddedScratchImage.clone(); 
	} 
}


OpenCV2BoxFilter::~OpenCV2BoxFilter(){
}

void OpenCV2BoxFilter::setNewImage(const Mat &imageToFilter){
	size_t imageWidth = imageToFilter.cols; 
	size_t imageHeight = imageToFilter.rows; 
	
	Size padSize = cv::Size(imageWidth+2*maxPaddingRequired, imageHeight+2*maxPaddingRequired); 
	
	Rect zeroPaddedScratchROI = Rect(maxPaddingRequired, maxPaddingRequired, imageWidth, imageHeight); 
	
	if (zeroPaddedScratchImage.cols != padSize.width || zeroPaddedScratchImage.rows != padSize.height || 
		zeroPaddedScratchImage.type() != imageToFilter.type()) {
		zeroPaddedScratchImage = Mat::zeros(padSize, imageToFilter.type()); 
	}
	
	Mat copyRect= zeroPaddedScratchImage(zeroPaddedScratchROI); 
	imageToFilter.copyTo(copyRect); 
	
	integral(zeroPaddedScratchImage, integralImage, filterType); 
	
}

void OpenCV2BoxFilter::setBoxFilter(Mat &destinationImage, Rect boxPosition, double scaleResultFactor){
	int leftx = boxPosition.x + maxPaddingRequired; 
	int topy = boxPosition.y + maxPaddingRequired; 
	int rightx = leftx+boxPosition.width; 
	int bottomy = topy+boxPosition.height; 
	
	int imageWidth = integralImage.cols - 2*maxPaddingRequired - 1; 
	int imageHeight = integralImage.rows -2*maxPaddingRequired - 1; 
	
	if (destinationImage.cols != imageWidth || destinationImage.rows != imageHeight || destinationImage.type() != filterType)
		destinationImage.create(imageWidth, imageHeight, filterType); 

	Mat m = integralImage(Rect(leftx, topy, imageWidth, imageHeight)); 
	
	m.copyTo(destinationImage); 
	destinationImage += integralImage(Rect(rightx, bottomy, imageWidth, imageHeight));
	destinationImage -= integralImage(Rect(leftx, bottomy, imageWidth, imageHeight)); 
	destinationImage -= integralImage(Rect(rightx, topy, imageWidth, imageHeight)); 
	
	if (scaleResultFactor != 1.0) {
		destinationImage *= scaleResultFactor; 
	}
	
} 

void OpenCV2BoxFilter::accumulateBoxFilter(cv::Mat &destinationImage, Rect boxPosition, double scaleResultFactor) {
	
	int leftx = boxPosition.x + maxPaddingRequired; 
	int topy = boxPosition.y + maxPaddingRequired; 
	int rightx = leftx+boxPosition.width; 
	int bottomy = topy+boxPosition.height; 
	
	size_t imageWidth = integralImage.cols - 2*maxPaddingRequired - 1; 
	size_t imageHeight = integralImage.rows -2*maxPaddingRequired - 1; 
	
	Mat roi = integralImage(Rect(leftx, topy, imageWidth, imageHeight)); 
	scaleAdd(roi, scaleResultFactor, destinationImage, destinationImage); 
	
	roi = integralImage(Rect(rightx, bottomy, imageWidth, imageHeight));
	scaleAdd(roi, scaleResultFactor, destinationImage, destinationImage); 
	
	roi = integralImage(Rect(leftx, bottomy, imageWidth, imageHeight)); 
	scaleAdd(roi, -scaleResultFactor, destinationImage, integralImage); 
	
	roi = integralImage(Rect(rightx, topy, imageWidth, imageHeight));
	scaleAdd(roi, -scaleResultFactor, destinationImage, integralImage); 
} 





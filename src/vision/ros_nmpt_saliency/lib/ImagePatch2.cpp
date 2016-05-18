/*
 *  ImagePatch2.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/24/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#include "ImagePatch2.h"
#include "DebugGlobals.h"
#include "NMPTUtils.h"
#include <iostream>
#include <math.h>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std; 
using namespace cv; 

ImagePatch2::ImagePatch2() {
}

ImagePatch2 & ImagePatch2::operator=(const ImagePatch2 &rhs) {
	if (this != &rhs) {
		copy(rhs); 
	}
	return *this; 
}

ImagePatch2::ImagePatch2(const ImagePatch2 &rhs) {
	if (this != &rhs) {
		copy(rhs); 
	} 
}

void ImagePatch2::copy(const ImagePatch2 &rhs, int clone) {
	if (!clone) {
		this->imgData = rhs.imgData; 
		this->intData = rhs.intData;
		this->sqIntData = rhs.sqIntData; 
		this->tIntData = rhs.tIntData; 
	} else {
		this->imgData = rhs.imgData.clone(); 
		this->intData = rhs.intData.clone();
		this->sqIntData = rhs.sqIntData.clone(); 
		this->tIntData = rhs.tIntData.clone(); 
	}
}

ImagePatch2::~ImagePatch2() {
}

void ImagePatch2::setImage(const Mat &image, int setData, int setIntegral, int setSqInt, int setTInt) {
	setImage(image, Rect(0,0,image.cols, image.rows), setData, setIntegral); 
}

void ImagePatch2::setImage(const Mat &image, Rect ROI, int setData, int setIntegral, int setSqInt, int setTInt) {
	
	int ROIsize = ROI.width * ROI.height; 
	int maxpix = 0x8FFFFFFF >> 3; 
	
	if (image.cols==0 || image.rows == 0 || ROI.width == 0 || ROI.height==0 
		|| (setData==0 && setIntegral==0 && setSqInt == 0 && setTInt == 0)) return; 
	
	if (image.depth() != CV_8U || image.channels() > 1 || ROIsize > maxpix ) {
		cerr << "Error: Image patch source must be 8-bit single channel matrix with < "<< maxpix << " pixels." << endl; 
		return; 
	}
	
	
	if (setData || (setTInt && !(setIntegral || setSqInt))) {
		image(ROI).copyTo(imgData); 
	}
	if (setIntegral && !(setSqInt || setTInt)) { //only setIntegral is checked
		integral(image(ROI), intData, CV_32S); 
	}
	if (setSqInt && !setTInt) { //setTInt is unchecked, but sqInt is
		integral(image(ROI), intData, sqIntData, CV_32S); 
	}
	if (setTInt) {
		integral(image(ROI), intData, sqIntData, tIntData, CV_32S); 
	}
	if (!setIntegral) intData = Mat();  //clear integral if it wasn't wanted
	if (!setSqInt) sqIntData = Mat(); //clear sq integral if it wasn't wanted
}

Size ImagePatch2::getImageSize() const {
	if (hasImageRep() )
		return imgData.size();
	if (hasIntegralRep() )
		return Size(intData.cols-1,intData.rows-1);
	if (hasSqIntegralRep() )
		return Size(sqIntData.cols-1,sqIntData.rows-1) ; 
	if (hasTIntegralRep() )
		return Size(tIntData.cols-1,tIntData.rows-1) ; 
	return Size(0,0); 
}

const cv::Mat ImagePatch2::getImageHeader() const {
	return imgData; 
}

const cv::Mat ImagePatch2::getIntegralHeader() const {
	return intData; 
}

const cv::Mat ImagePatch2::getSqIntegralHeader() const {
	return sqIntData; 
}

const cv::Mat ImagePatch2::getTIntegralHeader() const {
	return tIntData; 
}

int ImagePatch2::hasImageRep() const {
	return (imgData.cols > 0 && imgData.rows > 0 && imgData.data != NULL);
}

int ImagePatch2::hasIntegralRep() const {
	return (intData.cols > 0 && intData.rows > 0 && intData.data != NULL);
}

int ImagePatch2::hasSqIntegralRep() const {
	return (sqIntData.cols > 0 && sqIntData.rows > 0 && sqIntData.data != NULL);
}

int ImagePatch2::hasTIntegralRep() const {
	return (tIntData.cols > 0 && tIntData.rows > 0 && tIntData.data != NULL);
}

void ImagePatch2::getImageRep(cv::Mat &dest) const {
	if (hasImageRep()) {
		dest = imgData; 
		return; 
	}
	if (hasIntegralRep()) {
		NMPTUtils::unIntegrate(intData, dest); 
		return; 
	}
	if (hasSqIntegralRep()) {
		NMPTUtils::unSqIntegrate(sqIntData, dest); 
		return; 
	}
	if (hasTIntegralRep()) {
		cout << "Warning! Can't recreate image data from tilted integral alone." << endl; 
	}
}


void ImagePatch2::createRepIfNeeded(int setData, int setIntegral, int setSqInt, int setTInt) {
	int needed = setData && !hasImageRep() || setIntegral && !hasIntegralRep() 
	|| setSqInt && !hasSqIntegralRep() || setTInt && !hasTIntegralRep(); 
	if (needed) {
		Mat im; 
		getImageRep(im); 
		setImage(im, setData || hasImageRep(), setIntegral  || hasIntegralRep(), 
				 setSqInt || hasSqIntegralRep(), setTInt || hasTIntegralRep()); 
	}
}

FileStorage& operator << (cv::FileStorage &fs, const ImagePatch2 &rhs) {
	Mat im; 
	rhs.getImageRep(im); 
	fs << "{" << "has_im" << rhs.hasImageRep() << "has_int" << rhs.hasIntegralRep()
	<< "has_sqint" << rhs.hasSqIntegralRep() << "has_tint" << rhs.hasTIntegralRep(); 
	NMPTUtils::writeMatBinary(fs, "image",im); 
	fs << "}"; 
	return fs; 
}

void operator >> ( const cv::FileNode &fs, ImagePatch2 &rhs)  {
	int has_im, has_int, has_sqint, has_tint; 
	Mat im; 
	fs["has_im"] >> has_im; 
	fs["has_int"] >> has_int; 
	fs["has_sqint"] >> has_sqint; 
	fs["has_tint"] >> has_tint; 
	NMPTUtils::readMatBinary(fs["image"], im); 
	rhs.setImage(im, has_im, has_int, has_sqint, has_tint);  
}

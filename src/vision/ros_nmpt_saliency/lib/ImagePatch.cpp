/*
 *  ImagePatch.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/11/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "ImagePatch.h"
#include "FeatureRegressor.h"
#include "BoxFeature.h"
#include <iostream>
#include <math.h>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std; 
using namespace cv; 

integral_type _integral_max = ((integral_type)1) << (sizeof(integral_type)*8-9);  

ImagePatch::ImagePatch() {
	width = 0; 
	height = 0; 
}

ImagePatch & ImagePatch::operator=(const ImagePatch &rhs) {
	if (this != &rhs) {
		this->width = rhs.width; 
		this->height = rhs.height; 
		this->imgData = rhs.imgData.clone(); 
		this->intData = rhs.intData.clone(); 
	}
	return *this; 
}

ImagePatch::ImagePatch(const ImagePatch &rhs) {
	if (this != &rhs) {
		this->width = rhs.width; 
		this->height = rhs.height; 
		this->imgData = rhs.imgData.clone(); 
		this->intData = rhs.intData.clone(); 
	} 
}

ImagePatch::~ImagePatch() {
}

const integral_type* ImagePatch::startOfIntegralData() const{
	return (integral_type*)intData.data;
}

const unsigned char* ImagePatch::startOfImageData() const {
	return imgData.data;
}

int ImagePatch::imageDataRowWidth() const {
	return imgData.step/sizeof(char); 
}

int ImagePatch::imageDataRowBytes() const{
	return imgData.step; 
}

int ImagePatch::integralDataRowWidth() const{
	return intData.step/sizeof(int);
}

int ImagePatch::integralDataRowBytes() const {
	//return integralDataRowWidth()*sizeof(integral_type); 
	return intData.step;
}

void ImagePatch::addToStream(ostream& out) {
	int dataRowWidth = imageDataRowWidth(); 
	const unsigned char* data =  imgData.data;
	const integral_type* integralData = (integral_type *)intData.data; 
	
	out << width << " " << height << " " << dataRowWidth << endl;
	out << (int)(data != NULL) << endl; 
	if (data != NULL) {
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				out << (int)data[i*dataRowWidth+j] << " " ; 
			}
			out << endl; 
		}
	}
	out << (int)(integralData != NULL) << endl; 
	if (integralData != NULL) {
		for (int i = 0; i < height+1; i++) {
			for (int j = 0; j < width+1; j++) {
				out << integralData[i*(width+1)+j] << " " ;
			} 
			out << endl; 
		}
	}
} 

void ImagePatch::addToStreamBinary(ostream& out) {
	int dataRowWidth = imageDataRowWidth(); 
	const unsigned char* data = imgData.data;
	const integral_type* integralData =(integral_type*)intData.data; 
	
	char hasData = (char)(data != NULL); 
	char hasIntegralData = (char) (integralData!=NULL); 
	out.write((char*)&width, sizeof(int)); 
	out.write((char*)&height, sizeof(int)); 
	out.write((char*)&dataRowWidth, sizeof(int)); 
	out.write(&hasData,sizeof(char)); 
	if (hasData) {
		out.write((char*)data, height*dataRowWidth*sizeof(char)); 
	}
	out.write(&hasIntegralData,sizeof(char)); 
	if (hasIntegralData) {
		Size s = getIntegralSize(); 
		out.write((char*)integralData,s.height*integralDataRowBytes()); 
	} 
}

void ImagePatch::readFromStreamBinary(istream& in) {
	
	char hasData;// = (char)(data != NULL); 
	char hasIntegralData;// = (char) (integralData!=NULL); 
	int dataRowWidth; 
	
	
	in.read((char*)&width, sizeof(int)); 
	in.read((char*)&height, sizeof(int)); 
	in.read((char*)&dataRowWidth, sizeof(int)); 
	in.read(&hasData,sizeof(char)); 
	if (hasData) {		
		imgData.create(height, width, CV_8U);
		unsigned char* data = imgData.data;
		in.read((char*)data, height*imageDataRowBytes()); 
	}
	in.read(&hasIntegralData,sizeof(char)); 
	if (hasIntegralData) {
		intData.create(height+1, width+1, CV_32S);
		Size s = getIntegralSize();
		integral_type* integralData = (integral_type*) intData.data; 
		in.read((char*)integralData,s.height*integralDataRowBytes()); 
	}
}

void ImagePatch::readFromStream(istream& in) {
	int dataRowWidth;
	in >> width >> height >> dataRowWidth; 
	int hasData; 
	in >> hasData; 
	if (hasData) {
		imgData.create(height, width, CV_8U);
		unsigned char* data = imgData.data;
		dataRowWidth = imageDataRowWidth(); 
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				int pixVal; 
				in >> pixVal; 
				data[i*dataRowWidth+j] = (unsigned char) pixVal; 
			}
		}
	}
	in >> hasData; 
	if (hasData) {
		intData.create(height+1, width+1, CV_32S);
		integral_type* integralData = (integral_type*)intData.data; 
		size_t integralRowWidth = integralDataRowWidth(); 
		for (int i = 0; i < height+1; i++) {
			for (int j = 0; j < width+1; j++) {
				integral_type pixVal; 
				in >> pixVal; 
				integralData[i*integralRowWidth+j] = pixVal; 
			}
		}
	}
}

void ImagePatch::setImage(const Mat &image, int setData, int setIntegral) {
	setImage(image, Rect(0,0,image.cols, image.rows), setData, setIntegral); 
}

void ImagePatch::setImage(const Mat &image, Rect ROI, int setData, int setIntegral) {
	
	integral_type ROIsize = ROI.width * ROI.height; 
	if (image.depth() != CV_8U || image.channels() > 1 || ROIsize > _integral_max ) {
		cerr << "Error: Image patch source must be 8-bit single channel image with < 2^"<<(sizeof(integral_type)*8-1) <<"/256 pixels." << endl; 
		return; 
	}
	
	if (image.cols==0 || image.rows == 0 || ROI.width == 0 || ROI.height==0 
		|| (setData==0 && setIntegral==0)) return; 
	
	width = ROI.width; 
	height = ROI.height; 
	
	if (setData) {
		image(ROI).copyTo(imgData); 
	}
	if (setIntegral) {
		integral(image(ROI), intData); 
	}
}

void ImagePatch::setImage(const IplImage* image, CvRect ROI, int setData, int setIntegral) {
	setImage((const Mat &)image, (Rect)ROI, setData, setIntegral); 
}

void ImagePatch::setImage(const IplImage* image, int setData, int setIntegral) {
	CvRect roi = cvRect(0, 0, image->width, image->height); 
	this->setImage(image, roi, setData, setIntegral); 
	
}

Size ImagePatch::getImageSize() const {
	return Size(width,height);
}

Size ImagePatch::getIntegralSize() const {
	return intData.size(); 
}

IplImage* ImagePatch::getImageHeader() const {
	if (imgData.data == NULL) return NULL; 
	IplImage* retval = cvCreateImageHeader( getImageSize(), IPL_DEPTH_8U, 1 ); 
	cvSetData(retval, imgData.data, imgData.step); 
	return retval; 
}

void ImagePatch::getImageHeader(cv::Mat &header) const {
	header = imgData; 
}

void ImagePatch::writePatchesToFile(const char* filename, const vector<ImagePatch*> &patches) {
	
	ofstream out;
	out.open (filename, ios::binary);
	int size = patches.size(); 
	out.write((char*)&size,sizeof(int)); 
	for (unsigned int i = 0; i < patches.size(); i++) {
		patches[i]->addToStreamBinary(out); 
	}
	out.close(); 
}

vector<ImagePatch*> ImagePatch::readPatchesFromFile(const char* filename) {
	ifstream in;
	in.open (filename, ios::binary);
	int size = 0; 
	in.read((char*)&size, sizeof(int)); 
	vector<ImagePatch*> patches(size,(ImagePatch*)NULL);
	for (int i = 0; i < size; i++) {
		patches[i] = new ImagePatch(); 
		patches[i]->readFromStreamBinary(in); ; 
	}
	in.close(); 
	return patches; 
}

ostream& operator<< (ostream& ofs, ImagePatch* a) {
	a->addToStream(ofs);
	return ofs; 
}

istream& operator>> (istream& ifs, ImagePatch* a) {
	a->readFromStream(ifs);
	return ifs; 
}

ostream& operator<< (ostream& ofs, vector<ImagePatch*>& a) {
	ofs << a.size() << endl; 
	for (unsigned int i = 0; i < a.size(); i++) {
		ofs << a[i]; 
	}
	return ofs; 
}

istream& operator>> (istream& ifs, vector<ImagePatch*>& a) {
	int size = 0; 
	ifs >> size; 
	a.resize(size, (ImagePatch*)NULL); 
	for (int i = 0; i < size; i++) {
		a[i] = new ImagePatch(); 
		ifs >> a[i]; 
	}
	return ifs; 
}

void ImagePatch::testImagePatch() {
	if (getImageSize().width <= 0) {
		/*
		 IplImage* patch = cvCreateImage(cvSize(30, 30), IPL_DEPTH_8U, 1); 
		 cvSet(patch, cvRealScalar(1)); 
		 setImage(patch, 1, 1); 
		 cvReleaseImage(&patch); 
		 */
		Mat patch(30,30,CV_8U,1); 
		setImage(patch); 
		//return; 
	}
	const unsigned char* imdata = startOfImageData(); 
	const integral_type* intdata = startOfIntegralData(); 
	CvSize imSize = getImageSize(); 
	CvSize intSize = getIntegralSize(); 
	size_t imRowBytes = imageDataRowBytes(); 
	size_t intRowBytes = integralDataRowBytes(); 
	size_t imRowWidth = imageDataRowWidth(); 
	size_t intRowWidth = integralDataRowWidth(); 
	cout << "Created an image patch with size " << imSize.width << "x" << imSize.height << "; each row has " << imRowWidth << " elements and " << imRowBytes << " bytes." << endl; 
	cout << "Created an integral patch with size " << intSize.width << "x"<< intSize.height << "; each row has " << intRowWidth << " elements and " << intRowBytes << " bytes." << endl; 
	cout << "Image data: " << endl; 
	for (size_t i = 0; i < (size_t)imSize.height; i++) {
		cout << "| "; 
		for (size_t j = 0; j < imRowWidth; j++) {
			cout << (int) imdata[i*imRowWidth+j] << " | " ; 
		}
		cout << endl; 
	}	
	cout << "Integral data: " << endl; 
	for (size_t i = 0; i < (size_t)intSize.height; i++) {
		cout << "| "; 
		for (size_t j = 0; j < (size_t)intRowWidth; j++) {
			cout << intdata[i*intRowWidth+j] << " | " ; 
		}
		cout << endl; 
	}
}


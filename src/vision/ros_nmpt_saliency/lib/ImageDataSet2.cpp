/*
 *  ImageDataset2.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/25/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#include "ImageDataSet2.h"
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <sstream>

using namespace std; 
using namespace cv; 

ImageDataSet2::ImageDataSet2() {
}

ImageDataSet2::~ImageDataSet2(){
} 	

ImageDataSet2 & ImageDataSet2::operator=(const ImageDataSet2 &rhs) {
	if (this != &rhs) {
		filenames = rhs.filenames; 
		labels = rhs.labels; 
	}
	return *this; 
}

ImageDataSet2::ImageDataSet2(const ImageDataSet2 &rhs) {
	if (this != &rhs) {
		filenames = rhs.filenames; 
		labels = rhs.labels; 
	} 
}

void ImageDataSet2::addEntry(const string &filename, const vector<double> &newlabels) {
	if (filename.empty()) return; 
	filenames.push_back(filename); 
	if (!newlabels.empty())
		labels.push_back(newlabels); 
} 

ImageDataSet2::ImageDataSet2(const string &imageListFile, 
							 const string &imageLabelsFile){
	if (!imageLabelsFile.empty()) {
		ifstream tmp(imageLabelsFile.c_str());  
		string line; 
		getline(tmp, line); 
		tmp.close(); 
		
		stringstream ss(line); 
		int num = 0; 
		while (!ss.eof()) {
			double d; 
			ss >> d; 
			num++; 
		}
		
		ifstream inl(imageLabelsFile.c_str()); 
		ifstream in(imageListFile.c_str()); 
		while (!in.eof() && !inl.eof())
		{
			string fname; 
			in >> fname; 
			if (fname.empty()) break;
			vector<double> line_labels(num); 
			for (int i = 0; i < num; i++) {
				inl >> line_labels[i];  
			}
			addEntry(fname, line_labels); 
		} 	
		in.close(); 
		inl.close(); 
	} else {
		ifstream in(imageListFile.c_str()); 
		while (!in.eof())
		{
			string fname; 
			in >> fname; 		
			if (fname.empty()) break;	
			addEntry(fname); 
		} 	
		in.close(); 
	}
}

ImageDataSet2 ImageDataSet2::split(int splitStart, int splitEnd) {
	int numToRemove = splitEnd - splitStart + 1; 
	if (numToRemove < 1 || numToRemove > getNumEntries() || splitStart < 0 || splitEnd >= getNumEntries())
		return ImageDataSet2(); 
	vector<int> list(numToRemove); 
	for (int i = 0; i < numToRemove; i++) list[i] = i+splitStart; 
	return split(list, numToRemove); 
}

ImageDataSet2 ImageDataSet2::split(const vector<int> &removeToNewList, int numToRemove){
	ImageDataSet2 splitSet; 
	for (int i = 0; i < numToRemove; i++) {
		if (removeToNewList[i] < 0 || removeToNewList[i] >= getNumEntries()) return ImageDataSet2(); 
		if (numLabelsPerImage() > 0) {
			vector<double> list(numLabelsPerImage()); 
			for (int j = 0; j < numLabelsPerImage(); j++) list[j] = labels[removeToNewList[i]][j]; 
			splitSet.addEntry(filenames[removeToNewList[i]], list); 
		} else 
			splitSet.addEntry(filenames[removeToNewList[i]]); 
	}
	
	int last = 0; 
	int removed = 0; 
	for (int i = 0; i < getNumEntries(); i++) {
		if (removed < numToRemove && i == removeToNewList[removed]) {
			removed++; 
		} else {
			filenames[last] = filenames[i]; 
			labels[last] = labels[i]; 
			last++; 
		}
	}
	filenames.resize(last); 
	labels.resize(last); 
	return splitSet; 
} 	

void ImageDataSet2::merge(const ImageDataSet2 &setToAdd){
	if (setToAdd.numLabelsPerImage()!=numLabelsPerImage()) {
		cerr << "Attempting to merge incompatible image sets! Merge failed." << endl; 
		return; 
	}
	for (int i = 0; i < setToAdd.getNumEntries(); i++) {
		addEntry(setToAdd.getFileName(i), setToAdd.getFileLabels(i));
	}
} 	

std::string ImageDataSet2::getFileName(int fileNumber) const{
	return filenames[fileNumber]; 
} 	

vector<double> ImageDataSet2::getFileLabels(int fileNumber) const{
	vector<double> retval; 
	return labels.empty()? retval: labels[fileNumber]; 
} 	

int ImageDataSet2::numLabelsPerImage() const {
	return labels.empty()?0 : labels[0].size(); 
} 	

int ImageDataSet2::getNumEntries() const{
	return filenames.size(); 
} 	

int ImageDataSet2::empty() const {
	return filenames.empty(); 
}

FileStorage& operator << (cv::FileStorage &fs, const ImageDataSet2 &rhs) {
	fs << "{" << "empty" << rhs.empty(); 
	if (!rhs.empty()) {
		fs << "entries" << "[";  
		for (size_t i = 0; i < rhs.filenames.size(); i++) {
			//vector<double> labels = rhs.numLabelsPerImage() > 0? rhs.labels[i] :vector<double>(0); 
			fs << "{" << "name" <<  rhs.filenames[i] ;
			if (rhs.numLabelsPerImage() > 0){
				fs << "labels" << "[" << rhs.labels[i] << "]";
			}
			fs << "}"; 
		}
		fs << "]" ; 
	}
	fs << "}" ; 
	return fs; 
}

void operator >> ( const cv::FileNode &fs, ImageDataSet2 &rhs)  {
	int empty; 
	fs["empty"] >> empty; 
	if (empty) return; 
	FileNode node = fs["entries"]; 
	for (size_t i = 0; i < node.size(); i++) {
		string fname;
		vector<double> labels; 
		node[i]["name"] >> fname; 
		if (node[i].size() > 1) node[i]["labels"] >> labels; 
		rhs.addEntry(fname,labels); 
	}
}

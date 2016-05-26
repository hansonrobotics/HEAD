/*
 *  ImageDataSet.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/12/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "ImageDataSet.h"
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <iostream>

using namespace std; 

ImageDataSet::ImageDataSet(int numSlots, int labelsPerImage){
	this->numEntries = 0; 
	this->numSlots = 0; 
	this->numLabels = labelsPerImage; 
	filenames = (char**)NULL; 
	labels = (double**)NULL; 
	increaseNumSlots(numSlots); 
} 	

ImageDataSet::~ImageDataSet(){
	if (filenames != NULL) {
		free(filenames[0]); 
		free(filenames); 
	}
	if (labels != NULL) {
		free(labels[0]); 
		free(labels); 
	}
} 	

void ImageDataSet::addEntry(const string &filename, const vector<double> &labels) {
	if (numLabels == 0 && !labels.empty()) {
		cerr << "Attempting to set labels for a label-less ImageDataSet." << endl; 
		return; 
	}
	while (numEntries >= numSlots) increaseNumSlots((numSlots+1)*2); // won't infinite loop if numEntries is 0.
	strncpy(this->filenames[numEntries], filename.c_str(), maxStrLen); 
	if (numLabels > 0) {
		memcpy(this->labels[numEntries], &(labels[0]), numLabels*sizeof(double)); 
	}
	numEntries++; 	
}

ImageDataSet* ImageDataSet::loadFromFile(const string &imageListFile, const string &imageLabelsFile){
	if (!imageLabelsFile.empty()) {
		ifstream tmp(imageLabelsFile.c_str());  
		char line[5000]; 
		const char* sep = " \n"; 
		char *word, *brkt; 
		tmp.getline(line, 5000);  
		tmp.close(); 
		int num = 0; 
		for (word = strtok_r(line, sep, &brkt); word; word=strtok_r(NULL, sep, &brkt)) {
			num++; 
		}
		
		ImageDataSet* retval = new ImageDataSet(0, num); 
		vector<double> labels(num);
		char fname[5000]; 
		
		
		ifstream inl(imageLabelsFile.c_str()); 
		ifstream in(imageListFile.c_str()); 
		int it = 0; 
		while (!in.eof() && !inl.eof())
		{
			in.getline(fname, 5000);
			if (fname[0] == 0) break;
			
			for (int i = 0; i < num; i++) {
				inl >> labels[i];  
			}
			retval->addEntry(fname, labels); 
			it++; 	
		} 	
		it--; //Reads in one-too-many filenames.
		//numEntries = it; 
		//free(labels); 
		in.close(); 
		inl.close(); 
		return retval; 
	} else {
		
		ImageDataSet* retval = new ImageDataSet(); 
		ifstream in(imageListFile.c_str()); 
		char fname[5000]; 
		int it = 0; 
		while (!in.eof())
		{
			in.getline(fname, 5000);
			if (fname[0] == 0) break;			
			retval->addEntry(fname); 
			it++; 	
		} 	
		it--; //Reads in one-too-many filenames.
		//numEntries = it; 
		in.close(); 
		return retval; 
	}
}


ImageDataSet* ImageDataSet::split(int splitStart, int splitEnd) {
	int numToRemove = splitEnd - splitStart + 1; 
	if (numToRemove < 1 || numToRemove > numEntries || splitStart < 0 || splitEnd >= numEntries)
		return 0; 
	vector<int> list(numToRemove); 
	for (int i = 0; i < numToRemove; i++) list[i] = i+splitStart; 
	ImageDataSet* retval = split(list, numToRemove); 
	//free(list); 
	return retval; 
}

ImageDataSet* ImageDataSet::split(const vector<int> &removeToNewList, int numToRemove){
	ImageDataSet* splitSet = new ImageDataSet(numToRemove, numLabels); 
	for (int i = 0; i < numToRemove; i++) {
		if (removeToNewList[i] < 0 || removeToNewList[i] >= numEntries) return 0; 
		if (numLabels > 0) {
			vector<double> list(numLabels); 
			for (int j = 0; j < numLabels; j++) list[j] = labels[removeToNewList[i]][j]; 
			splitSet->addEntry(filenames[removeToNewList[i]], list); 
		} else 
			splitSet->addEntry(filenames[removeToNewList[i]]); 
	}
	
	int last = 0; 
	int removed = 0; 
	for (int i = 0; i < numEntries; i++) {
		if (removed < numToRemove && i == removeToNewList[removed]) {
			removed++; 
		} else {
			memcpy(filenames[last], filenames[i], maxStrLen*sizeof(char)); 
			if (numLabels > 0) {
				memcpy(labels[last], labels[i], numLabels*sizeof(double)); 
			}
			last++; 
		}
	}
	numEntries = last;
	return splitSet; 
} 	

void ImageDataSet::merge(ImageDataSet* setToAdd){
	if (setToAdd->numLabels!=numLabels) {
		cerr << "Attempting to merge incompatible image sets! Merge failed." << endl; 
		return; 
	}
	int newNumEntries = numEntries + setToAdd->numEntries; 
	if (newNumEntries < numSlots) increaseNumSlots(newNumEntries);
	memcpy(filenames[numEntries], setToAdd->filenames[0], setToAdd->numEntries*maxStrLen*sizeof(char));
	if (numLabels > 0)
		memcpy(labels[numEntries], setToAdd->labels[0], setToAdd->numEntries*numLabels*sizeof(double)); 
	numEntries = newNumEntries; 
} 	

std::string ImageDataSet::getFileName(int fileNumber){
	return filenames[fileNumber]; 
} 	

vector<double> ImageDataSet::getFileLabels(int fileNumber){
	vector<double> retval(0); 
	if (numLabels == 0) return retval; 
	for (int i = 0; i < numLabels; i++) retval.push_back(labels[fileNumber][i]); 
	return retval; 
} 	

int ImageDataSet::numLabelsPerImage(){
	return numLabels; 
} 	

int ImageDataSet::getNumEntries(){
	return numEntries; 
} 	

void ImageDataSet::increaseNumSlots(int newNumSlots){
	if (newNumSlots <= numSlots) return; 
	int oldNumSlots = numSlots; 
	numSlots = newNumSlots; 
	char** newFileNames = (char**)malloc(newNumSlots*sizeof(char*)); 
	char* fnameblock = (char*)malloc(newNumSlots*maxStrLen*sizeof(char)); 
	for (int i = 0; i < newNumSlots; i++ ) {
		newFileNames[i] = &(fnameblock[i*maxStrLen]); 
	}
	if (filenames != NULL && oldNumSlots>0) {
		memcpy(fnameblock, filenames[0], oldNumSlots*maxStrLen*sizeof(char)); 
		free(filenames[0]); 
		free(filenames); 
	}
	filenames = newFileNames; 
	
	if (numLabels > 0) {
		double** newLabels = (double**)malloc(newNumSlots*sizeof(double*)); 
		double* labelsblock = (double*)malloc(newNumSlots*numLabels*sizeof(double)); 
		for (int i = 0; i < newNumSlots; i++ ) {
			newFileNames[i] = &(fnameblock[i*maxStrLen]); 
			newLabels[i] = &(labelsblock[i*numLabels]); 
		}
		if (labels != NULL && oldNumSlots>0) {
			memcpy(labelsblock, labels[0], oldNumSlots*numLabels*sizeof(double)); 
			free(labels[0]); 
			free(labels); 
		}
		labels = newLabels; 
	}
} 	

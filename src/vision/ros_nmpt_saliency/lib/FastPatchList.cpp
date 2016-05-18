/*
 *  FastPatchList.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/25/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "FastPatchList.h"
#include "DebugGlobals.h"
#include <math.h>
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace std; 
using namespace cv; 

FastPatchList::FastPatchList(Size minSize, Size maxSize, double scaleInc, double stepWidth,
							 cv::Size patchSize, int scaleStepWidth) : PatchList(minSize, maxSize, scaleInc, stepWidth, patchSize), scalestepwidth(scaleStepWidth){
}

void FastPatchList::resetPointers() {
	if (_PATCHLIST_DEBUG)  cout<< "FastPatchList Reset Pointers" << endl; 
	resetContainerSizes(); 
	
	double* dest1 = (double*)filterImage.data; 
	double* dest2 = (double*)accumulatorImage.data; 
	int dstWidthStep1 = filterImage.step/sizeof(double); 
	int dstWidthStep2 = accumulatorImage.step/sizeof(double); 
		
	const integral_type* integralData = images[0]->startOfIntegralData(); 
	Size mins = getMinEffectivePatchSize(); 
	double currWidth = mins.width; 
	double currHeight = mins.height; 
	double currInc = stepwidth; 
	totalPatches = 0; 
		
	for (int i = 0; i < numScales; i++) {
		Size s = images[0]->getImageSize(); 
		int filterWidth = s.width - (int)currWidth + 1; 
		int filterHeight = s.height - (int)currHeight + 1; 
		int scalewidth = ceil(filterWidth *1.0/currInc); 
		int scaleheight = ceil(filterHeight*1.0/currInc); 
		
		numAtScale[i] = scalewidth*scaleheight; 
		scaleImageSizes[i] = cvSize(scalewidth,scaleheight); 
		scalePatchSizes[i]= cvSize((int)currWidth, (int)currHeight); 
		scaleFilterSizes[i]= cvSize((int)currWidth, (int)currHeight); 
		totalPatches+= numAtScale[i]; 
		if (_PATCHLIST_DEBUG) cout<< "FastPatchList: Scale " << i << " expects " << numAtScale[i] << " patches of size " << (int)currWidth << "x" << (int)currHeight << "." << endl; 
		
		srcAtScales[i] = (const integral_type**)malloc(numAtScale[i]*sizeof(integral_type*)); 
		destsAtScales[i] = (double**)malloc(numAtScale[i]*sizeof(double*));   
		accAtScales[i] = (double**)malloc(numAtScale[i]*sizeof(double*));  
		imLocAtScales[i] = (int*)malloc(numAtScale[i]*sizeof(int));  
		
		int ind = 0; 
		for (int j = 0; j < scaleheight; j++) {
			int width = images[0]->getImageSize().width; 
			int rowInd1 = ((int)(j*currInc))*(images[0]->integralDataRowWidth()); 
			int rowInd2 = j*dstWidthStep1; 
			int rowInd3 = j*dstWidthStep2; 
			int rowInd4 = ((int)(j*currInc))*(width); 
			for (int k = 0; k < scalewidth; k++) {
				srcAtScales[i][ind] = &integralData[rowInd1+(int)(k*currInc)];
				destsAtScales[i][ind] = &dest1[rowInd2+k]; 
				accAtScales[i][ind] = &dest2[rowInd3+k];
				imLocAtScales[i][ind] = rowInd4+(int)(k*currInc); 
				ind++; 
			}
		}
		
		//cout<< "Scale " << i << " found " << ind << " patches." << endl; 
		
		currWidth = currWidth*scaleinc; 
		currHeight = currHeight*scaleinc; 
		if (scalestepwidth)	currInc = currInc*scaleinc;
	}
	
	if (_PATCHLIST_DEBUG) cout<< "In all, " << totalPatches << " patches were listed." << endl; 	
	
	srcInds = (integral_type**)malloc(numAtScale[0]*sizeof(integral_type*)); 
	destInds = (double**)malloc(numAtScale[0]*sizeof(double*)); 
	accInds = (double**)malloc(numAtScale[0]*sizeof(double*)); 
	imLoc = (int*)malloc(numAtScale[0]*sizeof(int)); 
	
	shouldResetAccumulator = 1; 
}

int FastPatchList::setImageAllScalesNeedsPointerReset(IplImage* newImage) {
	
	Mat img = newImage; 
	
	oldNumScales = numScales; 
	origImageSize = img.size(); 
	
	numScales = getEffectiveNumScales(); 
	if (_PATCHLIST_DEBUG) cout<< "SetImage on FastPatchList with " << numScales << " scales." << endl; 
	
	
	if (numScales < 1) {
		cout<< "Warning: There are no scales. Possibly the minimum size is larger than the image?" << endl; 
	}
	
	int needsReset = 0; 
		
	for (int i = 0; i < numScales; i++) {
		if (_PATCHLIST_DEBUG) cout << "For scale " << i << " needsReset is so far " << needsReset << endl; 
		if (i == 0 ) {
			if (!images.empty()) { 
				 if (_PATCHLIST_DEBUG) cout << "Replacing old patch." << endl; 
				
				//checking for a change in image size or underlying memory structure
				const integral_type* previousIntegral = images[i]->startOfIntegralData(); 
				Size s1 = images[i]->getImageSize(); 
				images[i]->setImage(img,copyImageData,1); 
				Size s2 = images[i]->getImageSize(); 
				if (s1.width != s2.width || s1.height != s2.height ||
					previousIntegral != images[i]->startOfIntegralData() ){
					if (_PATCHLIST_DEBUG) {
						if (s1.width != s2.width) 
							cout << "Needs reset because image width at scale " << i 
							<< " changed from " << s1.width << " to " 
							<< s2.width << endl; 
						if (s1.height != s2.height) 
							cout << "Needs reset because image height at scale " << i 
							<< " changed from " << s1.height << " to " 
							<< s2.height << endl; 
						if (previousIntegral != images[i]->startOfIntegralData()) 
							cout << "Needs reset because integral pointer at scale " << i 
							<< " changed from " << (void*)previousIntegral << " to " 
							<< (void*)images[i]->startOfIntegralData()<< endl; 
					}
					needsReset = 1; 
				}
			} else {
				images.push_back(new ImagePatch()); 
				images[0]->setImage(img,copyImageData,1);  
				needsReset = 1; 
			}
		} else if ((unsigned int) i < images.size()) {	
			images[i] = images[0]; 
		} else {
			//cout << "Adding new one." << endl; 
			images.push_back(images[0]); 
			needsReset = 1; 
		}
		
	}
	return needsReset; 	
}

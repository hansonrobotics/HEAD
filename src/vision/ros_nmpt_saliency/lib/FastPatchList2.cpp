/*
 *  FastPatchList2.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/24/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#include "FastPatchList2.h"
#include "DebugGlobals.h"
#include <math.h>
#include <iostream>

using namespace std; 
using namespace cv; 

FastPatchList2::FastPatchList2()  {
	scalestepwidth = 1; 
	init(); 
}

FastPatchList2::FastPatchList2(Size baseObjectSize, Size minSize, Size maxSize, 
							   double scaleInc, double stepWidth, int dontCopyImage,
							   int scaleStepWidth) {
	scalestepwidth = scaleStepWidth; 
	init(baseObjectSize, minSize, maxSize, scaleInc, stepWidth, dontCopyImage); 
}

FastPatchList2 & FastPatchList2::operator=(const FastPatchList2 &rhs) {
	if (this != &rhs) {
		scalestepwidth=rhs.scalestepwidth; 
		copy(rhs); 
	}
	return *this; 
}

FastPatchList2::FastPatchList2(const FastPatchList2 &rhs) {
	if (this != &rhs) {
		scalestepwidth = rhs.scalestepwidth; 
		copy(rhs); 
	} 
}


FastPatchList2::~FastPatchList2() {
	if (_PATCHLIST_DEBUG) cout<< "Fast Patch List Destuctor" << endl; 
	if (_PATCHLIST_DEBUG) cout<< "Calling clearPointers()" << endl; 
	clearPointers(); 
	
}

void FastPatchList2::resetPointers() {
	if (_PATCHLIST_DEBUG)  cout<< "FastPatchList Reset Pointers" << endl; 
	resetContainerSizes(); 
	
	double* dest1 = (double*)filterImage.data; 
	double* dest2 = (double*)accumulatorImage.data; 
	const int* integralData = (int*)images[0].getIntegralHeader().data; 
	
	int dstWidthStep1 = filterImage.step/sizeof(double); 
	int dstWidthStep2 = accumulatorImage.step/sizeof(double); 
	
	Size mins = getMinEffectivePatchSize(); 
	double currWidth = mins.width; 
	double currHeight = mins.height; 
	double currInc = stepwidth; 
	totalPatches = 0; 
	
	for (int i = 0; i < numScales; i++) {
		Size s = images[0].getImageSize(); 
		int filterWidth = s.width - (int)currWidth + 1; 
		int filterHeight = s.height - (int)currHeight + 1; 
		int scalewidth = ceil(filterWidth *1.0/currInc); 
		int scaleheight = ceil(filterHeight*1.0/currInc); 
		
		numAtScale[i] = scalewidth*scaleheight; 
		scaleImageSizes[i] = Size(scalewidth,scaleheight); 
		scalePatchSizes[i]= Size((int)currWidth, (int)currHeight); 
		scaleFilterSizes[i]= Size((int)currWidth, (int)currHeight); 
		totalPatches+= numAtScale[i]; 
		if (_PATCHLIST_DEBUG) cout<< "FastPatchList: Scale " << i << " expects " << numAtScale[i] << " patches of size " << (int)currWidth << "x" << (int)currHeight << "." << endl; 
		
		srcAtScales[i].resize(numAtScale[i]); // = (const integral_type**)malloc(numAtScale[i]*sizeof(integral_type*)); 
		destsAtScales[i].resize(numAtScale[i]); // = (double**)malloc(numAtScale[i]*sizeof(double*));   
		accAtScales[i].resize(numAtScale[i]); // = (double**)malloc(numAtScale[i]*sizeof(double*));  
		imLocAtScales[i].resize(numAtScale[i]); // = (int*)malloc(numAtScale[i]*sizeof(int));  
		
		int ind = 0; 
		for (int j = 0; j < scaleheight; j++) {
			int width = images[0].getImageSize().width; 
			int rowInd1 = ((int)(j*currInc))*(images[0].getIntegralHeader().step/sizeof(int)); 
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
		
		if (_PATCHLIST_DEBUG) cout<< "Scale " << i << " found " << ind << " patches." << endl; 
		
		currWidth = currWidth*scaleinc; 
		currHeight = currHeight*scaleinc; 
		if (scalestepwidth)	currInc = currInc*scaleinc;
	}
	
	if (_PATCHLIST_DEBUG) cout<< "In all, " << totalPatches << " patches were listed." << endl; 	
	/*
	srcInds = (integral_type**)malloc(numAtScale[0]*sizeof(integral_type*)); 
	destInds = (double**)malloc(numAtScale[0]*sizeof(double*)); 
	accInds = (double**)malloc(numAtScale[0]*sizeof(double*)); 
	imLoc = (int*)malloc(numAtScale[0]*sizeof(int)); 
	*/
	shouldResetAccumulator = 1; 
}

int FastPatchList2::setImageAllScalesNeedsPointerReset(const Mat &img) {

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
				const int* previousIntegral = (int*)images[i].getIntegralHeader().data ; 
				Size s1 = images[i].getImageSize(); 
				images[i].setImage(img,copyImageData,1); 
				Size s2 = images[i].getImageSize(); 
				if (s1.width != s2.width || s1.height != s2.height ||
					previousIntegral != (int*)images[i].getIntegralHeader().data ){
					needsReset = 1; 
					if (_PATCHLIST_DEBUG) {
						if (s1.width != s2.width) 
							cout << "Needs reset because image width at scale " << i 
							<< " changed from " << s1.width << " to " 
							<< s2.width << endl; 
						if (s1.height != s2.height) 
							cout << "Needs reset because image height at scale " << i 
							<< " changed from " << s1.height << " to " 
							<< s2.height << endl; 
						if (previousIntegral != (int*)images[i].getIntegralHeader().data) 
							cout << "Needs reset because integral pointer at scale " << i 
							<< " changed from " << (void*)previousIntegral << " to " 
							<< (void*)images[i].getIntegralHeader().data<< endl; 
					}
				}
			} else {
				ImagePatch2 patch; 
				images.push_back(patch); 
				images[0].setImage(img,copyImageData,1);  
				needsReset = 1; 
			}
		} else if ( (size_t) i < images.size()) {	
			images[i] = images[0]; 
			if (images[i].getIntegralHeader().data != images[0].getIntegralHeader().data) {
				cout << "Warning: FastPatchList images don't have identical data." << endl; 
			}
		} else {
			//cout << "Adding new one." << endl; 
			images.push_back(images[0]); 
			needsReset = 1; 
			images[i] = images[0]; 
			if (images[i].getIntegralHeader().data != images[0].getIntegralHeader().data) {
				cout << "Warning: FastPatchList images don't have identical data." << endl; 
			}
		}
		
	}
	return needsReset; 	
}

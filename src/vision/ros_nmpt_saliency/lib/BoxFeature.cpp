/*
 *  BoxFeature.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "BoxFeature.h"
#include "DebugGlobals.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
//#include <omp.h>

#define max(x,y) (x>y?x:y)
#define min(x,y) (x<y?x:y)
#define sign(x) (x<0?-1:1)

using namespace std; 
using namespace cv; 

BoxFeature::BoxFeature(cv::Size expectedPatchSize, int nBoxes) : Feature(expectedPatchSize){
	snprintf(featureName, 128, "BoxFeature"); 
	
	patchSize = expectedPatchSize; 
	
	weights = NULL; 
	boxX = NULL; 
	boxY = NULL; 
	width = NULL; 
	height = NULL; 
	parameters = NULL; 
	minParamVals = NULL; 
	maxParamVals = NULL; 
	kernel = NULL; 
	boxX1 = NULL; 
	boxX2 = NULL; 
	boxY1 = NULL; 
	boxY2 = NULL; 
	
	createWithNBoxes(nBoxes); 
		
	prefersIntegral = 1; 
	parameterType = real; 
	
	if (_BOXFEATURE_DEBUG) cout<< "Randomizing Initial Parameters." << endl; 
	setRandomParameterValues(); 	
	
	if (_BOXFEATURE_DEBUG) cout<< "Constructor Finished." << endl; 
}

void BoxFeature::createWithNBoxes(int n) {
	
	cvReleaseMat(&weights); 
	cvReleaseMat(&boxX); 
	cvReleaseMat(&boxY); 
	cvReleaseMat(&width); 
	cvReleaseMat(&height); 
	cvReleaseMat(&parameters); 
	cvReleaseMat(&minParamVals); 
	cvReleaseMat(&maxParamVals); 
	cvReleaseMat(&boxX1); 
	cvReleaseMat(&boxY1); 
	cvReleaseMat(&boxX2); 
	cvReleaseMat(&boxY2); 
	cvReleaseImage(&kernel); 
	
	numBoxes = n; 
	
	weights = cvCreateMat(1, numBoxes, CV_64FC1) ;
	boxX = cvCreateMat(1, numBoxes, CV_64FC1); 
	boxY = cvCreateMat(1, numBoxes, CV_64FC1); 
	width = cvCreateMat(1, numBoxes, CV_64FC1); 
	height = cvCreateMat(1, numBoxes, CV_64FC1); 
	
	parameters = cvCreateMat(1, 5*numBoxes+4, CV_64FC1); 
	
	minParamVals = cvCreateMat(1, 5*numBoxes+4, CV_64FC1); 
	for (int i = 0; i < numBoxes; i++) {
		cvSetReal2D(minParamVals, 0, i*5, -5); 
		cvSetReal2D(minParamVals, 0, i*5+1, 1); 
		cvSetReal2D(minParamVals, 0, i*5+2, 1); 
		cvSetReal2D(minParamVals, 0, i*5+3, 1); 
		cvSetReal2D(minParamVals, 0, i*5+4, 1); 
	}
	cvSetReal2D(minParamVals, 0, 5*numBoxes, -1); 
	cvSetReal2D(minParamVals, 0, 5*numBoxes+1, -1); 
	cvSetReal2D(minParamVals, 0, 5*numBoxes+2, 0); 
	cvSetReal2D(minParamVals, 0, 5*numBoxes+3, 0); 
	
	maxParamVals = cvCreateMat(1, 5*numBoxes+4, CV_64FC1); 
	for (int i = 0; i < numBoxes; i++) {
		cvSetReal2D(maxParamVals, 0, i*5, 5); 
		cvSetReal2D(maxParamVals, 0, i*5+1, patchSize.width); 
		cvSetReal2D(maxParamVals, 0, i*5+2, patchSize.height); 
		cvSetReal2D(maxParamVals, 0, i*5+3, patchSize.width); 
		cvSetReal2D(maxParamVals, 0, i*5+4, patchSize.height); 
	}
	cvSetReal2D(maxParamVals, 0, 5*numBoxes, 1); 
	cvSetReal2D(maxParamVals, 0, 5*numBoxes+1, 1); 
	cvSetReal2D(maxParamVals, 0, 5*numBoxes+2, 1); 
	cvSetReal2D(maxParamVals, 0, 5*numBoxes+3, 1); 
	
	
	kernel = cvCreateImage(patchSize, IPL_DEPTH_64F, 1); 
	
	boxX1 = cvCreateMat(1, numBoxes, CV_64FC1);  
	boxX2 = cvCreateMat(1, numBoxes, CV_64FC1);  
	boxY1 = cvCreateMat(1, numBoxes, CV_64FC1);  
	boxY2 = cvCreateMat(1, numBoxes, CV_64FC1);  
	
}

BoxFeature::~BoxFeature() {
	cvReleaseMat(&weights ); 
	cvReleaseMat(&boxX ); 
	cvReleaseMat(&boxX1 ); 
	cvReleaseMat(&boxX2 ); 
	cvReleaseMat(&boxY ); 
	cvReleaseMat(&boxY1 ); 
	cvReleaseMat(&boxY2 ); 
	cvReleaseMat(&width ); 
	cvReleaseMat(&height ); 
}

void BoxFeature::setFeatureParameters(const cv::Mat &newParamVec) {	
	const CvMat dummy = newParamVec; 
	const CvMat* paramVec = &dummy; 
	
	if (_BOXFEATURE_DEBUG) cout<< "Starting to Set Feature parameters." << endl; 
	assert((paramVec->width-numExtraFeatures) %numFeaturesPerBox == 0 && 
		   paramVec->height == 1); 
	if (_BOXFEATURE_DEBUG) cout<< "Param Vec is the right size." << endl ;
	
	if (!(paramVec == parameters)) {
		int n = (paramVec->width - numExtraFeatures)/numFeaturesPerBox; 
		if (n == numBoxes)
			createWithNBoxes(n); 			
		cvCopy(paramVec, parameters);
	}
	
	if (_BOXFEATURE_DEBUG) cout<< " Making sure parameters are in proper range." << endl; 
	checkParameterBounds(parameters); 
	int ind = 0; 
	
	for (int i = 0; i < numBoxes; i++) {
		if (_BOXFEATURE_DEBUG) cout<< "Setting data for box " << i << endl; 
		cvSetReal2D(weights, 0, i, sign(cvGetReal2D(parameters, 0, ind++))); 
		double x=cvGetReal2D(parameters, 0, ind++); 
		double y=cvGetReal2D(parameters, 0, ind++); 
		double w=cvGetReal2D(parameters, 0, ind++);
		double h=cvGetReal2D(parameters, 0, ind++);
		cvSetReal2D(boxX, 0, i,x ); 
		cvSetReal2D(boxY, 0, i,y); 
		cvSetReal2D(width, 0, i,w); 
		cvSetReal2D(height, 0, i,h); 
		
		cvSetReal2D(boxX1, 0, i, max(ceil(x) - ceil((w-1.0)/2.0), 1)-1);
		cvSetReal2D(boxX2, 0, i, min(ceil(x) + floor((w-1.0)/2.0), patchSize.width)-1);
		cvSetReal2D(boxY1, 0, i, max(ceil(y) - ceil((h-1.0)/2.0), 1)-1);
		cvSetReal2D(boxY2, 0, i, min(ceil(y) + floor((h-1.0)/2.0), patchSize.height)-1);
	}	
	
	if (_BOXFEATURE_DEBUG) cout<<"Setting global feature params." << endl; 
	hasHSym = round(cvGetReal2D(parameters, 0, ind++)); 
	hasVSym = round(cvGetReal2D(parameters, 0, ind++)); 
	meanSub = round(cvGetReal2D(parameters, 0, ind++)); 
	normBrightness = round(cvGetReal2D(parameters, 0, ind++)); 
	
	keepBoxesInPatch(); 
	expandSymmetries(); 
	setKernelVisualization(); 
}

void BoxFeature::keepBoxesInPatch() {
	for (int i = 0; i < numBoxes; i++) {
		cvSetReal2D(boxX1, 0, i, max(cvGetReal2D(boxX1, 0, i), 0)); 
		cvSetReal2D(boxX1, 0, i, min(cvGetReal2D(boxX1, 0, i), patchSize.width-1)); 
		cvSetReal2D(boxX2, 0, i, max(cvGetReal2D(boxX2, 0, i), 0));
		cvSetReal2D(boxX2, 0, i, min(cvGetReal2D(boxX2, 0, i), patchSize.width-1)); 
		cvSetReal2D(boxY1, 0, i, max(cvGetReal2D(boxY1, 0, i), 0)); 
		cvSetReal2D(boxY1, 0, i, min(cvGetReal2D(boxY1, 0, i), patchSize.height-1)); 
		cvSetReal2D(boxY2, 0, i, max(cvGetReal2D(boxY2, 0, i), 0)); 
		cvSetReal2D(boxY2, 0, i, min(cvGetReal2D(boxY2, 0, i), patchSize.height-1)); 
	}
}

void BoxFeature::expandSymmetries() {
	int mul = 1; 
	if (hasHSym != 0) mul = mul*2;  
	if (hasVSym != 0) mul = mul*2; 
	
	if (mul==1) return; 
	
	int newNumBoxes = numBoxes*mul;
	CvMat* newBoxX1 = cvCreateMat(1, newNumBoxes, CV_64FC1);  
	CvMat* newBoxX2 = cvCreateMat(1, newNumBoxes, CV_64FC1);  
	CvMat* newBoxY1 = cvCreateMat(1, newNumBoxes, CV_64FC1);  
	CvMat* newBoxY2 = cvCreateMat(1, newNumBoxes, CV_64FC1);
	CvMat* newWeights = cvCreateMat(1, newNumBoxes, CV_64FC1);
	
	int ind = 0; 
	
	for (int i = 0; i < numBoxes; i++) {
		int x1L = cvGetReal2D(boxX1, 0, i); 
		int y1T = cvGetReal2D(boxY1, 0, i);
		int x1R = cvGetReal2D(boxX2, 0, i); 
		int y1B = cvGetReal2D(boxY2, 0, i); 
		
		int w = x1R - x1L+1; 
		int h = y1B - y1T+1;
		int x2L = patchSize.width - x1L - w; 
		int y2T = patchSize.height - y1T - h;
		int x2R = x2L + w-1; 
		int y2B = y2T + h-1; 
		double s = cvGetReal2D(weights, 0, i); 
		
		cvSetReal2D(newBoxX1, 0, ind, x1L); 
		cvSetReal2D(newBoxY1, 0, ind, y1T);
		cvSetReal2D(newBoxX2, 0, ind, x1R); 
		cvSetReal2D(newBoxY2, 0, ind, y1B); 
		cvSetReal2D(newWeights, 0, ind, s); 
		ind++; 
		
		if (hasHSym != 0) {
			cvSetReal2D(newBoxX1, 0, ind, x2L); 
			cvSetReal2D(newBoxY1, 0, ind, y1T);
			cvSetReal2D(newBoxX2, 0, ind, x2R); 
			cvSetReal2D(newBoxY2, 0, ind, y1B); 
			cvSetReal2D(newWeights, 0, ind, hasHSym*s); 
			ind++; 
		}
		
		if (hasVSym != 0) {
			cvSetReal2D(newBoxX1, 0, ind, x1L); 
			cvSetReal2D(newBoxY1, 0, ind, y2T);
			cvSetReal2D(newBoxX2, 0, ind, x1R); 
			cvSetReal2D(newBoxY2, 0, ind, y2B); 
			cvSetReal2D(newWeights, 0, ind, hasVSym*s); 
			ind++; 
		}
		
		if (hasHSym != 0 && hasVSym != 0) {
			cvSetReal2D(newBoxX1, 0, ind, x2L); 
			cvSetReal2D(newBoxY1, 0, ind, y2T);
			cvSetReal2D(newBoxX2, 0, ind, x2R); 
			cvSetReal2D(newBoxY2, 0, ind, y2B); 
			cvSetReal2D(newWeights, 0, ind, hasHSym*hasVSym*s); 
			ind++; 
		}
		
	}
	
	cvReleaseMat(&boxX1);
	cvReleaseMat(&boxX2);
	cvReleaseMat(&boxY1);
	cvReleaseMat(&boxY2); 
	cvReleaseMat(&weights);
	
	boxX1 = newBoxX1; 
	boxX2 = newBoxX2; 
	boxY1 = newBoxY1; 
	boxY2 = newBoxY2; 
	weights = newWeights; 
	
	numBoxes = newNumBoxes; 
	hasHSym = 0; 
	hasVSym = 0; 
	
	keepBoxesInPatch(); 
}

void BoxFeature::setKernelVisualization() {
	cvSetZero(kernel); 
	filterEnergy = 0; 
	for (int i = 0; i < numBoxes; i++) {
		int x = cvGetReal2D(boxX1, 0, i); 
		int y = cvGetReal2D(boxY1, 0, i); 
		int w = cvGetReal2D(boxX2, 0, i) - x+1; 
		int h = cvGetReal2D(boxY2, 0, i) - y+1;
		//int x1 = patchSize.width - x - w; 
		//int y1 = patchSize.height - y - h; 
		double s = cvGetReal2D(weights, 0, i); 
		if (s==0) continue; 
		cvSetImageROI(kernel, cvRect(x, y, w, h)); 
		cvAddS(kernel, cvRealScalar(s), kernel); 
		filterEnergy += w*h*s; 
	}
	cvResetImageROI(kernel); 
}



double BoxFeature::evaluateImagePatch(const ImagePatch* patch){
	int c1,c2,c3,c4;//, d; 
	
	Size filterSize = patch->getImageSize(); 
	int fwidth = filterSize.width; 
	int fheight = filterSize.height; 
	int integralWidthStep = patch->integralDataRowWidth(); 
	int ibytes = patch->integralDataRowBytes(); 
	
	if (_BOXFEATURE_DEBUG) cout << "Integral data row width is " << integralWidthStep << "(" << ibytes << " bytes)." << endl;
	
	const integral_type* integralData = patch->startOfIntegralData(); 
	
	
	double area = fwidth*fheight; 
	
	
	double retval = 0; 
	double widthRatio = fwidth*1.0/patchSize.width; 
	double heightRatio = fheight*1.0/patchSize.height; 
	double ratio = 1.0/(widthRatio*heightRatio); 
	
	
	double origEnergy = 0;
	double currEnergy = 0; 
	
	if (_BOXFEATURE_DEBUG) cout << "Ratio is " << ratio << endl;
	
	CvSize integralSize = patch->getIntegralSize(); 
	int maxind = integralSize.width*integralSize.height; 
	
	for (int n = 0; n < numBoxes; n++) {
		double s = cvGetReal2D(weights, 0, n); 		
		if (s==0) continue; 
		
		
		int xL = floor(cvGetReal2D(boxX1, 0, n)*widthRatio); 
		int yT = floor(cvGetReal2D(boxY1, 0, n)*heightRatio);
		int xR = floor(cvGetReal2D(boxX2, 0, n)*widthRatio); 
		int yB = floor(cvGetReal2D(boxY2, 0, n)*heightRatio); 
		xR++; 
		yB++; 
		
		
		double origBoxArea = (cvGetReal2D(boxX2, 0, n)-cvGetReal2D(boxX1, 0, n)+1)*(cvGetReal2D(boxY2, 0, n)-cvGetReal2D(boxY1, 0, n)+1); 
		double currBoxArea = (xR-xL)*(yB-yT);
		
		if (_BOXFEATURE_DEBUG) cout << "For box " << n<< ", origArea is " << origBoxArea << ", currArea is " << currBoxArea << endl;
		if (ratio !=1) s = s * origBoxArea / currBoxArea; 
		
		origEnergy += s*(cvGetReal2D(boxX2, 0, n)-cvGetReal2D(boxX1, 0, n)+1)*(cvGetReal2D(boxY2, 0, n)-cvGetReal2D(boxY1, 0, n)+1); //area of original filter, times filter weight
		currEnergy += s*(xR-xL)*(yB-yT);  //area of effective filter, times filter weight
		
		
		
		c1 = yT*integralWidthStep+xL; 
		c2 = yT*integralWidthStep+xR; 
		c3 = yB*integralWidthStep+xL; 
		c4 = yB*integralWidthStep+xR; 
		 
		if (c1 < 0 || c1 >= maxind || c2 < 0 || c2 >= maxind || c3 < 0 || c3 >= maxind || c4 < 0 || c4 >= maxind) {
			cout << "!!!!!! Error: integral indices are out of bounds! (" << c1 << ", " << c2 << ", " << c3 << ", " << c4 << ")" << endl; 
			exit(0); 
		}
		
		retval += s*(integralData[c1]-integralData[c2]-integralData[c3]+integralData[c4]); 
		
		if (retval > 1e8 || retval < -1e8) {
			 
			//patch->testImagePatch(); 
			cout << "!!!!!! Integral indices are  (" << c1 << ", " << c2 << ", " << c3 << ", " << c4 << "); s=" << s << endl; 
			cout << "!!!!!! Integral values are " << integralData[c1] << ", "  << integralData[c2] << ", "  << integralData[c3] << ", "  << integralData[c4] <<endl; 
		}	
	}
	
	
	if (_BOXFEATURE_DEBUG) cout << "currEnergy / origEnergy / filterEnergy are " << currEnergy << "/" << origEnergy << "/" << filterEnergy << endl;
	
	if (meanSub || normBrightness ) {
		
		if (_BOXFEATURE_DEBUG) cout << "Before normalization, retval is " << retval << endl;
		double norm; 
		c1 = 0; 
		c2 = fwidth; 
		c3 = fheight*integralWidthStep; 
		c4 = fheight*integralWidthStep+fwidth; //Is this a bug? should it be +1?
		
		//double energy = filterEnergy/ratio; 
		double energy = currEnergy; 
		
		
		if (normBrightness) {
			norm = integralData[c1]-integralData[c2]+integralData[c4]-integralData[c3]+1; 
			if (meanSub && energy != 0)
				retval = retval / norm * area - energy; 
			else
				retval = retval / norm * area; 
		} else {
			
			double mul = energy / area; 
			norm = integralData[c1]-integralData[c2]+integralData[c4]-integralData[c3]; 
			retval = retval - norm*mul; 
		}
	}
	
	if (_BOXFEATURE_DEBUG) cout << "Feature output is " << retval << endl;
	
	if (retval > 1e8 || retval < -10e8) {
		
		if (_BOXFEATURE_DEBUG) cout << "!!!!!!! Feature output is huge: " << retval << endl;
	}
	
	return retval; 
}

void BoxFeature::filterPatchList( PatchList* patches){
	if (_BOXFEATURE_DEBUG) cout << "Starting box feature filter" << endl; 
	int first = 1; 	
	
	//note: the type of these should be changed if we ever expect a single image to require 64-bit addressing.
	size_t c1,c2,c3,c4; 
	integral_type *si; 
	
	Size filterSize = patches->getFilterSizeAtScale(); //patches->getPatchSizeAtScale(patches->currentScale); 
	//CvSize integralSize = patches->getIntegralSize(); 
	int fwidth = filterSize.width; 
	int fheight = filterSize.height; 
	int integralWidthStep = patches->getIntegralWidthStepAtScale(); 
	
	double area = fwidth*fheight; 
	
	double widthRatio = fwidth*1.0/patchSize.width; 
	double heightRatio = fheight*1.0/patchSize.height; 
	double ratio = 1.0/(widthRatio*heightRatio); 
	
	//if (ratio!=1)
	if (_BOXFEATURE_DEBUG) 	cout << "Filter size ratio is " << ratio << endl; 
	
	int size = patches->getCurrentListLength(); 
	
	integral_type** sInds = &patches->srcInds[0]; 
	double** dInds = &patches->destInds[0]; 
	
	double origEnergy = 0;
	double currEnergy = 0; 
	
	
	if (_BOXFEATURE_DEBUG) cout << "Filter width is " << fwidth << "; Integral width step is " << integralWidthStep << endl; 
	
	for (int n = 0; n < numBoxes; n++) {
		if (_BOXFEATURE_DEBUG) cout << "Processing Box " << n << endl; 
		
		double s = cvGetReal2D(weights, 0, n); 
		if (s==0) continue; 
		
		int xL = floor(cvGetReal2D(boxX1, 0, n)*widthRatio);   //box x left corner
		int yT = floor(cvGetReal2D(boxY1, 0, n)*heightRatio);  //box y top corner
		int xR = floor(cvGetReal2D(boxX2, 0, n)*widthRatio);   //box x right corner
		int yB = floor(cvGetReal2D(boxY2, 0, n)*heightRatio);  //box y bottom corner
		xR++; //increment for integral index
		yB++; //increment for integral index
		
		double origBoxArea = (cvGetReal2D(boxX2, 0, n)-cvGetReal2D(boxX1, 0, n)+1)*(cvGetReal2D(boxY2, 0, n)-cvGetReal2D(boxY1, 0, n)+1); 
		double currBoxArea = (xR-xL)*(yB-yT);
		
		if (_BOXFEATURE_DEBUG) cout << "For box " << n<< ", origArea is " << origBoxArea << ", currArea is " << currBoxArea << endl;
		if (ratio !=1) s = s * origBoxArea / currBoxArea; 
		
		origEnergy += s*(cvGetReal2D(boxX2, 0, n)-cvGetReal2D(boxX1, 0, n)+1)*(cvGetReal2D(boxY2, 0, n)-cvGetReal2D(boxY1, 0, n)+1); //area of original filter, times filter weight
		currEnergy += s*(xR-xL)*(yB-yT);  //area of effective filter, times filter weight
		
		
		
		
		c1 = yT*integralWidthStep+xL; 
		c2 = yT*integralWidthStep+xR; 
		c3 = yB*integralWidthStep+xL; 
		c4 = yB*integralWidthStep+xR; 
		
		if (s==0) continue; 
//		int chunksize = 10000; 
		int i; 
		
//#pragma omp parallel shared(dInds, sInds, c1, c2, c3, c4, s, first, patches) private(si, i)
//		{
		
		if(first) { //If first, assign value to accumulator
			first =0; 
			if (s==1) {
//#pragma omp for schedule(dynamic, chunksize) nowait
				for ( i = 0; i < size; i++) {
					si = sInds[i]; 
					*dInds[i] = si[c1]-si[c2]-si[c3]+si[c4]; 
				}
			} else if (s==-1) {				
//#pragma omp for schedule(dynamic, chunksize) nowait
				for ( i = 0; i < size; i++) {
					si = sInds[i]; 
					*dInds[i] = si[c2]-si[c1]-si[c4]+si[c3]; 
				}
			} else {
				
//#pragma omp for schedule(dynamic, chunksize) nowait
				for ( i = 0; i < size; i++) {
					si = sInds[i]; 
					*dInds[i] = s*(si[c1]-si[c2]-si[c3]+si[c4]);  
				}
			}
		} else { //Otherwise increment value to accumulator
			if (s==1) {
				
//#pragma omp for schedule(dynamic, chunksize) nowait
				for ( i = 0; i < size; i++) {
					si = sInds[i]; 
					*dInds[i] += si[c1]-si[c2]-si[c3]+si[c4]; 
				}
			} else if (s==-1) {
				
//#pragma omp for schedule(dynamic, chunksize) nowait
				for ( i = 0; i < size; i++) {
					si = sInds[i]; 
					*dInds[i] += si[c2]-si[c1]-si[c4]+si[c3]; 
				}
			} else {
				
//#pragma omp for schedule(dynamic, chunksize) nowait
				for ( i = 0; i < size; i++) {
					si = sInds[i]; 
					*dInds[i] += s*(si[c1]-si[c2]-si[c3]+si[c4]);  
				}
			}
		}
//		}
	}
	
	if (_BOXFEATURE_DEBUG) cout << "Normalizing." << endl; 
	if (meanSub || normBrightness) {
		double norm; 
		c1 = 0; 
		c2 = fwidth; //fwidth is 1+(fwidth-1)
		c3 = fheight*integralWidthStep;   //fheight is 1+(fheight-1).
		c4 = fheight*integralWidthStep+fwidth; 
		
		//double energy =filterEnergy/ratio; 
		double energy = currEnergy; 
		
		if (normBrightness) {
			if (meanSub && energy !=0) {
				for (int i = 0; i < size; i++) {
					si = sInds[i]; 
					norm =si[c1]-si[c2]-si[c3]+si[c4]+1;
					
					*dInds[i] = *dInds[i]/norm*area-energy; 
				}
			} else {
				for (int i = 0; i < size; i++) {
					si = sInds[i]; 
					norm =si[c1]-si[c2]-si[c3]+si[c4]+1;
					*dInds[i] = *dInds[i]/norm*area; 
				}
			}
		} else {
			
			double mul = energy / area; 

			if (_BOXFEATURE_DEBUG) cout << "Subtracting mean: ratio=" << ratio << "; energy = " << energy << "; area = " << area << "; mul = " << mul << endl; 	
			for (int i = 0; i < size; i++) {
				si = sInds[i]; 
				norm =si[c1]-si[c2]-si[c3]+si[c4];
				*dInds[i] -= norm*mul; 
			}
			if (_BOXFEATURE_DEBUG) cout << endl; 
		}
	}
	
	if (_BOXFEATURE_DEBUG) cout << "Ending box feature filter" << endl; 
}


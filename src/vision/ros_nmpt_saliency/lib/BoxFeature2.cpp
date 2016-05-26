/*
 *  BoxFeature2.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/22/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#include "BoxFeature2.h"
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

void BoxFeature2::init(cv::Size expectedPatchSize, int nBoxes) {
	if (_BOXFEATURE_DEBUG) cout<< "BoxFeature init." << endl; 
	featureName= "BoxFeature"; 
	patchSize = expectedPatchSize; 
	prefersIntegral = 1; 
	createWithNBoxes(nBoxes); 
	
	if (_BOXFEATURE_DEBUG) cout<< "Randomizing Initial Parameters." << endl; 
	setRandomParameterValues(); 	
	
	if (_BOXFEATURE_DEBUG) cout<< "Constructor Finished." << endl; 
}

BoxFeature2::BoxFeature2(cv::Size expectedPatchSize, int nBoxes) {
	if (_BOXFEATURE_DEBUG) cout<< "BoxFeature constructor(size, nboxes)." << endl; 
	init(expectedPatchSize, nBoxes); 
}

BoxFeature2::BoxFeature2(){
	if (_BOXFEATURE_DEBUG) cout<< "BoxFeature default constructor." << endl; 
	init(); 
}

BoxFeature2::BoxFeature2(const BoxFeature2 &rhs) {
	if (_BOXFEATURE_DEBUG) cout<< "BoxFeature copy constructor." << endl; 
	if (this != &rhs) {
		init(rhs.patchSize, rhs.numBoxes); 
		setFeatureParameters(rhs.parameters); 
	}
}

BoxFeature2 & BoxFeature2::operator=(const BoxFeature2 &rhs) {
	if (_BOXFEATURE_DEBUG) cout<< "BoxFeature assignment." << endl; 
	if (this != &rhs) {
		init(rhs.patchSize, rhs.numBoxes); 
		setFeatureParameters(rhs.parameters); 
	}
	return *this; 
}

void BoxFeature2::createWithNBoxes(int n) {

	
	if (_BOXFEATURE_DEBUG) cout<< "BoxFeature createWithNBoxes." << endl; 
	numBoxes = n; 
	
	/*
	boxX.create(1, numBoxes, CV_64F); 
	boxY.create(1, numBoxes, CV_64F); 
	width.create(1, numBoxes, CV_64F); 
	height.create(1, numBoxes, CV_64F); 
	*/
	parameters.create(1, 5*numBoxes+4, CV_64F); 
	
	minParamVals.create(1, 5*numBoxes+4, CV_64F); 
	for (int i = 0; i < numBoxes; i++) {
		minParamVals.at<double>(0,i*5)= -5; 
		minParamVals.at<double>(0,i*5+1)= 1; 
		minParamVals.at<double>(0,i*5+2)= 1; 
		minParamVals.at<double>(0,i*5+3)= 1; 
		minParamVals.at<double>(0,i*5+4)= 1; 
	}
	minParamVals.at<double>(0, 5*numBoxes) = -1; 
	minParamVals.at<double>(0, 5*numBoxes+1) = -1; 
	minParamVals.at<double>(0, 5*numBoxes+2) = 0; 
	minParamVals.at<double>(0, 5*numBoxes+3) = 0; 
	
	maxParamVals.create(1, 5*numBoxes+4, CV_64F); 
	for (int i = 0; i < numBoxes; i++) {
		maxParamVals.at<double>(0, i*5) = 5; 
		maxParamVals.at<double>(0, i*5+1) = patchSize.width; 
		maxParamVals.at<double>(0, i*5+2) = patchSize.height; 
		maxParamVals.at<double>(0, i*5+3) = patchSize.width; 
		maxParamVals.at<double>(0, i*5+4) = patchSize.height; 
	}
	maxParamVals.at<double>(0, 5*numBoxes) = 1; 
	maxParamVals.at<double>(0, 5*numBoxes+1) = 1; 
	maxParamVals.at<double>(0, 5*numBoxes+2) = 1; 
	maxParamVals.at<double>(0, 5*numBoxes+3) = 1; 
	
	
	/*
	boxX1.create(1, numBoxes, CV_64F);  
	boxX2.create(1, numBoxes, CV_64F);  
	boxY1.create(1, numBoxes, CV_64F);  
	boxY2.create(1, numBoxes, CV_64F);  
	 */
	
}

BoxFeature2::~BoxFeature2() {
}

void BoxFeature2::setFeatureParameters(const cv::Mat &paramVec) {	
	
	if (_BOXFEATURE_DEBUG) cout<< "Starting to Set BoxFeature parameters." << endl; 
	assert((paramVec.cols-numExtraFeatures) %numFeaturesPerBox == 0 && 
		   paramVec.rows == 1); 
	if (_BOXFEATURE_DEBUG) cout<< "Param Vec is the right size." << endl ;
	
	if (!(&paramVec == &parameters)) {
		int n = (paramVec.cols - numExtraFeatures)/numFeaturesPerBox; 
		if (n != numBoxes) createWithNBoxes(n); 			
		parameters = paramVec.clone();
	}
	
	if (_BOXFEATURE_DEBUG) cout<< "Making sure parameters are in proper range." << endl; 
	checkParameterBounds(parameters); 
	
	int ind = 0; 
	//boxes.clear(); 
	pt1.clear(); 
	pt2.clear(); 
	weights.create(1, numBoxes, CV_64F) ;
	for (int i = 0; i < numBoxes; i++) {
		if (_BOXFEATURE_DEBUG) cout<< "Setting data for box " << i << endl; 
		weights.at<double>(0,i) = sign(parameters.at<double>(0, ind++)); 
		//cvSetReal2D(weights, 0, i, sign(parameters.at<double>(0,  ind++))); 
		double x=parameters.at<double>(0, ind++); 
		double y=parameters.at<double>(0, ind++); 
		double w=parameters.at<double>(0, ind++);
		double h=parameters.at<double>(0, ind++);
		//boxes.push_back(Rect(x,y,w,h)); 
		pt1.push_back(Point(max(ceil(x) - ceil((w-1.0)/2.0), 1)-1,
							max(ceil(y) - ceil((h-1.0)/2.0), 1)-1));
		pt2.push_back(Point(min(ceil(x) + floor((w-1.0)/2.0), patchSize.width)-1,
							min(ceil(y) + floor((h-1.0)/2.0), patchSize.height)-1));
		/*
		cvSetReal2D(boxX1, 0, i, max(ceil(x) - ceil((w-1.0)/2.0), 1)-1);
		cvSetReal2D(boxX2, 0, i, min(ceil(x) + floor((w-1.0)/2.0), patchSize.width)-1);
		cvSetReal2D(boxY1, 0, i, max(ceil(y) - ceil((h-1.0)/2.0), 1)-1);
		cvSetReal2D(boxY2, 0, i, min(ceil(y) + floor((h-1.0)/2.0), patchSize.height)-1);
		 */
	}	
	
	if (_BOXFEATURE_DEBUG) cout<<"Setting global feature params." << endl; 
	hasHSym = round(parameters.at<double>(0, ind++)); 
	hasVSym = round(parameters.at<double>(0, ind++)); 
	meanSub = round(parameters.at<double>(0, ind++)); 
	normBrightness = round(parameters.at<double>(0, ind++)); 
	
	keepBoxesInPatch(); 
	expandSymmetries(); 
	setKernelVisualization(); 
}

void BoxFeature2::keepBoxesInPatch() {
	if (_BOXFEATURE_DEBUG) cout<<"Cropping BoxFeature boxes to patch size." << endl; 
	for (int i = 0; i < numBoxes; i++) {
		pt1[i].x = max(pt1[i].x, 0); 
		pt1[i].x = min(pt1[i].x, patchSize.width-1); 
		pt2[i].x = max(pt2[i].x, 0); 
		pt2[i].x = min(pt2[i].x, patchSize.width-1); 
		
		pt1[i].y = max(pt1[i].y, 0); 
		pt1[i].y = min(pt1[i].y, patchSize.height-1); 
		pt2[i].y = max(pt2[i].y, 0); 
		pt2[i].y = min(pt2[i].y, patchSize.height-1); 
		
		/*
		cvSetReal2D(boxX1, 0, i, max(cvGetReal2D(boxX1, 0, i), 0)); 
		cvSetReal2D(boxX1, 0, i, min(cvGetReal2D(boxX1, 0, i), patchSize.width-1)); 
		cvSetReal2D(boxX2, 0, i, max(cvGetReal2D(boxX2, 0, i), 0));
		cvSetReal2D(boxX2, 0, i, min(cvGetReal2D(boxX2, 0, i), patchSize.width-1)); 
		cvSetReal2D(boxY1, 0, i, max(cvGetReal2D(boxY1, 0, i), 0)); 
		cvSetReal2D(boxY1, 0, i, min(cvGetReal2D(boxY1, 0, i), patchSize.height-1)); 
		cvSetReal2D(boxY2, 0, i, max(cvGetReal2D(boxY2, 0, i), 0)); 
		cvSetReal2D(boxY2, 0, i, min(cvGetReal2D(boxY2, 0, i), patchSize.height-1)); 
		 */
	}
}

void BoxFeature2::expandSymmetries() {
	if (_BOXFEATURE_DEBUG) cout<<"Expanding BoxFeature symmetries." << endl; 
	int mul = 1; 
	if (hasHSym != 0) mul = mul*2;  
	if (hasVSym != 0) mul = mul*2; 
	
	if (mul==1) return; 
	
	int newNumBoxes = numBoxes*mul;
	
	vector<Point> npt1, npt2; 
	/*
	CvMat* newBoxX1.create(1, newNumBoxes, CV_64F);  
	CvMat* newBoxX2.create(1, newNumBoxes, CV_64F);  
	CvMat* newBoxY1.create(1, newNumBoxes, CV_64F);  
	CvMat* newBoxY2.create(1, newNumBoxes, CV_64F);
	CvMat* newWeights.create(1, newNumBoxes, CV_64F);
	*/
	Mat newWeights(1, newNumBoxes, CV_64F);
	int ind = 0; 
	
	for (int i = 0; i < numBoxes; i++) {
		int x1L = pt1[i].x; 
		int y1T = pt1[i].y;
		int x1R = pt2[i].x;
		int y1B = pt2[i].y; 
		
		int w = x1R - x1L+1; 
		int h = y1B - y1T+1;
		int x2L = patchSize.width - x1L - w; 
		int y2T = patchSize.height - y1T - h;
		int x2R = x2L + w-1; 
		int y2B = y2T + h-1; 
		
		double s = weights.at<double>( 0, i); 
		
		npt1.push_back(Point(x1L, y1T)); 
		npt2.push_back(Point(x1R, y1B)); 
		newWeights.at<double>(0,ind) = s; 
		/*
		cvSetReal2D(newBoxX1, 0, ind, x1L); 
		cvSetReal2D(newBoxY1, 0, ind, y1T);
		cvSetReal2D(newBoxX2, 0, ind, x1R); 
		cvSetReal2D(newBoxY2, 0, ind, y1B); 
		cvSetReal2D(newWeights, 0, ind, s); 
		 */
		ind++; 
		
		if (hasHSym != 0) {
			npt1.push_back(Point(x2L, y1T)); 
			npt2.push_back(Point(x2R, y1B)); 
			newWeights.at<double>(0,ind) = hasHSym*s; 
			/*
			cvSetReal2D(newBoxX1, 0, ind, x2L); 
			cvSetReal2D(newBoxY1, 0, ind, y1T);
			cvSetReal2D(newBoxX2, 0, ind, x2R); 
			cvSetReal2D(newBoxY2, 0, ind, y1B); 
			cvSetReal2D(newWeights, 0, ind, hasHSym*s); 
			 */
			ind++; 
		}
		
		if (hasVSym != 0) {
			npt1.push_back(Point(x1L,y2T)); 
			npt2.push_back(Point(x1R,y2B)); 
			newWeights.at<double>(0,ind) = hasVSym*s; 
			/*
			cvSetReal2D(newBoxX1, 0, ind, x1L); 
			cvSetReal2D(newBoxY1, 0, ind, y2T);
			cvSetReal2D(newBoxX2, 0, ind, x1R); 
			cvSetReal2D(newBoxY2, 0, ind, y2B); 
			cvSetReal2D(newWeights, 0, ind, hasVSym*s); 
			 */
			ind++; 
		}
		
		if (hasHSym != 0 && hasVSym != 0) {
			npt1.push_back(Point(x2L,y2T)); 
			npt2.push_back(Point(x2R,y2B)); 
			newWeights.at<double>(0,ind) = hasHSym*hasVSym*s; 
			/*
			cvSetReal2D(newBoxX1, 0, ind, x2L); 
			cvSetReal2D(newBoxY1, 0, ind, y2T);
			cvSetReal2D(newBoxX2, 0, ind, x2R); 
			cvSetReal2D(newBoxY2, 0, ind, y2B); 
			cvSetReal2D(newWeights, 0, ind, hasHSym*hasVSym*s); 
			 */
			ind++; 
		}
		
	}
	
	weights = newWeights.clone(); 
	pt1 = npt1; 
	pt2 = npt2; 
	/*
	
	boxX1 = newBoxX1; 
	boxX2 = newBoxX2; 
	boxY1 = newBoxY1; 
	boxY2 = newBoxY2; 
	weights = newWeights; 
	*/
	numBoxes = newNumBoxes; 
	hasHSym = 0; 
	hasVSym = 0; 
	
	keepBoxesInPatch(); 
}

void BoxFeature2::setKernelVisualization() {
	
	if (_BOXFEATURE_DEBUG) cout<<"Setting BoxFeature visualization." << endl; 
	kernel = Mat::zeros(patchSize, CV_64F); 
	filterEnergy = 0; 
	for (int i = 0; i < numBoxes; i++) {
		int x = pt1[i].x; //cvGetReal2D(boxX1, 0, i); 
		int y = pt1[i].y; //cvGetReal2D(boxY1, 0, i); 
		int w = pt2[i].x - x + 1; //cvGetReal2D(boxX2, 0, i) - x+1; 
		int h = pt2[i].y - y + 1; //cvGetReal2D(boxY2, 0, i) - y+1;
		//int x1 = patchSize.width - x - w; 
		//int y1 = patchSize.height - y - h; 
		double s = weights.at<double>( 0, i); 
		if (s==0) continue; 
		Mat roi = kernel(Rect(x, y, w, h)); 
		roi += s; 
		//cvAddS(kernel, cvRealScalar(s), kernel); 
		filterEnergy += w*h*s; 
	}
	//cvResetImageROI(kernel); 
}

double BoxFeature2::evaluateImagePatch(const ImagePatch2 &patch) const{
	int c1,c2,c3,c4;//, d; 
	
	const Mat intmat = patch.getIntegralHeader(); 
	
	Size filterSize = patch.getImageSize(); 
	int fwidth = filterSize.width; 
	int fheight = filterSize.height; 
	int integralWidthStep = intmat.step/sizeof(int); 
	int ibytes = intmat.step; 
	
	if (_BOXFEATURE_DEBUG) cout << "Integral data row width is " << integralWidthStep << "(" << ibytes << " bytes)." << endl;
	
	const int* integralData = (const int*)intmat.data; 
	
	
	double area = fwidth*fheight; 
	
	
	double retval = 0; 
	double widthRatio = fwidth*1.0/patchSize.width; 
	double heightRatio = fheight*1.0/patchSize.height; 
	double ratio = 1.0/(widthRatio*heightRatio); 
	
	
	double origEnergy = 0;
	double currEnergy = 0; 
	
	if (_BOXFEATURE_DEBUG) cout << "Ratio is " << ratio << endl;
	
	Size integralSize = intmat.size(); 
	int maxind = integralSize.width*integralSize.height; 
	
	for (int n = 0; n < numBoxes; n++) {
		double s = weights.at<double>(0, n); 		
		if (s==0) continue; 
		
		/*
		int xL = floor(cvGetReal2D(boxX1, 0, n)*widthRatio); 
		int yT = floor(cvGetReal2D(boxY1, 0, n)*heightRatio);
		int xR = floor(cvGetReal2D(boxX2, 0, n)*widthRatio); 
		int yB = floor(cvGetReal2D(boxY2, 0, n)*heightRatio); 
		*/
		int xL = floor(pt1[n].x*widthRatio); 
		int yT = floor(pt1[n].y*heightRatio);
		int xR = floor(pt2[n].x*widthRatio); 
		int yB = floor(pt2[n].y*heightRatio); 
		xR++; 
		yB++; 
		
		
		//double origBoxArea = (cvGetReal2D(boxX2, 0, n)-cvGetReal2D(boxX1, 0, n)+1)*(cvGetReal2D(boxY2, 0, n)-cvGetReal2D(boxY1, 0, n)+1); 
		double origBoxArea = (pt2[n].x-pt1[n].x+1)*(pt2[n].y - pt1[n].y+1); 
		double currBoxArea = (xR-xL)*(yB-yT);
	
		
		if (_BOXFEATURE_DEBUG) cout << "For box " << n<< ", origArea is " << origBoxArea << ", currArea is " << currBoxArea << endl;
		if (ratio !=1) s = s * origBoxArea / currBoxArea; 
		
		//origEnergy += s*(cvGetReal2D(boxX2, 0, n)-cvGetReal2D(boxX1, 0, n)+1)*(cvGetReal2D(boxY2, 0, n)-cvGetReal2D(boxY1, 0, n)+1); //area of original filter, times filter weight
		//currEnergy += s*(xR-xL)*(yB-yT);  //area of effective filter, times filter weight
		origEnergy += s*origBoxArea; 
		currEnergy += s*currBoxArea; 		
		
		
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

void BoxFeature2::filterPatchList( PatchList2 *patches) const{
	if (_BOXFEATURE_DEBUG) cout << "Starting box feature filter" << endl; 
	int first = 1; 	
	
	//note: the type of these should be changed if we ever expect a single image to require 64-bit addressing.
	size_t c1,c2,c3,c4; 
	const int *si; 
	
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
	
	vector<const int*> &sInds = patches->srcInds;  
	vector<double*> &dInds = patches->destInds; 
	
	double origEnergy = 0;
	double currEnergy = 0; 
	
	
	if (_BOXFEATURE_DEBUG) cout << "Filter width is " << fwidth << "; Integral width step is " << integralWidthStep << endl; 
	
	for (int n = 0; n < numBoxes; n++) {
		if (_BOXFEATURE_DEBUG) cout << "Processing Box " << n << endl; 
		
		double s = weights.at<double>( 0, n); 
		if (s==0) continue; 
		
		/*
		int xL = floor(cvGetReal2D(boxX1, 0, n)*widthRatio);   //box x left corner
		int yT = floor(cvGetReal2D(boxY1, 0, n)*heightRatio);  //box y top corner
		int xR = floor(cvGetReal2D(boxX2, 0, n)*widthRatio);   //box x right corner
		int yB = floor(cvGetReal2D(boxY2, 0, n)*heightRatio);  //box y bottom corner
		 */
		int xL = floor(pt1[n].x*widthRatio); 
		int yT = floor(pt1[n].y*heightRatio);
		int xR = floor(pt2[n].x*widthRatio); 
		int yB = floor(pt2[n].y*heightRatio); 
		xR++; //increment for integral index
		yB++; //increment for integral index
		
		
		//double origBoxArea = (cvGetReal2D(boxX2, 0, n)-cvGetReal2D(boxX1, 0, n)+1)*(cvGetReal2D(boxY2, 0, n)-cvGetReal2D(boxY1, 0, n)+1); 
		double origBoxArea = (pt2[n].x-pt1[n].x+1)*(pt2[n].y - pt1[n].y+1); 
		double currBoxArea = (xR-xL)*(yB-yT);
		
		if (_BOXFEATURE_DEBUG) cout << "For box " << n<< ", origArea is " << origBoxArea << ", currArea is " << currBoxArea << endl;
		if (ratio !=1) s = s * origBoxArea / currBoxArea; 
		
		//origEnergy += s*(cvGetReal2D(boxX2, 0, n)-cvGetReal2D(boxX1, 0, n)+1)*(cvGetReal2D(boxY2, 0, n)-cvGetReal2D(boxY1, 0, n)+1); //area of original filter, times filter weight
		//currEnergy += s*(xR-xL)*(yB-yT);  //area of effective filter, times filter weight
		origEnergy += s*origBoxArea; 
		currEnergy += s*currBoxArea; 
		
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


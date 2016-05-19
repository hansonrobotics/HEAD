#include "FastSalience.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <sys/types.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "NMPTUtils.h" 
#include "BlockTimer.h"
#include "DebugGlobals.h"

#define SAL_FILTER_TYPE CV_32F

using namespace std; 
using namespace cv; 

FastSalience::FastSalience(int numtemporal, int numspatial, float firsttau, int firstrad){
	init(numtemporal, numspatial, firsttau, firstrad); 
	
}

void FastSalience::copy(const FastSalience &rhs) {
	if (_SALIENCE_DEBUG) cout << "Calling copy" << endl; 
	init(rhs.ntemporal, rhs.nspatial, rhs.tau0, rhs.rad0); 
	useDoB = rhs.useDoB; 
	useDoE = rhs.useDoE; 
	useColor = rhs.useColor; 
	useParams = rhs.useParams; 
	estParams = rhs.estParams; 
	power = rhs.power; 
	
	tau = rhs.tau.clone(); 
	rad = rhs.rad.clone(); 
	absMeanDoBI = rhs.absMeanDoBI.clone(); 
	absMeanDoBRG = rhs.absMeanDoBRG.clone(); 
	absMeanDoBBY = rhs.absMeanDoBBY.clone(); 
	absMeanDoEI = rhs.absMeanDoEI.clone(); 
	absMeanDoERG = rhs.absMeanDoERG.clone(); 
	absMeanDoEBY = rhs.absMeanDoEBY.clone(); 
	meanDoBI = rhs.meanDoBI.clone(); 
	meanDoBRG = rhs.meanDoBRG.clone(); 
	meanDoBBY = rhs.meanDoBBY.clone(); 
	meanDoEI = rhs.meanDoEI.clone(); 
	meanDoERG = rhs.meanDoERG.clone(); 
	meanDoEBY = rhs.meanDoEBY.clone(); 
	
	ALPHA = rhs.ALPHA; 
	if (_SALIENCE_DEBUG) cout << "Copied" << endl; 
	
}


FastSalience & FastSalience::operator=(const FastSalience &rhs) {
	if (this != &rhs) {
		copy(rhs); 
	}
	return *this; 
}


FastSalience::FastSalience() {
	init(); 
}

FastSalience::FastSalience(const FastSalience &rhs) {
	if (this != &rhs) {
		copy(rhs); 
	} 
}


void FastSalience::init(int numtemporal, int numspatial, float firsttau, int firstrad) {
	ALPHA = .95; 
	
	ntemporal = numtemporal;
	nspatial = numspatial;
	tau0 = firsttau;
	rad0 = firstrad; 
	
	useDoB = 1; 
	useDoE = 1; 
	useColor = 1; 
	useParams = 1; 
	estParams = 1; 
	power = 1; 
	
	tau.create(1,ntemporal,CV_64F); 
	tau.at<double>(0,0) = tau0; 
	for (int i=1; i<ntemporal; i++) {
		tau.at<double>(0,i)=2*tau.at<double>(i-1);
	}
	
	//scale parameters
	rad.create(1,nspatial+1, CV_64F); 
	rad.at<double>(0,0) = rad0;
	
	for (int i=1; i<=nspatial; i++) {
		rad.at<double>(0,i)=2*rad.at<double>(0,i-1);
		if (i==1 && rad0 == 0) rad.at<double>(0,i)+=1; 
	}
	rad = 2*rad+1; 
	
	int maxSpScale= rad.at<double>(0,nspatial); 
	channelFilter = OpenCV2BoxFilter(maxSpScale/2, SAL_FILTER_TYPE); 	
	
	redBoxConvolutionAtScale.resize(nspatial+1);
	greenBoxConvolutionAtScale.resize(nspatial+1);
	blueBoxConvolutionAtScale.resize(nspatial+1);
	
	
	temporalImageI.clear();
	temporalImageRG.clear();
	temporalImageBY.clear();
	temporalImageI.resize(nspatial*ntemporal);
	temporalImageRG.resize(nspatial*ntemporal);
	temporalImageBY.resize(nspatial*ntemporal);
	
	meanDoBI = Mat::zeros(1,nspatial, CV_64F); 
	meanDoBRG = Mat::zeros(1,nspatial, CV_64F); 
	meanDoBBY = Mat::zeros(1,nspatial, CV_64F); 
	
	absMeanDoBI = Mat::ones(1,nspatial, CV_64F); 
	absMeanDoBRG = Mat::ones(1,nspatial, CV_64F); 
	absMeanDoBBY = Mat::ones(1,nspatial, CV_64F); 
	
	meanDoEI = Mat::zeros(ntemporal, nspatial, CV_64F); 
	meanDoERG = Mat::zeros(ntemporal, nspatial, CV_64F); 
	meanDoEBY = Mat::zeros(ntemporal, nspatial, CV_64F); 
	
	absMeanDoEI = Mat::ones(ntemporal, nspatial, CV_64F); 
	absMeanDoERG = Mat::ones(ntemporal, nspatial, CV_64F); 
	absMeanDoEBY = Mat::ones(ntemporal, nspatial, CV_64F); 
}

FastSalience::~FastSalience() {	
}

void FastSalience::setUseDoEFeatures(int flag) { useDoE = flag;} 
void FastSalience::setUseDoBFeatures(int flag) { useDoB = flag;} 
void FastSalience::setUseColorInformation(int flag) {useColor = flag; } 
void FastSalience::setUseGGDistributionParams(int flag) {useParams = flag;} 
void FastSalience::setEstimateGGDistributionParams(int flag) {estParams = flag;} 
void FastSalience::setGGDistributionPower(double value) {power = value;} 

void FastSalience::calcLogProb(Mat &logProb, Mat &accum, const Mat &scaleBig, const Mat &scaleSmall, double &meanEst, double &absMeanEst ) const {
	double meanval, absMean; 
	if (scaleBig.rows > 0 && scaleBig.cols > 0 && scaleSmall.rows > 0 && scaleSmall.cols > 0)
		logProb = scaleBig - scaleSmall; 
	if (estParams) {
		meanval = mean(logProb)[0];
		meanEst = ALPHA*meanEst + (1-ALPHA)*meanval; 
	}
	if (useParams) {
		logProb -= meanEst;			
	}
	
	logProb = abs(logProb); 
	if (power != 1.0) {
		pow(logProb,power,logProb); 
	}
	if (estParams) {
		absMean = mean(logProb)[0];
		absMeanEst = ALPHA*absMeanEst + (1-ALPHA)*absMean; 
	}
	if (useParams) {
		double wt = 1.0/absMeanEst; 
		logProb *= wt; 
	}
	accum += logProb; 
}


void FastSalience::detect( const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask )  {
	updateSalience(image); 
	vector<KeyPoint> temp = getKeyPoints(2); 
	keypoints.insert(keypoints.end(), temp.begin(), temp.end()); 
}

void FastSalience::read( const FileNode& fn ) {
	
}
void FastSalience::write( FileStorage& fs ) const {
}


void FastSalience::updateSalience(const Mat &frame)  {
	
	Mat colorframe = frame; 
	//frame.convertTo(colorframe, CV_32F); 
	if (colorframe.channels() ==3) {
		if (!useColor) {
			cvtColor(colorframe, redChannel, CV_BGR2GRAY); 
		} else {
			vector<Mat> channels(3); 
			channels[0] = blueChannel; 
			channels[1] = greenChannel; 
			channels[2] = redChannel;  
			split(colorframe, channels); 
			if (blueChannel.data != channels[0].data || greenChannel.data != channels[1].data || 
				redChannel.data != channels[2].data) {
				blueChannel = channels[0]; 
				greenChannel = channels[1]; 
				redChannel = channels[2]; 
			}
		}
	} else {
		useColor = 0; 
		redChannel = colorframe; 
	}
	
	//Do Box Convolutions at each scale
	channelFilter.setNewImage(redChannel); 
	if (_SALIENCE_DEBUG) cout << "Filtering" << endl;  ; 
	
	for (int i = 0; i < nspatial+1; i++) {
		int width = rad.at<double>(0,i); 
		int halfwidth = width/2; 
		Rect boxPosition(-halfwidth, -halfwidth, width, width); 
		channelFilter.setBoxFilter(redBoxConvolutionAtScale[i], boxPosition, 1.0/(boxPosition.width*boxPosition.height)); 
	}
	
	if (useColor) {
		channelFilter.setNewImage(greenChannel); 
		for (int i = 0; i < nspatial+1; i++) {
			//Mat m = greenBoxConvolutionAtScale[i]; 
			int width = rad.at<double>(0,i); 
			int halfwidth = width/2; 
			Rect boxPosition(-halfwidth, -halfwidth, width, width); 
			channelFilter.setBoxFilter(greenBoxConvolutionAtScale[i], boxPosition, 1.0/(boxPosition.width*boxPosition.height)); 
		}
		
		channelFilter.setNewImage(blueChannel); 
		for (int i = 0; i < nspatial+1; i++) {
			int width = rad.at<double>(0,i); 
			int halfwidth = width/2; 
			Rect boxPosition(-halfwidth, -halfwidth, width, width); 
			channelFilter.setBoxFilter(blueBoxConvolutionAtScale[i], boxPosition, 1.0/(boxPosition.width*boxPosition.height)); 
		}
	}
	
	salImageDouble.create(colorframe.size(), SAL_FILTER_TYPE); 
	salImageDouble = 0.; 
	
	
	if (temporalImageI[0].cols != frame.cols || temporalImageI[0].rows != frame.rows) {
		for (size_t i = 0; i < temporalImageI.size(); i++) {
			if (_SALIENCE_DEBUG) cout << "For temporal scale "<< i << " resetting temporal images." << endl; 
			temporalImageRG[i] = Mat::zeros(frame.size(), SAL_FILTER_TYPE);  
			temporalImageBY[i] = Mat::zeros(frame.size(), SAL_FILTER_TYPE);  
			temporalImageI[i] = Mat::zeros(frame.size(), SAL_FILTER_TYPE); 
		}
	}
	
	//Compute DoB features for each scale
	for (int i=0; i<nspatial; i++) {
		if (_SALIENCE_DEBUG) cout << "For spatial scale "<< i << endl; 
		if (!useColor) {
			iDoB = redBoxConvolutionAtScale[i+1] - redBoxConvolutionAtScale[i]; 
		} else {
			if (_SALIENCE_DEBUG) cout << "Computing color contrasts" << endl; 
			iDoB = .59 * greenBoxConvolutionAtScale[i+1]; 
			iDoB += .3 * redBoxConvolutionAtScale[i+1]; 
			iDoB += .11 * blueBoxConvolutionAtScale[i+1]; 
			iDoB -= .59 * greenBoxConvolutionAtScale[i]; 
			iDoB -= .3 * redBoxConvolutionAtScale[i]; 
			iDoB -= .11 * blueBoxConvolutionAtScale[i]; 
			rgDoB = redBoxConvolutionAtScale[i]; 
			rgDoB -= greenBoxConvolutionAtScale[i];
			rgDoB += greenBoxConvolutionAtScale[i+1]; 
			rgDoB -=  redBoxConvolutionAtScale[i+1]; 
			byDoB = blueBoxConvolutionAtScale[i]; 
			byDoB -= .5*redBoxConvolutionAtScale[i];
			byDoB -= .5*greenBoxConvolutionAtScale[i]; 
			byDoB -= blueBoxConvolutionAtScale[i+1]; 
			byDoB += .5*redBoxConvolutionAtScale[i+1];
			byDoB += .5*greenBoxConvolutionAtScale[i+1]; 
		} //end useColor
		
		
		// update temporal responses
		for (int j=0; j<ntemporal; j++) {
			if (_SALIENCE_DEBUG) cout << "Updating temporal response " << j << endl; 
			double a = tau.at<double>(0,j); 
			double b = 1.0/(1.0+a); // b = 1/(1+tau)
			a = a *b;               // a = tau / (1+tau)
			
			addWeighted(iDoB, a, temporalImageI[i*ntemporal+j], b,  
						0,temporalImageI[i*ntemporal+j]); 
		}
		
		if (useColor) {
			for (int j=0; j<ntemporal; j++) {
				if (_SALIENCE_DEBUG) cout << "Updating temporal response rg " << (i*ntemporal+j) << endl; 
				double a = tau.at<double>(0,j); 
				double b = 1.0/(1.0+a); // b = 1/(1+tau)
				a = a *b;               // a = tau / (1+tau)
				addWeighted(rgDoB, a, temporalImageRG[i*ntemporal+j], b, 
							0,temporalImageRG[i*ntemporal+j]);  
			}
			for (int j=0; j<ntemporal; j++) {
				if (_SALIENCE_DEBUG) cout << "Updating temporal response by " << (i*ntemporal+j) << endl; 
				double a = tau.at<double>(0,j); 
				double b = 1.0/(1.0+a); // b = 1/(1+tau)
				a = a *b;               // a = tau / (1+tau)
				addWeighted(byDoB, a, temporalImageBY[i*ntemporal+j], b, 
							0,temporalImageBY[i*ntemporal+j]); 
			}
		} //end useColor
		
		
		if (useDoE) {
			
			if (_SALIENCE_DEBUG) cout << "Computing DoE filters probs " << endl; 
			//update saliency
			for (int j=1; j<ntemporal; j++) {	
				//void FastSalience::calcLogProb(Mat &logProb, Mat &accum, const Mat &scaleBig, const Mat &scaleSmall, double &meanEst, double &absMeanEst ) {
				
				calcLogProb(DoE, salImageDouble, temporalImageI[i*ntemporal+j], 
							temporalImageI[i*ntemporal+j-1], meanDoEI.at<double>(j,i), 
							absMeanDoEI.at<double>(j,i)); 
			}
			
			if (useColor) {
				for (int j=1; j<ntemporal; j++) {	
					calcLogProb(DoE, salImageDouble, temporalImageRG[i*ntemporal+j], 
								temporalImageRG[i*ntemporal+j-1], meanDoERG.at<double>(j,i), 
								absMeanDoERG.at<double>(j,i)); 
					
				}
				
				for (int j=1; j<ntemporal; j++) {	
					calcLogProb(DoE, salImageDouble, temporalImageBY[i*ntemporal+j], 
								temporalImageBY[i*ntemporal+j-1], meanDoEBY.at<double>(j,i), 
								absMeanDoEBY.at<double>(j,i)); 
				}
			} //end useColor
		} //end useDoE
		
		if (useDoB) {
			if (_SALIENCE_DEBUG) cout << "Probs of DoB" << endl; 
			Mat dummy; 
			calcLogProb(iDoB, salImageDouble, dummy, dummy, meanDoBI.at<double>(0,i), absMeanDoBI.at<double>(0,i)); 
			if (useColor) {
				calcLogProb(rgDoB, salImageDouble, dummy, dummy, meanDoBRG.at<double>(0,i), absMeanDoBRG.at<double>(0,i)); 
				calcLogProb(byDoB, salImageDouble, dummy, dummy, meanDoBBY.at<double>(0,i), absMeanDoBBY.at<double>(0,i)); 
			}//end useColor			
			
		}//end useDoB 
		
	}//end scale
	if (_SALIENCE_DEBUG) {
	cout << "meanDoEI: " << endl; 
	NMPTUtils::printMat(meanDoEI); 
	cout << "absMeanDoEI: " << endl; 
	NMPTUtils::printMat(absMeanDoEI);
	cout << "meanDoBI: " << endl; 
	NMPTUtils::printMat(meanDoBI); 
	cout << "absMeanDoBI: " << endl; 
	NMPTUtils::printMat(absMeanDoBI);
	cout << " =====================" << endl; 
	}
}

void FastSalience::getSalImage(Mat &floatim) const {
	normalize(salImageDouble, floatim, 0, 1, NORM_MINMAX, CV_32F);
}

void FastSalience::getSalMap(Mat &image) const {
	image.create(salImageDouble.size(), salImageDouble.type());
	salImageDouble.copyTo(image); 
}

vector<KeyPoint> FastSalience::getKeyPoints(int radius) const{
	int borderr = 0.02*salImageDouble.rows; 
	int borderc = 0.02*salImageDouble.cols; 
	Mat salImg=salImageDouble;
	int w = salImg.cols; 
	int h = salImg.rows; 
	
	Mat kept(salImg.size(), CV_8U); 
	Mat cmp(salImg.size(), CV_8U); 
	
	double min, max;
	minMaxLoc(salImg, &min, &max);
	
	double minsalval = max/3; 
	kept = salImg >= minsalval; 
	
	
	int top1, top2,  left1, left2, h1, w1; 
	
	for (int i = -radius; i <= radius; i++) {
		if (i < 0) {
			top1 = 0; 
			top2 = -i; 
			h1 = h-1+i;
		} else {
			top2 = 0; 
			top1 = i;
			h1 = h-1-i;
		}
		for (int j = -radius; j <= radius; j++) {
			if (i==0 && j==0) continue; 
			
			if (j < 0) {
				left1 = 0; 
				left2 = -j; 
				w1 = w-1+j;
			} else {
				left2 = 0; 
				left1 = j;
				w1 = w-1-j;
			}
			
			Rect r1(left1, top1, w1, h1); 
			Rect r2(left2, top2, w1, h1); 
			
			Mat salROI1 = salImg(r1);
			Mat salROI2 = salImg(r2);
			
			Mat cmpROI = cmp(r1);
			Mat keptROI = kept(r1); 
			
			compare(salROI1, salROI2, cmpROI, CMP_GE);
			bitwise_and(cmpROI, keptROI, keptROI); 
		}
	}
	
	vector<KeyPoint> pts; 
	
	for (int i = borderr; i < h-borderr; i++) {
		for (int j = borderc; j < w-borderc; j++) {
			if (kept.at<uint8_t>( i, j) > 0) {
				KeyPoint pt; 
				pt.pt = Point(j,i); 
				pts.push_back(pt); 
			}
		}
	}
	
	return pts; 
}


cv::FileStorage& operator << (cv::FileStorage &fs, const FastSalience &sal){
	return fs;
}
cv::FileNode& operator >> (cv::FileNode &fs, FastSalience &sal) {
	return fs; 
}


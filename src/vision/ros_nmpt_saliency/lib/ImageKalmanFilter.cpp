/*
 *  ImageKalmanFilter.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 6/1/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "ImageKalmanFilter.h"
#include <opencv2/imgproc/imgproc_c.h>

using namespace cv; 
using namespace std; 

ImageKalmanFilter& ImageKalmanFilter::operator=(const ImageKalmanFilter& rhs){
	copyFrom(rhs); 
	return *this; 
}

ImageKalmanFilter::ImageKalmanFilter(const ImageKalmanFilter &copy){
	copyFrom(copy); 
}

void ImageKalmanFilter::copyFrom(const ImageKalmanFilter &copy) {
	
	if (&copy == this) return; 
	
	if (croppedImage != NULL) cvReleaseImage(&croppedImage); 
	if (croppedVariance != NULL) cvReleaseImage(&croppedVariance); 
	if (mu != NULL) cvReleaseImage(&mu); 
	if (Sigma != NULL) cvReleaseImage(&Sigma); 
	if (sigmaQInv != NULL) cvReleaseImage(&sigmaQInv); 
	if (sigmaQLog != NULL) cvReleaseImage(&sigmaQLog); 
	if (sigmaQLogIntegral != NULL) cvReleaseImage(&sigmaQLogIntegral); 
	//if (rImage != NULL) cvReleaseImage(&rImage); 
	//if (qImage != NULL) cvReleaseImage(&qImage); 
	
	croppedImage = NULL; 
	croppedVariance = NULL; 
	mu = NULL; 
	Sigma = NULL; 
	sigmaQInv = NULL; 
	sigmaQLog = NULL; 
	sigmaQLogIntegral = NULL; 
	//rImage = NULL; 
	//qImage = NULL; 
	if (copy.croppedImage != NULL) croppedImage = cvCloneImage(copy.croppedImage); 
	if (copy.croppedVariance != NULL) croppedVariance = cvCloneImage(copy.croppedVariance); 
	if (copy.mu != NULL) mu = cvCloneImage(copy.mu); 
	if (copy.Sigma != NULL) Sigma = cvCloneImage(copy.Sigma); 
	if (copy.sigmaQInv != NULL) sigmaQInv = cvCloneImage(copy.sigmaQInv); 
	if (copy.sigmaQLog != NULL) sigmaQLog = cvCloneImage(copy.sigmaQLog); 
	if (copy.sigmaQLogIntegral != NULL) sigmaQLogIntegral = cvCloneImage(copy.sigmaQLogIntegral); 
	//if (copy.rImage != NULL) rImage = cvCloneImage(copy.rImage); 
	//if (copy.qImage != NULL) qImage = cvCloneImage(copy.qImage); 
	rImage = copy.rImage; 
	qImage = copy.qImage; 
	
	useRetinalCoordinates = copy.useRetinalCoordinates; 
	padFactor = copy.padFactor; 
	muPrior = copy.muPrior; 
	sigmaSquaredPrior = copy.sigmaSquaredPrior; 
	obsSize = copy.obsSize; 
	numMotionFeatures = copy.numMotionFeatures; 
	numTransformVals = copy.numTransformVals; 
	matNx1a = copy.matNx1a.clone(); 
	sigmaD = copy.sigmaD;
	sigmaDAccumulated = copy.sigmaDAccumulated; 
	q = copy.q; 
}

ImageKalmanFilter::ImageKalmanFilter() {
	init(Size(320,240)); 
}

ImageKalmanFilter::ImageKalmanFilter(Size obsSize) {
	init(obsSize); 
}

void ImageKalmanFilter::init(Size obsSize) {
	croppedImage = NULL; 
	croppedVariance = NULL; 
	mu = NULL; 
	Sigma = NULL; 
	sigmaQInv = NULL; 
	sigmaQLog = NULL; 
	sigmaQLogIntegral = NULL; 
	rImage = NULL; 
	qImage = NULL; 
	
	padFactor = 7.0; 
	muPrior = 0.5; 
	sigmaSquaredPrior = 0.25; 
	
	numTransformVals = 2; 
	
	setObsSize(obsSize); 
	
	setUseRetinalCoordinates(1); 
}

int ImageKalmanFilter::getNumTransformVals() {
	return numTransformVals; 
}

ImageKalmanFilter::~ImageKalmanFilter() {
	
	
	//cvReleaseMat(&matNx1a); 
	cvReleaseImage(&croppedImage); 
	cvReleaseImage(&croppedVariance); 
	cvReleaseImage(&mu); 
	cvReleaseImage(&Sigma); 	
	cvReleaseImage(&sigmaQInv); 
	cvReleaseImage(&sigmaQLog); 
	cvReleaseImage(&sigmaQLogIntegral); 
}

void ImageKalmanFilter::setUseRetinalCoordinates(int yesorno) {
	useRetinalCoordinates = yesorno; 
}

void ImageKalmanFilter::setObsSize(Size s) {
	
	//cvReleaseMat(&matNx1a); 
	matNx1a = Mat(numTransformVals, 1, CV_64F); 
	obsSize = s; 	
	cvReleaseImage(&croppedImage); 
	cvReleaseImage(&croppedVariance); 
	croppedImage = cvCreateImage(obsSize, IPL_DEPTH_64F, 1); 
	croppedVariance = cvCreateImage(obsSize, IPL_DEPTH_64F, 1); 
	setWorldSizePadFactor(padFactor); 
}



void ImageKalmanFilter::setMuPrior(double val) {
	Size s = obsSize; 
	s.width = s.width*padFactor; 
	s.height = s.height*padFactor; 
	
	muPrior = val; 
	
	cvReleaseImage(&mu); 
	mu = cvCreateImage(s, IPL_DEPTH_64F, 1); 
	cvSet(mu, cvRealScalar(muPrior)); 
	/*
	 cvReleaseImage(&muGradX);
	 cvReleaseImage(&muGradY);
	 cvReleaseImage(&muGradSigmaInvX);
	 cvReleaseImage(&muGradSigmaInvY);
	 muGradX = cvCreateImage(s, IPL_DEPTH_64F, 1);  
	 muGradY = cvCreateImage(s, IPL_DEPTH_64F, 1);  
	 muGradSigmaInvX = cvCreateImage(s, IPL_DEPTH_64F, 1);;
	 muGradSigmaInvY = cvCreateImage(s, IPL_DEPTH_64F, 1);  
	 cvSetZero(muGradX);
	 cvSetZero(muGradY);
	 cvSetZero(muGradSigmaInvX);
	 cvSetZero(muGradSigmaInvY);
	 */
	
}


void ImageKalmanFilter::setSigmaSquaredPrior(double val) {
	CvSize s = obsSize; 
	s.width = s.width*padFactor; 
	s.height = s.height*padFactor; 
	
	
	sigmaSquaredPrior = val; 
	
	cvReleaseImage(&Sigma); 
	Sigma = cvCreateImage(s, IPL_DEPTH_64F, 1); 
	cvSet(Sigma, cvRealScalar(sigmaSquaredPrior));
 	sigmaDAccumulated = sigmaSquaredPrior; 
	
	cvReleaseImage(&sigmaQInv); 
	cvReleaseImage(&sigmaQLog); 
	sigmaQInv = cvCreateImage(s, IPL_DEPTH_64F, 1);
	sigmaQLog = cvCreateImage(s, IPL_DEPTH_64F, 1);
	sigmaQLogIntegral = cvCreateImage(cvSize(s.width+1,s.height+1), IPL_DEPTH_64F, 1);
	cvAddS(Sigma, cvRealScalar(q), sigmaQInv); 
	cvLog(sigmaQInv, sigmaQLog);
	cvDiv(NULL, sigmaQInv, sigmaQInv); 
	cvIntegral(sigmaQLog, sigmaQLogIntegral); 
	
	/*
	 cvReleaseImage(&SigmaGradX);
	 cvReleaseImage(&SigmaGradY);
	 cvReleaseImage(&SigmaGradSigmaInvX);
	 cvReleaseImage(&SigmaGradSigmaInvY);
	 cvReleaseImage(&SigmaGradSigmaInvSquaredX);
	 cvReleaseImage(&SigmaGradSigmaInvSquaredY);
	 SigmaGradX = cvCreateImage(s, IPL_DEPTH_64F, 1); 
	 SigmaGradY = cvCreateImage(s, IPL_DEPTH_64F, 1);  
	 SigmaGradSigmaInvX = cvCreateImage(s, IPL_DEPTH_64F, 1);  
	 SigmaGradSigmaInvY = cvCreateImage(s, IPL_DEPTH_64F, 1); 	
	 SigmaGradSigmaInvSquaredX = cvCreateImage(s, IPL_DEPTH_64F, 1); 
	 SigmaGradSigmaInvSquaredY = cvCreateImage(s, IPL_DEPTH_64F, 1);  
	 cvSetZero(SigmaGradX);
	 cvSetZero(SigmaGradY);
	 cvSetZero(SigmaGradSigmaInvX);
	 cvSetZero(SigmaGradSigmaInvY);
	 cvSetZero(SigmaGradSigmaInvSquaredX);
	 cvSetZero(SigmaGradSigmaInvSquaredY);
	 */
}

void ImageKalmanFilter::setWorldSizePadFactor(double val) {
	padFactor = val; 
	
	setMuPrior(muPrior); 
	setSigmaSquaredPrior(sigmaSquaredPrior); 
}


void ImageKalmanFilter::setRSquared(double std) {
	sigmaD = std;
	//sigmaDAccumulated = std; 
}

void ImageKalmanFilter::setQSquared(double std) {
	q = std;
}


void ImageKalmanFilter::setRSquared(IplImage** im) {
	rImage = im; 
}
void ImageKalmanFilter::setQSquared(IplImage** im) {
	qImage = im; 
}

Rect ImageKalmanFilter::getROIForTransformation(const Mat &tau){ 
	int tx = tau.at<double>( 0, 0); 
	int ty = tau.at<double>( 1, 0); 
	int bigw = mu->width; 
	int bigh = mu->height; 
	
	return Rect(bigw/2-obsSize.width/2+tx, bigh/2-obsSize.height/2+ty, 
				  obsSize.width, obsSize.height); 
}

likelihood ImageKalmanFilter::modelLogLikelihoodFast(const Mat &tau, IplImage* seenImage, double scale) {
	if (scale > 1) scale = 1; 
	if (scale <= 0) scale = .1; 
	
	
	if (_IMM_DEBUG) cout << "Getting sensor likelihood." << endl; 
	
	
	Rect roi = getROIForTransformation(tau);
	Size smallSize = Size(roi.width*scale, roi.height*scale); 
	Rect smallRoi = Rect(0, 0, smallSize.width, smallSize.height); 
	
	likelihood retval; 
	retval.grad = Mat::zeros(tau.size(), CV_64F); 
	
	cvSetImageROI(croppedImage, smallRoi); 
	cvSetImageROI(croppedVariance, smallRoi); 
	
	if (_IMM_DEBUG) cout << "croppedImage = seen-predicted" << endl; 
	cvSetImageROI(mu, roi);
	cvResize(mu, croppedImage, CV_INTER_NN); 
	cvResize(seenImage, croppedVariance, CV_INTER_NN); 
	//getForTransform(tau, croppedImage, croppedVariance); 
	cvSub(croppedImage, croppedVariance, croppedImage);
	cvResetImageROI(mu); 	
	
	if (_IMM_DEBUG) cout << "croppedImage = (seen-predicted)^2" << endl; 
	cvMul(croppedImage, croppedImage, croppedImage); 
	
	
	if (_IMM_DEBUG) cout << "croppedImage = (seen-predicted)^2/(sigma^2+q^2)" << endl; 
	//cvDiv(croppedImage, croppedVariance, croppedImage); 
	cvSetImageROI(sigmaQInv, roi); 
	cvResize(sigmaQInv, croppedVariance, CV_INTER_NN); 
	cvMul(croppedImage, croppedVariance, croppedImage); 
	cvResetImageROI(sigmaQInv); 
	
	if (_IMM_DEBUG) cout << "croppedVariance = log(sigma^2+q^2)" << endl; 
	//cvLog(croppedVariance, croppedVariance); 
	
	CvScalar a = cvSum(croppedImage); 
	cvResetImageROI(croppedImage); 
	cvResetImageROI(croppedVariance); 
	
	double logsum = cvGetReal2D(sigmaQLogIntegral, roi.y, roi.x) - 
	cvGetReal2D(sigmaQLogIntegral, roi.y+roi.height, roi.x) - 
	cvGetReal2D(sigmaQLogIntegral, roi.y, roi.x+roi.width) +
	cvGetReal2D(sigmaQLogIntegral, roi.y+roi.height, roi.x+roi.width);
	
	
	retval.val = -.5*a.val[0]-.5*logsum*scale*scale;//b.val[0]; 
	
	return retval; 
	
}

likelihood ImageKalmanFilter::modelLogLikelihood(const Mat &tau, IplImage* seenImage) {
	
	
	if (_IMM_DEBUG) cout << "Getting sensor likelihood." << endl; 
	
	Rect roi = getROIForTransformation(tau);
	
	likelihood retval; 
	retval.grad = Mat::zeros(tau.size(), CV_64F);
	
	if (_IMM_DEBUG) cout << "croppedImage = seen-predicted" << endl; 
	cvSetImageROI(mu, roi);
	//getForTransform(tau, croppedImage, croppedVariance); 
	cvSub(seenImage, mu, croppedImage);
	cvResetImageROI(mu); 
	
	if (_IMM_DEBUG) cout << "croppedImage = (seen-predicted)^2" << endl; 
	cvMul(croppedImage, croppedImage, croppedImage); 
	
	if (_IMM_DEBUG) cout << "croppedImage = (seen-predicted)^2/(sigma^2+q^2)" << endl; 
	//cvDiv(croppedImage, croppedVariance, croppedImage); 
	cvSetImageROI(sigmaQInv, roi); 
	cvMul(croppedImage, sigmaQInv, croppedImage); 
	cvResetImageROI(sigmaQInv); 
	
	
	if (_IMM_DEBUG) cout << "croppedVariance = log(sigma^2+q^2)" << endl; 
	CvScalar a = cvSum(croppedImage); 
	
	double logsum = cvGetReal2D(sigmaQLogIntegral, roi.y, roi.x) - 
	cvGetReal2D(sigmaQLogIntegral, roi.y+roi.height, roi.x) - 
	cvGetReal2D(sigmaQLogIntegral, roi.y, roi.x+roi.width) +
	cvGetReal2D(sigmaQLogIntegral, roi.y+roi.height, roi.x+roi.width);
	
	retval.val = -.5*a.val[0]-.5*logsum;//b.val[0]; 
	
	return retval; 
}

double ImageKalmanFilter::obsLogLikelihood(const Mat &tau, IplImage* seenImage) {
	//return motionModelLogLikelihood(tau, action).val + sensorModelLogLikelihood(tau, seenImage).val; 
	return modelLogLikelihood(tau, seenImage).val; 
}



void ImageKalmanFilter::updateModel(IplImage* seenImage, const Mat &tau, IplImage* sqDiffIm) {	
	if (_IMM_DEBUG) cout << "Updating Sensor Model." << endl; 
	
	Rect roi = getROIForTransformation(tau);
	
	if (rImage == NULL) {
		cvAddS(Sigma, cvRealScalar(sigmaD), Sigma); 
	} else {
		cvAdd(Sigma, *rImage, Sigma); 
		if (_IMM_DEBUG) cout << "Added rImage to Sigma; rImage min/max is " ; 
		double min, max; 
		cvMinMaxLoc(*rImage, &min, &max);; 
		if (_IMM_DEBUG) cout << min << "/" << max << "; Sigma min/max is " ; 
		cvMinMaxLoc(Sigma, &min,&max); 
		if (_IMM_DEBUG) cout << min << "/" << max << endl; 
		
	}
	sigmaDAccumulated = sigmaDAccumulated+sigmaD; 
	
	
	if (useRetinalCoordinates) {
		vector<Rect> rois = getRectanglePatchIntersection(cvSize(mu->width,mu->height),
															cvSize(mu->width,mu->height), 
															cvSize(-1,-1),
															tau.at<double>(0,0),
															tau.at<double>(1,0)); 
		
		if (_IMM_DEBUG) cout << endl << "Updating with tau=[" << tau.at<double>(0,0) << " " << tau.at<double>(1,0) 
			<< "] using (" <<rois[0].x<<" " << rois[0].y<<" " << rois[0].width<<" "<< rois[0].height <<") (" 
			<<rois[1].x<<" " <<rois[1].y<<" " <<rois[1].width<<" "<<rois[1].height<<") "<< endl;
		
		
		recenterMap(mu, muPrior, rois[0], rois[1]); 
		recenterMap(Sigma, sigmaDAccumulated, rois[0], rois[1]); 
		
		matNx1a.setTo(0); 
		
		roi = getROIForTransformation(matNx1a);
	}
	
	if (_IMM_DEBUG) cout << "Updating variance for all pixels." << endl; 
	
	cvResetImageROI(Sigma); 
	
	if (_IMM_DEBUG) cout << "sigmaDAccumulateds are " << sigmaDAccumulated << "<--->" << cvGetReal2D(Sigma, 0, 0) << endl; 
	
	//getForTransform(tau, croppedImage, croppedVariance); 
	
	if (_IMM_DEBUG) cout << "Getting 1/(sigma+q)" << endl; 
	
	// croppedVariance = 1/(sigma+q)
	cvSetImageROI(Sigma, roi); 
	
	if (qImage == NULL) {
		cvAddS(Sigma, cvRealScalar(q), croppedVariance); 
	} else {
		cvSetImageROI(*qImage, roi); 
		cvAdd(Sigma, *qImage, croppedVariance); 
		cvResetImageROI(*qImage); 
	}
	cvDiv(NULL, croppedVariance, croppedVariance); 
	
	if (_IMM_DEBUG) cout << "Getting sigma/(sigma+q)" << endl; 
	// croppedVariance = sigma/(sigma+q)
	cvMul(Sigma, croppedVariance, croppedVariance); 
	
	if (_IMM_DEBUG) cout << "Getting sigma/(sigma+q)*(mu-seenImage)" << endl; 
	//croppedImage = sigma/(sigma+q)(mu-seenImage)
	cvSetImageROI(mu, roi); 
	cvSub(seenImage, mu, croppedImage);
	if (sqDiffIm != NULL) {
		cvMul(croppedImage, croppedImage, sqDiffIm); 
	}
	cvMul(croppedImage, croppedVariance, croppedImage); 
	
	if (_IMM_DEBUG) cout << "Getting mu+sigma/(sigma+q)(mu-seenImage)" << endl; 
	//mu = mu+sigma/(sigma+q)(mu-seenImage)
	cvAdd(croppedImage, mu, mu); 
	
	if (_IMM_DEBUG) cout << "Getting (1-sigma/(sigma+q))" << endl; 
	//croppedVariance = (1-sigma/(sigma+q))
	cvSubRS(croppedVariance, cvRealScalar(1), croppedVariance); 
	
	if (_IMM_DEBUG) cout << "Getting (1-sigma/(sigma+q))*sigma" << endl; 
	//sigma = (1-sigma/(sigma+q))*sigma
	cvMul(croppedVariance, Sigma, Sigma); 
	
	cvResetImageROI(mu); 
	cvResetImageROI(Sigma); 	
	
	if (qImage == NULL) {
		cvAddS(Sigma, cvRealScalar(q), sigmaQInv); 
	} else {
		cvAdd(Sigma, *qImage, sigmaQInv); 
	}
	
	cvLog(sigmaQInv, sigmaQLog);
	cvDiv(NULL, sigmaQInv, sigmaQInv); 	
	cvIntegral(sigmaQLog, sigmaQLogIntegral); 
}

void ImageKalmanFilter::recenterMap(IplImage* image, double defaultVal, Rect roi0, Rect roi1) {
	
	IplImage* clone = cvCloneImage(image); 
	cvSet(image,cvRealScalar(defaultVal)); 
	cvSetImageROI(image,roi0); 
	cvSetImageROI(clone,roi1); 
	cvCopy(clone,image); 
	cvResetImageROI(image); 
	cvReleaseImage(&clone); 
}



//Computes intersection regions of two rectangles in their own coordinate systems.
//r1 and r2 are rectangles of size r1size and r2size. PatchSize is a patch, which 
//may be smaller than either size of a subrectangle, centered in r1 and r2. If
//patchSize is (-1,-1), then the smaller of r1size/r2size is used. xoff is the
//the x position of r2 relative to r1. yoff is the y position of r2 relative to
// r1. Afterward, retval[0] is the region of interest (roi) in r1 that intersects
// the patch in r2, and retval[1] is the roi in r2 that intersects the patch in
// r1. 
vector<Rect> ImageKalmanFilter::getRectanglePatchIntersection(Size r1size, Size r2size, Size patchSize, int xoff, int yoff) {	
	
	if (patchSize.width < 0 || patchSize.width > r1size.width ||
		patchSize.height < 0 || patchSize.height > r1size.height) 
		patchSize = r1size; 
	if (patchSize.width > r2size.width || patchSize.height > r2size.height)
		patchSize = r2size; 
	
	if (_IMM_DEBUG) cout << "xoff: " << xoff << "; yoff: " << yoff << endl; 
	
	double tlx1, tlx2, tly1, tly2, brx1, brx2, bry1,bry2; 
	tlx1 = r1size.width *1.0 / 2.0 - patchSize.width*1.0/2.0;
	tly1 = r1size.height *1.0 / 2.0 - patchSize.height*1.0/2.0;
	brx1 = r1size.width *1.0 / 2.0 + patchSize.width*1.0/2.0 -1;
	bry1 = r1size.height *1.0 / 2.0 + patchSize.height*1.0/2.0-1;
	
	tlx2 = r2size.width *1.0 / 2.0 - patchSize.width*1.0/2.0 +xoff;
	tly2 = r2size.height *1.0 / 2.0 - patchSize.height*1.0/2.0+yoff;
	brx2 = r2size.width *1.0 / 2.0 + patchSize.width*1.0/2.0+xoff-1;
	bry2 = r2size.height *1.0 / 2.0 + patchSize.height*1.0/2.0+yoff-1;
	
	if (_IMM_DEBUG) cout << "BB1: (" << tlx1 << ", " << tly1 << ") -> ("
		<< brx1 << ", " <<bry1 << ")" << endl; 
	if (_IMM_DEBUG) cout << "BB2: (" << tlx2 << ", " << tly2 << ") -> ("
		<< brx2 << ", " <<bry2 << ")" << endl; 
	
	if (tlx2 < 0) {
		double diff = 0-tlx2; 
		tlx2 = tlx2+diff; 
		tlx1 = tlx1+diff; 
	}
	if (tly2 < 0) {
		double diff = 0-tly2; 
		tly2 = tly2+diff; 
		tly1 = tly1+diff; 
	}
	if (tlx1 < 0) {
		double diff = 0-tlx1; 
		tlx2 = tlx2+diff; 
		tlx1 = tlx1+diff; 
	}
	if (tly1 < 0) {
		double diff = 0-tly1; 
		tly2 = tly2+diff; 
		tly1 = tly1+diff; 
	}
	if (brx2 > r2size.width-1) {
		double diff = r2size.width-1-brx2; 
		brx2 = brx2+diff; 
		brx1 = brx1+diff; 
	}
	if (bry2 > r2size.height-1) {
		double diff = r2size.height-1-bry2; 
		bry2 = bry2+diff; 
		bry1 = bry1+diff; 
	}
	if (brx1 > r1size.width-1) {
		double diff = r1size.width-1-brx1; 
		brx2 = brx2+diff; 
		brx1 = brx1+diff; 
	}
	if (bry1 > r1size.height-1) {
		double diff = r1size.height-1-bry1; 
		bry2 = bry2+diff; 
		bry1 = bry1+diff; 
	}
	
	if (_IMM_DEBUG) cout << "BB1: (" << tlx1 << ", " << tly1 << ") -> ("
		<< brx1 << ", " <<bry1 << ")" << endl; 
	if (_IMM_DEBUG) cout << "BB2: (" << tlx2 << ", " << tly2 << ") -> ("
		<< brx2 << ", " <<bry2 << ")" << endl; 
	
	Rect roi1 = Rect((int)tlx1, (int)tly1, (int)brx1-(int)tlx1+1, (int)bry1-(int)tly1+1); 
	Rect roi2 = Rect((int)tlx2, (int)tly2, (int)brx2-(int)tlx2+1, (int)bry2-(int)tly2+1); 
	
	vector<Rect> retval; 
	retval.push_back(roi1); 
	retval.push_back(roi2); 
	return retval; 
	
}

int ImageKalmanFilter::getUsesExternalREstimate() const{
	return rImage != NULL; 
}

int ImageKalmanFilter::getUsesExternalQEstimate() const{
	return qImage != NULL; 
}

Size ImageKalmanFilter::getObsSize() {
	return obsSize; 
}

void ImageKalmanFilter::addToStream(ostream& out) const {
	if (_IMM_DEBUG) cout << "Adding IKF Object To Stream" << endl; 
	out << obsSize.width << " " << obsSize.height << endl; 
	out << padFactor << " " << muPrior << " " << sigmaSquaredPrior << " " << useRetinalCoordinates << endl ;
}

void ImageKalmanFilter::readFromStream(istream& in) {
	if (_IMM_DEBUG) cout << "Reading IKF Object from Stream" << endl; 
	in >> obsSize.width >> obsSize.height; 
	in >> padFactor >> muPrior >> sigmaSquaredPrior >> useRetinalCoordinates; 
	
	if (_IMM_DEBUG) cout << "Calling setObsSize" << endl; 
	setObsSize(obsSize); 	
}



ostream& operator<< (ostream& ofs, const ImageKalmanFilter &model) {
	model.addToStream(ofs); 
	return ofs; 
}
istream& operator>> (istream& ifs, ImageKalmanFilter &model) {
	model.readFromStream(ifs); 
	return ifs; 
}
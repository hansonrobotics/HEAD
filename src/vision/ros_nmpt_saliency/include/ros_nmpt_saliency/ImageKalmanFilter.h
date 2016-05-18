/*
 *  ImageKalmanFilter.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 6/1/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef _IMAGEKALMANFILTER_H
#define _IMAGEKALMANFILTER_H

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <iostream>
#include <vector>

#include "DebugGlobals.h"
#include "StructTypes.h"


/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A Collection of Kalman Filters for 
 * estimating the appearance of an image. 
 *
 * The Kalman Filter gives a Mean estimate of the values of the image's pixels,
 * and maintains a variance Sigma for each pixel.
 *
 * Generally, only part of the image is seen at once. This region of the image
 * will have its means updated, and its variances will shrink. All other 
 * regions will have their means held constant while their variances increase.
 *
 * There are many parameters to a Kalman Filter that may be set:
 *
 * It is useful to think of there being a stochastic relation between x, the true
 * pixel value at a location, and y, the observed pixel value.
 * Specifically y is drawn from a normal distribution with mean x and Variance
 * Q. 
 *
 * It is also useful to think of there being a drift in x, i.e. the image pixel 
 * values may change over time. This drift is 0 mean, and has covariance R. 
 * If R is 0, you are making a strong assertion that the environment will never 
 * change, that nothing moves.
 *
 * Finally, you may set a prior mean estimate Mu of x, and a prior uncertainty,
 * Sigma. 
 *
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */
class ImageKalmanFilter {	
	public:
	
    /**
	 * \brief Assignment Operator. Perform a deep copy of another ImageKalmanFilter.
	 **/
	ImageKalmanFilter& operator=(const ImageKalmanFilter& rhs);
	
	
    /**
	 * \brief Copy Constructor. Perform a deep copy of another ImageKalmanFilter.
	 **/
	ImageKalmanFilter(const ImageKalmanFilter &copy); 
	
	
	/**
	 * \brief Default Constructor. Equivalent to calling the main constructor with 
	 * an observation size of 340x240.
	 **/
	ImageKalmanFilter(); 
	
	/**
	 * \brief Main Constructor. Initialize with the size of the image that will
	 * be observed at each timestep. 
	 *
	 * This is the size of the camera input. By default, ImageKalmanFilter assumes
	 * that the total seeable world is less than 7 times the width and height of 
	 * this image. For example, if the observation size that you get from your
	 * camera is 320x240, the size of the ImageKalmanFilter will be 2240x1680. You
	 * can change this after the constructor call by calling setWorldSizePadFactor
	 * to something other than 7.0.
	 **/
	ImageKalmanFilter(cv::Size obsSize); 
	
	/**
	 * \brief Destructor.
	 **/
	virtual ~ImageKalmanFilter(); 
	
	/**
	 * \brief Tell the ImageKalmanFilter what size observations to expect. This
	 * allows you to change the underlying memory structrue of the ImageKalmanFilter
	 * without creating a new object. 
	 *
	 * Note that this will essentially reconstruct the object, reallocating and
	 * reinitializing all data structure memory. The padSizeFactor (size of the
	 * world in relation to the observation size) will be used, so the total
	 * number of pixels in the ImageKalmanFilter will change. 
	 **/
	void setObsSize(cv::Size s); 
	
	/**
	 * \brief Query the size of Observation that this ImageKalmanFilter expects.
	 **/ 
	cv::Size getObsSize(); 
	
	/**
	 * \brief Set the size of the ImageKalmanFilter in terms of the width and height
	 * of the observations. 
	 * 
	 * Note that this will essentially reconstruct the object, 
	 * reallocating and reinitializing all data structure memory. The current obsSize 
	 * (size of the images coming from a camera) will be used, so the total
	 * number of pixels in the ImageKalmanFilter will change. 
	 **/
	void setWorldSizePadFactor(double val=7.0); 
	
	/**
	 * \brief Sets the coordinate frame to World Coordinates (0) or Retinal Coordinates
	 * (Non-0). By default, retinal coordinates are used.
	 *
	 * In retinal coordinates, the center of the ImageKalmanFilter is always
	 * colocated with the center of the most recent observation. Actions are interpreted
	 * as being relevant to this central location. 
	 *
	 * In world coordinates, actions are interpreted as being relative to the servo
	 * reference frame rather than the camera reference frame. 
	 *
	 * The InternalMotionModel is much more robust in a retinal coordinate frame than 
	 * in a servo coordinate frame. It is not recommended to set this to 0.
	 **/
	void setUseRetinalCoordinates(int yesorno = 1); 
	
	
	/**
	 * \brief Set the mean of the Image intensity estimates to all be a specified value.
	 *
	 * The default value of 0.5 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. 
	 **/
	void setMuPrior(double val=0.5); 	
	
	/**
	 * \brief Set the variance of the Image intensity estimates to all be a specified value.
	 *
	 * The default value of 0.25 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. Note that 0.25 variance means standard deviation of 0.5.
	 **/
	void setSigmaSquaredPrior(double val = 0.25); 
	
	/**
	 * \brief Set the pixel drift variance (how much do pixels change their value over time?).
	 *
	 * The default value of 0.0001 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. Note that 0.0001 variance means standard deviation of 0.01.
	 * This is relatively low, and asserts that the appearance of the world will be mostly
	 * static. 
	 **/
	void setRSquared(double std = 0.0001); 
	
	/**
	 * \brief Set the camera noise variance (how much do you trust an unreliable sensor?).
	 *
	 * The default value of 0.01 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. Note that 0.01 variance means standard deviation of 0.1.
	 * This is relatively high, and asserts that the current field of view is an unreliable
	 * representation of the world's appearance. This helps filter out abberations caused by
	 * rapid motion through the visual field. 
	 **/
	void setQSquared(double std = 0.01); 	
	
	
	/**
	 * \brief Set the pixel drift variance on a pixel-by-pixel basis. If you happen to know
	 * (or learn) that certain regions of the world have more motion than others, this is
	 * useful. 
	 *
	 * Learning the statistics of motion in the world is an active area of research, and 
	 * this feature is not well supported. 
	 *
	 * You are responsible for managing the memory of the underlying image. Setting *im to
	 * NULL will cause the program to revert to uniform r-values across the image plane. 
	 **/
	void setRSquared(IplImage** im); 
	
	/**
	 * \brief Set the sensor reliability on a pixel-by-pixel basis. If you happen to know
	 * (or learn) that certain regions of the lens have higher distortion than others,
	 * this may be useful.
	 * 
	 * Learning the statistics of one's own sensor distortions is an active area of research, and 
	 * this feature is not well supported. 
	 *
	 * You are responsible for managing the memory of the underlying image. Setting *im to
	 * NULL will cause the program to revert to uniform r-values across the image plane. 
	 **/	
	void setQSquared(IplImage** im); 	
		
	/**
	 * \brief Checks whether an external image of pixel-drifts has been supplied using 
	 * setRSquared(IplImage**). If not (or if NULL was set), this returns 0, 1 otherwise.
	 **/	
	int getUsesExternalREstimate() const; 
	
	/**
	 * \brief Checks whether an external image of sensor-noise has been supplied using 
	 * setQSquared(IplImage**). If not (or if NULL was set), this returns 0, 1 otherwise.
	 **/	
	int getUsesExternalQEstimate() const;	
	
	/**
	 * \brief How large is the space of image transformations potentially induced by motor
	 * commands? Right now this is 2 (translation, x and y), but in the future the 
	 * transform space may increase to include rotation, scale, etc., in which case this
	 * will return more. 
	 **/
	int getNumTransformVals(); 
	
	/**
	 * \brief Update the ImageKalmanFilter by adding a new seenImage at a specified transformation,
	 * tau. tau must be a vector of size [getNumTransformVals() x 1]. 
	 *
	 * Optionally, this function can output the an image of the same size as seenImage representing
	 * the squared difference between what the ImageKalmanFilter expected to see based on 
	 * the transform tau (memory) and what was actually seen (sensation). This may be useful in
	 * learning about the reliability of your sensors, and the spatial layout of systematic
	 * distortions. 
	 **/
	void updateModel(IplImage* seenImage, const cv::Mat &tau, IplImage* sqDiffIm = NULL); 
	
	
	/**
	 * \brief Compute the log-likelihood value of an observation given a particular transform
	 * (translation, x&y, in units of pixels). 
	 **/
	double obsLogLikelihood(const cv::Mat &tau, IplImage* seenImage);  
	
	/**
	 * \brief Compute the log-likelihood value of an observation given a particular transform
	 * (translation, x&y, in units of pixels). 
	 *
	 * The gradients of the likelihood function with respect to tau are not currently computed.
	 **/
	likelihood modelLogLikelihood(const cv::Mat &tau, IplImage* seenImage); 
	
	/**
	 * \brief Compute the approximate log-likelihood value of an observation given a particular 
	 * transform (translation, x&y, in units of pixels). 
	 *
	 * This is done by a subsampling method,
	 * only sampling and comparing every [scale]th pixel of the current ImageKalmanFilter estimate
	 * (memory) with those of the seen image (sensation). 
	 *
	 * The gradients of the likelihood function with respect to tau are not currently computed.
	 **/
	likelihood modelLogLikelihoodFast(const cv::Mat &tau, IplImage* seenImage, double scale) ; 
	
	/**
	 * \brief The mean of the current pixel-wise estimate of the image intensity values. 
	 * This is the actual underlying estimate data. It is exposed for convenient inspection.
     * Don't modify it.
	 */ 
	IplImage *mu; 
	
	/**
	 * \brief The variance of the current pixel-wise estimate of the image intensity values. 
	 * This is the actual underlying estimate data. It is exposed for convenient inspection.
     * Don't modify it.
	 */ 
	IplImage *Sigma; 
	
	
	/**
	 * \brief Write to a file.
	 */
	friend std::ostream& operator<< (std::ostream& ofs, const ImageKalmanFilter &feat); 
	
	/**
	 * \brief Read from a file.
	 */
	friend std::istream& operator>> (std::istream& ifs, ImageKalmanFilter &feat); 
	
	
private:
	
	void init(cv::Size obsSize); 
	
	void copyFrom(const ImageKalmanFilter &copy); 
	
    std::vector<cv::Rect> getRectanglePatchIntersection(cv::Size r1size, cv::Size r2size, cv::Size patchSize, int xoff, int yoff); 
	cv::Rect getROIForTransformation(const cv::Mat &tau); 
	
	IplImage *sigmaQInv;
	IplImage *sigmaQLog;
	IplImage *sigmaQLogIntegral;
	
	IplImage **rImage; 
	IplImage **qImage; 
	
	int useRetinalCoordinates; 
	
	double padFactor; 
	double muPrior; 
	double sigmaSquaredPrior; 
	
	/* Parameters from the robot */
	cv::Size obsSize; 
	
	/* Internal parameters */
	int numMotionFeatures; // m
	int numTransformVals;  // n
	
	/* Cached image memory for efficiency */
	IplImage* croppedImage; 
	IplImage* croppedVariance; 
	
	cv::Mat matNx1a; 
	
	double sigmaD;
	double sigmaDAccumulated; 
	double q; 
	
	
	void recenterMap(IplImage* image, double defaultVal, cv::Rect roi0, cv::Rect roi1);
	
	
	virtual void readFromStream(std::istream& in); 
	virtual void addToStream(std::ostream& out) const; 
	
	
};

std::ostream& operator<< (std::ostream& ofs, const ImageKalmanFilter &model); 
std::istream& operator>> (std::istream& ifs, ImageKalmanFilter &model) ;

#endif
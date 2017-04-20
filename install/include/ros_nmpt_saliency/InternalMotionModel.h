/*
 *  InternalMotionModel.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 2/10/10.
 *  Copyright 2010 Apple Inc. All rights reserved.
 *
 */

#ifndef _INTERNALMOTIONMODEL_H
#define _INTERNALMOTIONMODEL_H

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>

#include <iostream>
#include <vector>

#include "DebugGlobals.h"
#include "StructTypes.h"
#include "MatrixKalmanFilter.h"
#include "ImageKalmanFilter.h"


/**
 * \ingroup MPGroup
 * \brief <tt> <b> Machine Perception Primitive: </b> </tt> An implementation of
 * a model that allows a robot to learn about its own motion. 
 *
 * This code is an extension of the approach first described in Butko and Movellan,
 * "Learning to Look," 2010 (see \ref bib_sec).
 *
 * The default parameters were found to work well for three different robots. Hopefully
 * they will work for your robot too. 
 *
 * For learning, it is sufficient to call "updateModelMAP" with the robot's current 
 * field of view (a 320x240 floating point image) and previous action. After learning,
 * appropriate motor commands can be found using "recommendActionForDesiredTransform".
 *
 * Actions should be represented using
 * a relative coordinate system such that 0 means "no change," and the sign indicates 
 * the direction of motion. The default parameters were chosen in a situation where
 * an action of 1 represented the largest possible movement on a given motor (e.g.
 * from the left-most end of the motor's range to the right-most). So, an action of 
 * 1 would always move the motor to the right-most extreme, regardless of where you started,
 * -1 would always move the motor to the left-most extreme. A value of .1 would move the 
 * motor 1/10th of the total range of motion. Sending an action of -1, +1 would result
 * in the same end point as calling -1, +1/3, +1/3, +1/3. This endpoint will be the same
 * regardless of starting point because the intial -1 will always saturate the motor's
 * range, pushing it to the extreme. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class InternalMotionModel {
public:
	
	/**
	 * \brief Assignment Operator. Perform a deep copy of another InternalMotionModel.
	 **/
	InternalMotionModel& operator=(const InternalMotionModel& rhs);
	
	/**
	 * \brief Copy Constructor. Perform a deep copy of another InternalMotionModel.
	 **/
	InternalMotionModel(const InternalMotionModel &copy); 
	
	/**
	 * \brief Default Constructor. Equivalent to calling the main constructor with 2 servos, 
	 * an observation size of (320,240), and 30 dynamics models. 
	 **/
	InternalMotionModel(); 
	
	/**
	 * \brief Main Constructor.
	 * 
	 * The internal motion model takes three arguments: numActuators (degrees of 
	 * freedom that affect camera motion),  obsSize (size of the image grabbed by the
	 * camera), and numDynamicsModels. 
	 * 
	 * The dynamics models are important for high speed tracking, but they are not
	 * necessary for the basic behavior of the system to work. The dynamics model
	 * is trained by having separate time-points at frame-rate intervals after an 
	 * eye movement. For example, if the camera is 30FPS, then the learning protocal
	 * would be to issue an eye movement, collect the next 30 frames, and then call 
	 * updateModelMAP(image[i], action, i) for each of the 30 images in turn. 
	 * 
	 * Calling updateModelMAP(image, action) will only update the latest timepoint
	 * model, which is the one used by recommendActionForDesiredTransform() without
	 * an action history.
	 **/
	InternalMotionModel(int numActuators, cv::Size obsSize=cv::Size(320,240), size_t numDynamicsModels=30); 
	
	/**
	 * \brief Destructor.
	 **/
	virtual ~InternalMotionModel(); 
	
	/**
	 * \brief Resets the number of actuators that we are learning a motion model over. 
	 * This resets the model to the prior model. 
	 **/
	void setNumActuators(int num); 	
	
	/**
	 * \brief Tell the InternalMotionModel what size observations to expect. This
	 * resets any knowledge of the appearance of the world. 
	 **/
	void setObsSize(cv::Size s); 
	
	
	/**
	 * \brief Set the size of the world appearance estimate in terms of the width and height
	 * of the observations. This	 * resets any knowledge of the appearance of the world. 
	 **/
	void setWorldSizePadFactor(double val=7.0); 
	
	/**
	 * \brief Set the number of time points used for estimating the Dynamics Model. 
	 * This resets the estimate of the motion parameters.
	 * 
	 * The dynamics models are important for high speed tracking, but they are not
	 * necessary for the basic behavior of the system to work. The dynamics model
	 * is trained by having separate time-points at frame-rate intervals after an 
	 * eye movement. For example, if the camera is 30FPS, then the learning protocal
	 * would be to issue an eye movement, collect the next 30 frames, and then call 
	 * updateModelMAP(image[i], action, i) for each of the 30 images in turn. 
	 **/
	void setNumDynamicsModels(size_t num) ;
	
	/**
	 * \brief Get the number of time points used for estimating the Dynamics Model. 
	 **/
	size_t getNumDynamicsModels() const; 
	
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
	 * \brief EXPERIMENTAL: Use computed motor reliabilities when updating the
	 * motion model using updateModel() or updateModelMAP(). You probably should not use this. 
	 **/ 
	void setUseMotorUncertaintyInLearning(int yesorno = 0) ;
	
	/**
	 * \brief EXPERIMENTAL: Use computed sensor reliabilities when updating the
	 * visual model using updateModel() or updateModelMAP(). You probably should not use this. 
	 **/ 
	void setUseSensorUncertaintyInLearning(int yesorno = 0);
	
	/**
	 * \brief EXPERIMENTAL: Use a directly learned inverse model for computing
	 * recommendActionForDesiredTransform(). You probably should not use this. 
	 * 
	 * It can be enabled using setUseLearnedInverseModel(1), but it should not be. It is
	 * unreliable when there are more motors than translation dimensions, and the 
	 * computed inverse method used by default is much better at taking the reliability of
	 * the individaul motors into account. 
	 **/ 
	void setUseLearnedInverseModel(int yesorno = 0); 
	
	/**
	 * \brief Set the mean of Lambda intensity estimates to all be a specified value.
	 * Lambda is the estimate of the world appearance.
	 *
	 * The default value of 0.5 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. 
	 **/
	void setMuLambdaPrior(double val=0.5); 
	
	/**
	 * \brief Set the variance of the Image intensity estimates to all be a specified value.
	 * Lambda is the estimate of the world appearance.
	 *
	 * The default value of 0.25 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. Note that 0.25 variance means standard deviation of 0.5.
	 **/
	void setSigmaSquaredLambdaPrior(double val = 0.25); 
	
	/**
	 * \brief Set the pixel drift variance (how much do pixels change their value over time?).
	 * Lambda is the estimate of the world appearance.
	 *
	 * The default value of 0.0001 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. Note that 0.0025 variance means standard deviation of 0.05.
	 * This means the pixel values from frame to frame are expected to change by about 5% (rarely
	 * more than 15%). 
	 **/
	void setRSquaredLambda(double std = 0.0025); 
	
	/**
	 * \brief Set the camera noise variance (how much do you trust an unreliable sensor?).
	 * Lambda is the estimate of the world appearance.
	 *
	 * The default value of 0.01 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. Note that 0.01 variance means standard deviation of 0.1.
	 * This is relatively high, and asserts that the current field of view is an unreliable
	 * representation of the world's appearance. This helps filter out abberations caused by
	 * rapid motion through the visual field. 
	 **/
	void setQSquaredLambda(double std = 0.01); 	
	
	
	/**
	 * \brief Set the mean of Lambda intensity estimates to all be a specified value.
	 * Eta is an estimate of which areas of the world change quickly or slowly (have
	 * high motion).
	 *
	 * The default value of 0.5 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. 
	 **/
	void setMuEtaPrior(double val=0.5); 
	
	/**
	 * \brief Set the variance of the Image intensity estimates to all be a specified value.
	 * Eta is an estimate of which areas of the world change quickly or slowly (have
	 * high motion).
	 *
	 * The default value of 0.25 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. Note that 0.25 variance means standard deviation of 0.5.
	 **/
	void setSigmaSquaredEtaPrior(double val = 0.25); 
	
	/**
	 * \brief Set the pixel drift variance (how much do pixels change their value over time?).
	 * Eta is an estimate of which areas of the world change quickly or slowly (have
	 * high motion).
	 *
	 * The default value of 0.0001 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. Note that 0.0001 variance means standard deviation of 0.01.
	 * This is relatively low, and asserts that the appearance of the world will be mostly
	 * static. 
	 **/
	void setRSquaredEta(double std = 0.0001); 
	
	/**
	 * \brief Set the camera noise variance (how much do you trust an unreliable sensor?).
	 * Eta is an estimate of which areas of the world change quickly or slowly (have
	 * high motion).
	 *
	 * The default value of 0.01 reflects the assumption that pixel intensities are between
	 * 0 and 1. If your images uses a different range of (floating point) values, you may
	 * want to set a different mean. Note that 0.01 variance means standard deviation of 0.1.
	 * This is relatively high, and asserts that the current field of view is an unreliable
	 * representation of the world's appearance. This helps filter out abberations caused by
	 * rapid motion through the visual field. 
	 **/
	void setQSquaredEta(double std = 0.01); 	
	
    /**
     * \brief Set the mean of the motion model parameters Alpha to have each element with identical value.
     */
	void setMuAlphaPrior(double vectorVal = 0.0); 
	
    /**
     * \brief Set the uncertainty in the motion model parameters Alpha 
	 * to a diagonal matrix with identical values along the diagonal.
     */
	void setSigmaAlphaPrior(double diagval = 250000.0); 
	
    /**
     * \brief Set R, the drift in the estimate of the motion model parameters Alpha 
	 * to a diagonal matrix with identical values along the diagonal.
     */
	void setRAlpha(double diagval = 25.0);
	
    /**
     * \brief Set Q, the noise in observations of the transforms tau (x,y translation)
	 * to a diagonal matrix with identical values along the diagonal.
     */
	void setQAlpha(double diagval = 400.0); 
	
	
    /**
     * \brief Set the mean of the motor noise model parameters Nu to have each element with identical value.
     */
	void setMuNuPrior(double vectorVal = 20); 
	
    /**
     * \brief Set the uncertainty in the motor noise model parameters Nu 
	 * to a diagonal matrix with identical values along the diagonal.
     */
	void setSigmaNuPrior(double diagval = 250000.0); 
	
    
    /**
     * \brief Set R, the drift in the estimate of the motor noise model parameters Nu,
	 * to a diagonal matrix with identical values along the diagonal.
     */
	void setRNu(double diagval = 25.0);
	
    /**
     * \brief Set Q, the error in the estimate of the transforms tau (x,y translation), 
	 * to a diagonal matrix with identical values along the diagonal.
     */
	void setQNu(double diagval = 400.0); 
	
	/**
	 * \brief Make a decision about how to look at something. This command is 
	 * sufficient for low-speed tracking, in which each decision about where
	 * to move next is made after the previous eye-movement has completed, on
	 * the order of 1s. For* high-speed trackinging, where decsions are made 
	 * faster than about 300ms, it is important to use the other 
	 * recommendActionForDesiredTransform which takes an actionHistory and 
	 * decisionTimeStamp as arguments. 
	 * 
	 * 
	 * The desired transform
	 * (tau) is a desired pixel translation. When using retinal coordinates, the 
	 * translation 0, 0 is the central pixel of the current visual field. 
	 *
	 * The 
	 * result is passed by reference back through the parameter action. This is 
	 * the command you should send to the robot servos. 
	 *
	 * The units of tau are pixel-based, with respect to the original image size used
	 * for training the motion model. For example, if the model was trained with images
	 * of size 320x240, and you want to look at the top left of the visual field, then
	 * tau should be (160,-120). Note that you are responsible for making this be the
	 * case. It's common to have the runtime visual field be higher resolution than 
	 * that used in training, for example 640x480. In this case, you would want
	 * to scale your pixel units by a factor of two. To know what your particular
	 * scaling factor should be, call "getEyeMovementUnitsSize()" to find the size of
	 * image that the InternalMotionModel was originally trained with, and scale
	 * your transforms accordingly before calling recommendActionForDesiredTransform.
	 **/
	void recommendActionForDesiredTransform(const cv::Mat &tau, cv::Mat &action); 
	
	/**
	 * \brief Make a decision about how to look at something, taking the dynamics model
	 * and a history of eye-movements into account. Only use this command if you have
	 * trained the InternalMotionModel's dynamics model, and are making decisions about
	 * where to move the eyes on a sub-second timescale. 
	 * 
	 * The desired transform
	 * (tau) is a desired pixel translation. When using retinal coordinates, the 
	 * translation 0, 0 is the central pixel of the current visual field. 
	 * 
	 * To correct
	 * for system dynamics, actionRecord is a history of the eye-movements you have
	 * sent, and the times you sent them. 
	 *
	 * decisionTimeStamp is a record of when the
	 * visual information used to make the current saccade was generated. For example,
	 * if you are tracking a face with a face-finder, decsionTimeStamp should be a timeStamp
	 * at which the image was acquired, before the face finder started running. This is because
	 * by the time the face finder finishes running, the robot's eyes are likely to be in a 
	 * different place than when the image was acquired, and the dynamics model needs to
	 * correct for this. 
	 *
	 * The result is passed by reference back through the parameter action. This is 
	 * the command you should send to the robot servos. 
	 *
	 * The units of tau are pixel-based, with respect to the original image size used
	 * for training the motion model. For example, if the model was trained with images
	 * of size 320x240, and you want to look at the top left of the visual field, then
	 * tau should be (160,-120). Note that you are responsible for making this be the
	 * case. It's common to have the runtime visual field be higher resolution than 
	 * that used in training, for example 640x480. In this case, you would want
	 * to scale your pixel units by a factor of two. To know what your particular
	 * scaling factor should be, call "getEyeMovementUnitsSize()" to find the size of
	 * image that the InternalMotionModel was originally trained with, and scale
	 * your transforms accordingly before calling recommendActionForDesiredTransform.
	 **/
	void recommendActionForDesiredTransform(const cv::Mat &tau, const std::vector<actionRecord> &history, double decisionTimeStamp, cv::Mat &action); 
	
	/**
	 * \brief Get the units that recommendActionForDesiredTransform is expecting. 
	 * 
	 * This returns the observation size the this InternalMotionModel was trained with,
	 * which is needed to request a "desiredTransform". The units of the desired
	 * transform are -getEyeMovementSize().width/2:getEyeMovementSize().width/2 and
	 * the analog for height. It is possible to request an eye-movement outside this range,
	 * which is simply equivalent to asking the robot to turn to look at something 
	 * outside its current field of view. 
	 **/ 
	cv::Size getEyeMovementUnitsSize(); 
	
	/**
	 * \brief This is the main training function. It finds the most likely offset 
	 * of the seenImage in the representation of remembered visual history, and uses
	 * that, along with the action taken, to train the Motion Model. Optionally,
	 * frameNum and frameTime can be supplied to train the DynamicsModel. 
	 * 
	 * The process of updating the model has the following steps: 
	 * 
	 * \li cv::Mat MAPTransform; 
	 * \li findMaxLikelihoodTransformFaster(seenImage, action, MAPTransform, frameNum);
	 * \li updateModel(seenImage, action, MAPTransform, frameNum); 
	 * \li if (frameNum > -1) updateFrameTime(frameNum, frameTime); 
	 *
	 * You can call these functions instead yourself, if you want to inspect the
	 * process. Alternatively, you can call findMaxLikelihoodTransform(), which
	 * is slower but more accurate than findMaxLikelihoodTransformFaster(). 
	 **/
	void updateModelMAP(IplImage* seenImage, const cv::Mat &action, int frameNum=-1, double frameTime=0); 
	
	/**
	 * \brief Find the mean transform (x,y translation) predicted by the current motion model
	 * for a given action. The result is passed back through the tauMat reference.
	 **/ 
	void getTauMeanForAction(const cv::Mat &action, cv::Mat &tauMat, int frameNum=-1); 
	
	/**
	 * \brief Find the log likelihood of a particular transform tau (x,y translation) 
	 * give an seenImage from the camera and action, the last taken eye-movement. 
	 **/ 
	double obsLogLikelihood(const cv::Mat &tau, IplImage* seenImage, const cv::Mat &action, int frameNum=-1); 
	
	/**
	 * \brief Find the transform (x,y translation) that maximizes obsLogLikelihood.  
	 * The result is passed back through the reference to the matrix tau. 
	 **/ 
	void findMaxLikelihoodTransform(IplImage* seenImage, const cv::Mat &action, cv::Mat &tau, int frameNum=-1); 
	
	/**
	 * \brief Find the transform (x,y translation) that approximately maximizes obsLogLikelihood.  
	 * The result is passed back through the reference to the matrix tau. This function is much
	 * faster (at a negligible cost to accuracy) than findMaxLikelihoodTransform(). 
	 **/ 
	void findMaxLikelihoodTransformFaster(IplImage* seenImage, const cv::Mat &action, cv::Mat &tau, int frameNum=-1);	
	
	/**
	 * \brief Assert that tau is the correct transform (x,y translation) induced by the
	 * last taken eyemovement (action). This updates the appearance model of the world
	 * with the seenImage, and the model of the robot's motion with action. 
	 **/ 
	void updateModel(IplImage* seenImage, const cv::Mat &action, const cv::Mat &tau, int frameNum=-1); 
	
	/**
	 * \brief Find the transform (x,y translation) that approximately maximizes obsLogLikelihood.  
	 * The result is passed back through the reference to the matrix tau. This function is much
	 * faster (at a negligible cost to accuracy) than findMaxLikelihoodTransform(). 
	 **/ 
	void updateFrameTime(int frameNum, double frameTime, double timeConst = 0.1); 
	
	/**
	 * \brief The estimate of the appearance of the world around the robot. This is the
	 * actual underlying estimate data. It is exposed for convenient inspection.
	 * Don't modify it.
	 */
	ImageKalmanFilter lambda; 
	
	/**
	 * \brief An estimate of the variability of the world around the robot, which will be high
	 * in regions where the sensors are unreliable, and objects are likely to move. This is the
	 * actual underlying estimate data. It is exposed for convenient inspection.
	 * Don't modify it.
	 */
	ImageKalmanFilter eta; 
	
	/**
	 * \brief An estimate of the motor gains learned by the robot. This points to the 
	 * "steady state" dynamics which are used for making eye-movements on
	 * a slow timescale. This is the
	 * actual underlying estimate data. It is exposed for convenient inspection.
	 * Don't modify it.
	 */
	MatrixKalmanFilter *alpha; 
	
	/**
	 * \brief An estimate of the reliability of the different individual motors. 
	 * This information is currently used for doing reliable inverse kinematics. 
	 * The pointer is to the 
	 * "steady state" dynamics which are used for making eye-movements on
	 * a slow timescale. This is the
	 * actual underlying estimate data. It is exposed for convenient inspection.
	 * Don't modify it.
	 */
	MatrixKalmanFilter *nu; 
	
	/**
	 * An collection of  motor gains used to estimate a family of entire trajectories
	 * learned by the robot. This is the
	 * actual underlying estimate data. It is exposed for convenient inspection.
	 * Don't modify it.
	 **/ 
	std::vector<MatrixKalmanFilter> alphaDynamics; 	
	
	/**
	 * An collection of  motor reliability estimates used to estimate a family of 
	 * entire trajectories learned by the robot. This is the
	 * actual underlying estimate data. It is exposed for convenient inspection.
	 * Don't modify it.
	 **/ 
	std::vector<MatrixKalmanFilter> nuDynamics; 	
	
	/**
	 * An collection of timepoints for the motor trajectories learned by the robot.
	 * These are important for planning eye-movements on a sub-second timescale. This is the
	 * actual underlying estimate data. It is exposed for convenient inspection.
	 * Don't modify it.
	 **/ 
	std::vector<double> timeStamps; 
	
	/**
	 * \brief A directly learned inverse model that can be used by recommendActionForDesiredTransform().
	 * It can be enabled using setUseLearnedInverseModel(1), but it should not be. It is
	 * unreliable when there are more motors than translation dimensions, and the 
	 * computed inverse method used by default is much better at taking the reliability of
	 * the individaul motors into account. This is the
	 * actual underlying estimate data. It is exposed for convenient inspection.
	 * Don't modify it.
	 **/ 
	MatrixKalmanFilter inverseMotionModel; 
	
	
	/**
	 * \brief Write to a file.
	 */
	friend std::ostream& operator<< (std::ostream& ofs, const InternalMotionModel &feat); 
	
	/**
	 * \brief Read from a file.
	 */
	friend std::istream& operator>> (std::istream& ifs, InternalMotionModel &feat); 
	
	
private:
	void init(int numActuators=2, cv::Size obsSize=cv::Size(320,240), size_t numDynamicsModels=30); 
	void copyFrom(const InternalMotionModel& copy); 
	
	std::vector<cv::Rect> getRectanglePatchIntersection(cv::Size r1size, cv::Size r2size, cv::Size patchSize, int xoff, int yoff); 
	std::vector<double> getTauVectorMeanForAction(std::vector<double> action, int frameNum=-1); 
	
	
	void setCurrentAlpha(int frameNum) ; 
	void resetCurrentAlpha()	;
	
	int useMotorUncertaintyInLearning; 
	int useLearnedInverseModel; 
	
	/* Parameters from the robot */
	int numActuators; 
	int numDynamicsModels; 
	
	/* Internal parameters */
	int numMotionFeatures; // m
	int numTransformVals;  // n
	int numInvMotionFeatures; 
	
	/* Cached image memory for efficiency */
	//IplImage* croppedImage; 
	//IplImage* croppedVariance; 
	cv::Mat invCMat; 
	cv::Mat cMat; 
	cv::Mat absCMat; 
	cv::Mat features; 
	cv::Mat invFeatures; 
	
	cv::Mat matNx1a; 
	cv::Mat matNx1b; 
	cv::Mat matNxNa; 
	cv::Mat matNxNb; 
	 
	void updateSensorModel(IplImage* seenImage, const cv::Mat &tau); 
	void updateMotionModel(const cv::Mat &action, const cv::Mat &tau); 
	
	likelihood getObsLogLikelihood(const cv::Mat &tau, IplImage* seenImage, const cv::Mat &action, int frameNum=-1); 
	likelihood motionModelLogLikelihood(const cv::Mat &tau, const cv::Mat &action); 
	likelihood sensorModelLogLikelihood(const cv::Mat &tau, IplImage* seenImage); 
	likelihood sensorModelLogLikelihoodFast(const cv::Mat &tau, IplImage* seenImage, double scale) ;
	
	
	void getMotionFeatures(const cv::Mat &action, cv::Mat &features); // motion feature vector, mx1
	
	/*
	void addMatrixToStream(ostream& out, CvMat* matrix); 
	CvMat* readMatrixFromStream(istream &in); 
	*/
	virtual void readFromStream(std::istream& in); 
	virtual void addToStream(std::ostream& out) const; 
};

std::ostream& operator<< (std::ostream& ofs, const InternalMotionModel &model) ;
std::istream& operator>> (std::istream& ifs, InternalMotionModel &model) ;

#endif
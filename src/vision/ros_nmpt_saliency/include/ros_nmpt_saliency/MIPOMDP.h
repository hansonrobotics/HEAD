/*
 *  MIPOMDP.h
 *  FaceTracker
 *
 *  Created by Nicholas Butko on 10/3/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _MIPOMDP_H
#define _MIPOMDP_H

#include <opencv2/core/core_c.h>
#include <iostream>
#include <string>
#include "OpenCVHaarDetector.h"
#include "ObjectDetector.h"
#include "ImagePatchPyramid.h"
#include "MultinomialObservationModel.h"
#include "OpenLoopPolicies.h"
#include "ConvolutionalLogisticPolicy.h"
#include "ImageDataSet.h"

/**
 * \ingroup MPGroup
 * \brief <tt> <b> Machine Perception Primitive: </b> </tt> An implementation of
 * the "Multinomial IPOMDP" algorithm from Butko and Movellan, 2009 (see \ref 
 * bib_sec).
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class MIPOMDP  {
public: 
	
	/*Constructors*/
	
	
	/**
	 *\brief Load a saved MIPOMDP from a file. The file should have been saved 
	 * via the saveToFile() method. 
	 * 
	 * @param filename The name of a file that was saved via the saveToFile() 
	 * method. 
	 **/
	static MIPOMDP* loadFromFile(const char* filename); 
	
	/**
	 * \brief Main Constructor: Manually create an MIPOMDP. 
	 *
	 * An MIPOMDP needs to know how big the images it receives are,
	 * to what smaller size it should scale each image patch, the size (height /
	 * width) of the grid-cell tiling of the image, how many image patches there
	 * will be, the size (height/width) in grid-cells of each image patch, and 
	 * what object detector to apply. 
	 *
	 * In order to be able to properly save and load object detectors, the 
	 * class of the object detector must be specified in the MIPOMDP. To use an 
	 * object detector other than OpenCVHaarDetector, replace 
	 * "OpenCVHaarDetector" in MIPOMDP.h and MIPOMDP.cpp, and recompile the 
	 * NMPT library. (setHaarCascadeScaleFactor() and setHaarCascadeMinSize()
	 * will also need to be removed, or implemented in your own object detector
	 * for successful compilation). 
	 *
	 * @param inputImageSize The size of the images that will be given to the 
	 * IPP to turn into MIPOMDP observations. This allocates memory for 
	 * underlying data, but it can be changed easily later if needed without 
	 * recreating the object by calling the changeInputSize() functions. 
	 * 
	 * @param subImageSize The common size to which all image patches will be 
	 * reduced, creating the foveation effect. The smaller subImageSize is, the 
	 * faster search is, and the more extreme the  effect of foveation. This 
	 * allocates memory for underlying data, but it can be changed easily 
	 * later if needed without recreating the object by calling the 
	 * changeInputSize() functions.
	 *
	 * @param gridSize Size of the discretization of the image. The number of POMDP states is the 
	 * product of the demensions of this size (e.g. 21x21).
	 * 
	 * @param numSubImages Number of Patches in the Image Patch Pyramid. 
	 * 
	 * @param subImageGridPoints A matrix that describes the size and shape of 
	 * each level (patch) of the IP Pyramid. This must be a matrix with size 
	 * [numSubImages x 2]. Each row contains the  width and height of the 
	 * corresponding levels. These should be in order of *decreasing* size, so 
	 * that the largest Image Patch is first. For example, in Butko and Movellan 
	 * CVPR 2009, we used [21 21; 15 15; 9 9; 3 3]. Finaly, note that it is not 
	 * necessary that the largest patch cover the entire image. However, when 
	 * the largest patch is the same size as grid-cell-matrix, special 
	 * optimizations become available that reduce the complexity of the 
	 * algorithm when the  same image, or same frame of video, is fixated 
	 * multiple times. 
	 *
	 * @param haarDetectorXMLFile A file that was saved as the result of using 
	 * OpenCV's haar-detector training facilities. In order to be able to 
	 * properly save and load object detectors, the class of the object detector
	 * must be specified in the MIPOMDP. To use an object detector other than 
	 * OpenCVHaarDetector, simply replace "OpenCVHaarDetector" in MIPOMDP.h and 
	 * MIPOMDP.cpp, and recompile the NMPT library. 
	 **/
	MIPOMDP(CvSize inputImageSize, CvSize subImageSize, CvSize gridSize, 
			int numSubImages, CvMat* subImageGridPoints, 
			const char* haarDetectorXMLFile);   
	
	/**
	 *\brief Default Constructor: Create an MIPOMDP with the same properties as 
	 * the MIPOMDP used in  Butko and Movellan, CVPR 2009 (see \ref bib_sec).
	 * NOTE: This requires "data/haarcascade_frontalface_alt2.xml" to be a 
	 * haar-detector file that exists.
	 **/
	MIPOMDP(); 
	
	/**
	 *\brief Default Destructor: Deallocates all memory associated with the 
	 * MIPOMDP
	 **/
	virtual ~MIPOMDP();
	
	/*Save*/
	
	/**
	 *\brief Save MIPOMDP Data: Save data about the structure of the MIPOMDP so 
	 * that it can be persist past the current run of the program. This includes
	 * details about the structure of the IPP, the object detector used, the 
	 * parameters of the multionmial observation model, the policy used. It does
	 * not include details about the state of the MIPOMDP (the current location 
	 * of the target), but rather the properties of the model.
	 *
	 * @param filename The name of the saved file. 
	 **/
	void saveToFile(const char* filename);	
		
	/*Modify*/
	
	/**
	 *\brief Interface with MIPOMDP: Reset the belief about the location of the 
	 * object to be uniform over space (complete uncertainty). 
	 **/
	void resetPrior(); 
	
	/**
	 *\brief Interface with MIPOMDP: Tell MIPOMDP whether there is a possiblity 
	 * that the target can move. When searching a static frame, set this to 0. 
	 * When searching sequential frames of a movie, set this to 1.
	 * 
	 * By default, a simple dynamical model is assumed. The object is expected
	 * to move with brownian motion, with a small probability of jumping 
	 * randomly anywhere in the image. 
	 *
	 * NOTE: If a frame is searched with one of the search functions that 
	 * performs multiple fixations, dynamics are automatically, temporarily 
	 * disabled. At the end of the function call, the targetCanMove is restored 
	 * to its former state. 
	 * 
	 * @param flag Set to 1 (default) if searching sequential frames of a movie. 
	 * Set to 0 if searching a static image. 
	 **/
	void setTargetCanMove(int flag); 
	
	/**
	 *\brief Interface with MIPOMDP: Figure out whether the MIPOMDP expects that 
	 * the target can move. 
	 * 
	 * @return 1 (default) if searching sequential frames of a movie, 0 if 
	 * searching a static image. 
	 **/
	int getTargetCanMove(); 
	
	/**
	 *\brief  Interface with MIPOMDP: Find the shape of the grid that forms the 
	 * basis for the MIPOMDP state space and action space. 
	 * 
	 * @return The size of the visual grid world, where each location is a 
	 * potential object location (state) and potential eye-movement (action). 
	 **/
	CvSize getGridSize(); 
	
	/*Current state*/
	
	/**
	 *\brief Interface with MIPOMDP State: Given the current point, where does 
	 * the MIPOMDP's Convolutional logistic policy recommend looking? Since the 
	 * CLP (usually) gives stochastic output, this will not always return the 
	 * same result, even given the same belief state. 
	 * 
	 * @return A grid-cell point that should be good to fixate. 
	 **/
	CvPoint recommendSearchPointForCurrentBelief(); 
	
	/**
	 *\brief Interface with MIPOMDP State: Get an estimate of the probability 
	 * that the MIPOMDP knows the exactly correct location of the target. 
	 * 
	 * I.e. what is the probability of being correct given the (MxN)
	 * alternative forced choice task, "Where is the target" (where M is the 
	 * width of the grid, and N is the height). 
	 *
	 * @return  The probability that the MIPOMDP knows the exactly correct 
	 * location of the target. 
	 **/
	double getProb(); 
	
	/**
	 *\brief Interface with MIPOMDP State: The certainty (information-reward) 
	 * about the location of the target, i.e. the mutual information (minus a 
	 * constant) between all previous actions/observations and the target 
	 * location. 
	 * 
	 * @return The information-reward (Sum p*log(p)) of the current belief 
	 * state. Has range -(1/MN)log(MN):0 (where M is the width of the grid, and
	 * N is the height). 
	 **/
	double getReward(); 
	
	/**
	 *\brief Interface with MIPOMDP State: Find the grid-cell that is most 
	 * likely to be the location of  the target.
	 * 
	 * @return The grid-cell that is most likely to be the location of the 
	 * target. 
	 **/
	CvPoint getMostLikelyTargetLocation(); 
	
	/**
	 *\brief Interface with MIPOMDP State: Access the current belief 
	 * distribution directly. Changes each time one of the search functions is 
	 * called. 
	 * 
	 * This is representd as an image of type IPL_DEPTH_32F, 1 channle. This is 
	 * so that the belief-distribution can be visualized directly in a GUI. 
	 **/
	IplImage* currentBelief; //has size of grid
	
	/**
	 *\brief Interface with MIPOMDP State: A visually informative representation
	 * of the IPP Foveal represention. 
	 *
	 * This is meant as an image that is appropriate for display in a GUI, to 
	 * visualize the algorithm in action. The image has the same size as 
	 * inputImageSize. In order to increase efficiency, generation of this 
	 * visualization should be disabled if it is not going to be accessed. This 
	 * can be achieved by calling setGeneratePreview(0). 
	 **/
	IplImage* foveaRepresentation; //has size of full gray image		
	
	/*Search Functions*/
	
	/**
	 *\brief Search function: Search an image (or frame of video) at a location
	 * recommended by the Convolutional Logistic Policy, and update belief
	 * based on what was found. 
	 * 
	 * Since this is taken to be a new frame, the IPP knows not to use same-
	 * frame optimizations. 
	 * 
	 * @param grayFrame The image to search. Must be of type IPL_DEPTH_8U, 1
	 * channel. 
	 * 
	 * @return The most likely location of the search target. 
	 **/
	CvPoint searchNewFrame(IplImage* grayFrame);	
	
	/**
	 *\brief Search function: Search an image (or frame of video) at a location
	 * decided by the calling program. and update belief
	 * based on what was found. 
	 * 
	 * Since this is taken to be a new frame, the IPP knows not to use same-
	 * frame optimizations. 
	 * 
	 * @param grayFrame The image to search. Must be of type IPL_DEPTH_8U, 1
	 * channel. 
	 * 
	 * @param searchPoint The grid location to center the digital fovea for 
	 * further image processing.
	 * 
	 * @return The most likely location of the search target. 
	 **/
	CvPoint searchNewFrameAtGridPoint(IplImage* grayFrame, CvPoint searchPoint); 
	
	/**
	 *\brief Search function: Search an image (or frame of video) at a location
	 * recommended by the calling program, and update belief
	 * based on what was found. 
	 * 
	 * Since this is taken to be a repeat of the last frame, the IPP knows 
	 * that it can use same-frame optimizations (if appropriate). 
	 * 
	 * @param grayFrame The image to search. Must be of type IPL_DEPTH_8U, 1
	 * channel. 
	 * 
	 * @param searchPoint The grid location to center the digital fovea for 
	 * further image processing.
	 * 
	 * @return The most likely location of the search target. 
	 **/
	virtual CvPoint searchFrameAtGridPoint(IplImage* grayFrame,  CvPoint searchPoint); 
	
	/**
	 *\brief Search function: Search an image (or frame of video) at a sequence
	 * of fixation points recommended by the CLP, and update belief
	 * based on what was found. Employs an early-stop criterion of the first
	 * repeat fixation. Otherwise, stops at when confidence in the target 
	 * location reaches a maximum value. 
	 * 
	 * Since this is taken to be a new frame, the IPP knows not to use same-
	 * frame optimizations on the first saccade, and then 
	 * that it can use same-frame optimizations on subsequent fixations
	 * (if appropriate). 
	 * 
	 * @param grayFrame The image to search. Must be of type IPL_DEPTH_8U, 1
	 * channel. 
	 * 
	 * @param confidenceThresh Probability that max-location really contains
	 * the object before stopping. 
	 *
	 * @return The most likely location of the search target. 
	 **/
	CvPoint searchFrameUntilConfident(IplImage* grayFrame, 
									  double confidenceThresh); 
	
	/**
	 *\brief Search function: Search an image (or frame of video) at a sequence
	 * of fixation points chosen randomly, and update belief
	 * based on what was found. Employs an early-stop criterion of the first
	 * repeat fixation. Otherwise, stops at when confidence in the target 
	 * location reaches a maximum value. 
	 * 
	 * Since this is taken to be a new frame, the IPP knows not to use same-
	 * frame optimizations on the first saccade, and then 
	 * that it can use same-frame optimizations on subsequent fixations
	 * (if appropriate). 
	 * 
	 * @param grayFrame The image to search. Must be of type IPL_DEPTH_8U, 1
	 * channel. 
	 * 
	 * @param confidenceThresh Probability that max-location really contains
	 * the object before stopping. 
	 * 
	 * @return The most likely location of the search target. 
	 **/
	CvPoint searchFrameRandomlyUntilConfident(IplImage* grayFrame, 
											  double confidenceThresh); 
	
	/**
	 *\brief Search function: Search an image (or frame of video) at a sequence
	 * of fixation points recommended by the CLP, and update belief
	 * based on what was found. Fixates the specified number of times (no
	 * early stopping is used). 
	 * 
	 * Since this is taken to be a new frame, the IPP knows not to use same-
	 * frame optimizations on the first saccade, and then 
	 * that it can use same-frame optimizations on subsequent fixations
	 * (if appropriate). 
	 * 
	 * @param grayFrame The image to search. Must be of type IPL_DEPTH_8U, 1
	 * channel. 
	 *
	 * @param numfixations The number of fixations to apply. 
	 * 
	 * @return The most likely location of the search target. 
	 **/
	CvPoint searchFrameForNFixations(IplImage* grayFrame, int numfixations); 
	
	/**
	 *\brief Search function: Search an image (or frame of video) at a sequence
	 * of fixation points recommended by an open loop fixation policy, and 
	 * update belief based on what was found. Fixates the specified number of 
	 * times (no early stopping is used). 
	 * 
	 * Since this is taken to be a new frame, the IPP knows not to use same-
	 * frame optimizations on the first saccade, and then 
	 * that it can use same-frame optimizations on subsequent fixations
	 * (if appropriate). 
	 * 
	 * @param grayFrame The image to search. Must be of type IPL_DEPTH_8U, 1
	 * channel. 
	 *
	 * @param numfixations The number of fixations to apply. 
	 * 
	 * @param OLPolicyType The type of open-loop policy to use. Should be one of
	 * OpenLoopPolicy::RANDOM, OpenLoopPolicy::ORDERED, OpenLoopPolicy::SPIRAL.
	 * 
	 * @return The most likely location of the search target. 
	 **/
	CvPoint searchFrameForNFixationsOpenLoop(IplImage* grayFrame, 
											 int numfixations, 
											 int OLPolicyType); 
	
	/**
	 *\brief Search function: Search an image (or frame of video) at a sequence
	 * of fixation points recommended by the CLP, and update belief
	 * based on what was found. Fixates the specified number of times (no
	 * early stopping is used). Visualize the process in a provided OpenCV
	 * UI Window, at a specified frame rate.
	 * 
	 * Since this is taken to be a new frame, the IPP knows not to use same-
	 * frame optimizations on the first saccade, and then 
	 * that it can use same-frame optimizations on subsequent fixations
	 * (if appropriate). 
	 * 
	 * @param grayFrame The image to search. Must be of type IPL_DEPTH_8U, 1
	 * channel. 
	 *
	 * @param numfixations The number of fixations to apply. 
	 *
	 * @param window The string handle (name) of an OpenCV UI Window that
	 * you have previously created. Output will displayed in this window. 
	 * 
	 * @param msec_wait Number of milliseconds to wait between fixations so
	 * that the process of each fixation can be appreciated visually by the
	 * user. 
	 * 
	 * @return The most likely location of the search target. 
	 **/	
	CvPoint searchFrameForNFixationsAndVisualize(IplImage* grayFrame, 
												 int numfixations, 
												 const char* window, 
												 int msec_wait); 
	
	/**
	 *\brief Search function: Search an image (or frame of video) at every
	 * grid-point (no early stopping is used). 
	 * 
	 * Since this is taken to be a new frame, the IPP knows not to use same-
	 * frame optimizations on the first saccade, and then 
	 * that it can use same-frame optimizations on subsequent fixations
	 * (if appropriate). 
	 * 
	 * @param grayFrame The image to search. Must be of type IPL_DEPTH_8U, 1
	 * channel. 
	 * 
	 * @return The most likely location of the search target. 
	 **/
	CvPoint searchFrameAtAllGridPoints(IplImage* grayFrame); 
	
	/**
	 *\brief Search function: Apply the Object Detector to the entire input
	 * image. 
	 * 
	 * The belief map is not changed by this operation. The count
	 * vector of objects found in each grid-cell is set. The location of the
	 * grid-cell with the most found objects is returned. The fovea 
	 * representation is set to contain the entire high-resolution image, 
	 * overlaid with grid-cells. 
	 **/
	CvPoint searchHighResImage(IplImage* grayFrame); 	
	
	/*IPP Interface*/
	
	/**
	 *\brief Interface with IPP: Change the size of the input image and the 
	 * downsampled image patches.  Omitting a newSubImageSize causes the 
	 * smallest-used-scale to have a 1-1 pixel mapping with the downsampled 
	 * image patch -- i.e. information is not lost in the smallest scale. 
	 *
	 * @param newInputSize The size of the next image that will be searched. 
	 **/
	void changeInputImageSize(CvSize newInputSize);
	
	/**
	 *\brief Interface with IPP: 
	 * Change the size of the input image and the downsampled image patches.  
	 * 
	 * @param newInputSize The size of the next image that will be searched. 
	 * 
	 * @param newSubImageSize The desired size of the downsampled image patches. 
	 * If the subImageSize is too small (below getMinSize()), the smallest scale 
	 * is dropped and subImageSize is scaled up proportionally to the next 
	 * scale. This process is repeated until subImageSIze is greater than 
	 * getMinSize(). By default, minSize is 60x40. 
	 **/
	void changeInputImageSize(CvSize newInputSize, CvSize newSubImageSize);
		
	/**
	 *\brief Interface with IPP: The total number of levels that the IP Pyramid 
	 * has. 
	 *
	 * Note that this may be different from the number of scales that the IP 
	 * Pyramid is using. If the subImageSize is too small (below getMinSize()), 
	 * the smallest scale is dropped and subImageSize is scaled up 
	 * proportionally to the next scale. This process is repeated until 
	 * subImageSIze is greater than getMinSize(). To find out how many scales 
	 that the IPP is using, call getUsedScales(). 
	 **/
	int getNumScales(); 	
	
	/**
	 *\brief Interface with IPP: Map a pixel location in the original image into
	 * a grid-cell. 
	 **/
	CvPoint gridPointForPixel(CvPoint pixel); 
	
	/**
	 *\brief Interface with IPP: Find the pixel in the original image that is in
	 * the center of a grid-cell. 
	 **/
	CvPoint pixelForGridPoint(CvPoint gridPoint); 	
	
	/**
	 *\brief Interface with IPP: Turns on/off the code that modifies 
	 * foveaRepresentation to visualize the  process of fixating. 
	 *
	 * @param flag Set to 0 if visualization is not desired (more efficient) or 
	 * to 1 if visualization is desired.
	 **/
	void setGeneratePreview(int flag); 
	
	/**
	 *\brief Interface with IPP: Set the minimum allowed subImageSize. 
	 * 
	 * If the subImageSize is too small (below getMinSize()), the smallest scale 
	 * is dropped and subImageSize is scaled up proportionally to the next 
	 * scale. This process is repeated until subImageSIze is greater than 
	 * getMinSize(). By default, minSize is 60x40. 	 
	 * 
	 * The will have no effect on the current subImageSize, or getUsedScales() 
	 * until changeInputImageSize() is called. 
	 *
	 * @param minsize The minimum allowed subImageSize.
	 **/
	void setMinSize(CvSize minsize); 
	
	/**
	 *\brief Interface with IPP: Set whether same-frame optimizations are being 
	 * used. 
	 *
	 * Under certain conditions, the computation needed to search a frame a 
	 * second time are less than the computations needed to search it a first 
	 * time. In these conditions, the same-frame optimizations will 
	 * automatically be used. However, this requires setting setNewImage() each 
	 * time the image to search changes (i.e. a new frame). If you are in a 
	 * situation in which you know that each frame will only be fixated at one 
	 * point, you may wish to turn same-frame optimizations off.  
	 *
	 * Generally same-frame optimizations should not be turned on unless you 
	 * know that they were turned on automatically. Turning them on when 
	 * inappropriate will lead to incorrect behavior. In general, it is 
	 * appropriate to turn them on if the first scale (largest scale) in the IPP
	 * is the same size as entire visual field. 
	 *
	 * @param flag If 0, Same Frame Optimizations will not be used. If 1, Same 
	 * Frame Optimizations will be used regardless of whether or not it's 
	 * appropriate. Be careful setting this to 1. 
	 **/
	void useSameFrameOptimizations(int flag); 	
	
	/**
	 *\brief Interface with IPP: Save a variety of visual representations of the
	 * process of fixating with an IPP to image files. 
	 * 
	 * This method saves a series of .png image files, each with a prefix given
	 * by base_filename. Images with the following suffix are created: 
	 * \li FullInputImage - The full input image contained in grayFrame. 
	 * \li Scale-[0:N] - The down-sampled representation of each image patch. 
	 * \li FoveatedInputImage - A reconstruction of the full image using the 
	 * donwsampled patches.
	 * \li FoveatedInputImageWithLooking - Same as above, with white boxes drawn
	 * around each scale.
	 * \li FoveatedInputImageWithGrid - Same as above, but with a grid overlayed
	 * showing the grid-cells.
	 * \li FullInputImageWithGrid - Full image with black rectangles showing
	 * grid-cell locations. 
	 * \li FullInputImageWithLooking - Same as above, but with wite boxes drawn
	 * around each scale.
	 * 
	 * Additionally, one CSV file is created, suffix "FaceCounts.csv", which
	 * records the output of the object detector on the foveated representation
	 * in each grid-cell. 
	 *
	 * @param grayFrame The image to search. This image should have size
	 * inputImageSize, and be of type IPL_DEPTH_8U with a single channel. 
	 *
	 * @param searchPoint The center of fixation.
	 *
	 * @param base_filename All of the files generated by this function will be
	 * given this as a prefix. 
	 **/
	void saveVisualization(IplImage* grayFrame, CvPoint searchPoint, 
						   const char* base_filename); 
	/**
	 *\brief Interface with IPP: Get the count of faces found in each grid
	 * cell after the last search call. 
	 **/
	IplImage* getCounts(); 
	
	/*Object Detector Interface*/
	
	/**
	 *\brief Interface with Object Detector: Sets the factor by which the 
	 * image-patch-search-scale is increased. Should be greater than 1. By 
	 * default, the scale factor is 1.1, meaning that faces are searched for at
	 * sizes that increase by 10%.
	 *
	 * @param factor The size-granularity of object search.
	 **/
	void setHaarCascadeScaleFactor(double factor); 
	
	/**
	 *\brief Interface with Object Detector: Sets the minimum patch size at
	 * which the classifier searches for the object. By default, this is 0, 
	 * meaning that the smallest size appropriate to XML file is used. In the 
	 * case of the frontal face detector provided, this happens to be 20x20 
	 * pixels.
	 *
	 * @param size The width/height of the smallest patches to try to detect the
	 * object.
	 */
	void setHaarCascadeMinSize(int size); 
	
	
	/*Observation Model Interface*/
	
	/**
	 *\brief Interface with Observation Model: Fit a multinomial observation 
	 * model by recording the object detector output at different fixation
	 * points, given known face locations contained in an ImageDataSet.
	 * 
	 * Resets the current observation model to a uniform prior before tabulating
	 * the data. 
	 * 
	 * @param trainingSet An iamge dataset, consisting of image files and 
	 * labeled object locations. The first two indices (0, 1) of all labels in
	 * the trainingSet should be the width and height (respectively) of the 
	 * object's center in the image. 
	 **/
	void trainObservationModel(ImageDataSet* trainingSet); 
	
	/**
	 *\brief Interface with Observation Model: Add data to an existing 
	 * multinomial observation model by recording the object detector output at
	 * different fixation points, given known face locations contained in an 
	 * ImageDataSet.
	 * 
	 * Does not reset the current observation model to a uniform prior before 
	 * tabulating the data. 
	 * 
	 * @param trainingSet An iamge dataset, consisting of image files and 
	 * labeled object locations. The first two indices (0, 1) of all labels in 
	 * the trainingSet should be the width and height (respectively)
	 * of the object's center in the image. 
	 **/
	void addDataToObservationModel(ImageDataSet* trainingSet); 
	
	/**
	 *\brief Interface with Observation Model:  Reset the experience-counts used
	 * to estimate the multinomial parameters.  Sets all counts to 1.
	 *
	 * The multinomial distribution probabilities are estimated from experience
	 * counts. The table of counts forms a dirichlet posterior over the 
	 * parameters of the multinomials. When the counts are reset to 1, there is
	 * a flat prior over multinomial parameters, and all outcomes are seen as
	 * equally likely. This is a particularly bad model, and will make it
	 * impossible to figure out the location of the face, so only call
	 * resetCounts if you're then going to call updateProbTableCounts for enough
	 * images to build up a good model. 
	 **/
	void resetModel(); 		
	
	/**
	 *\brief Interface with Observation Model: Combine the evidence from two
	 * MIPOMDPs' multinomial observation models (merge their counts and subtract
	 * off the extra priors).
	 *
	 * @param otherPomdp A second MIPOMDP with a model that has been fit to
	 * different data than this one:  We can estimate the model for the combined
	 * set of data by simply adding the counts, and subtracting duplicate
	 * priors. In this way, we can efficiently compose models fit to different
	 * subsets of a larger dataset (e.g. for cross-validation). 
	 **/
	void combineModels(MIPOMDP* otherPomdp); 		
	
	/*Policy Interface*/
	
	/**
	 *\brief Interface with CLP: Tell the CLP what kind of convolution policy to
	 * use in the future. 
	 * 
	 * @param policyNumber Must be one of: 
	 * \li ConvolutionalLogisticPolicy::BOX
	 * \li ConvolutionalLogisticPolicy::GAUSSIAN
	 * \li ConvolutionalLogisticPolicy::IMPULSE
	 * \li ConvolutionalLogisticPolicy::MAX
	 **/
	void setPolicy(int policyNumber); 
	
	/**
	 *\brief Interface with CLP: Tell the CLP the shape of the convolution
	 * kernel to use.
	 * 
	 * @param softmaxGain Has the following effect for: 
	 * \li ConvolutionalLogisticPolicy::BOX - The kernel is a square with
	 * integral=softmaxGain.
	 * \li ConvolutionalLogisticPolicy::GAUSSIAN - The kernel is a Gaussian with
	 * integral=softmaxGain
	 * \li ConvolutionalLogisticPolicy::IMPULSE - The kernel is an impulse
	 * response with value softmaxGain
	 * \li ConvolutionalLogisticPolicy::MAX - No effect
	 *
	 * @param boxSize Has the following effect for: 
	 * \li ConvolutionalLogisticPolicy::BOX - The kernel is a boxSize x boxSize
	 * grid-cell square.
	 * \li ConvolutionalLogisticPolicy::GAUSSIAN - The kernel is a Gaussian with
	 * standard-deviation boxSize grid-cells.
	 * \li ConvolutionalLogisticPolicy::IMPULSE - No effect
	 * \li ConvolutionalLogisticPolicy::MAX - No effect
	 **/
	void setHeuristicPolicyParameters(double softmaxGain, double boxSize); 
	
	
	/**
	 * \brief Change the file used by the object detector for doing detecting.
	 * This is critical if a weights file is located at an absolute path that 
	 * may have changed from training time. 
	 * 
	 * When an ObjectDetector is loaded from disk, it will try to reload its
	 * weights file from the same source used in training. If this fails, a
	 * warning will be printed, and the detector's source will need to be set.
	 */
	void setObjectDetectorSource(std::string newFileName); 
	
protected:
	
	MIPOMDP(CvSize gridSize); 
	
	ImagePatchPyramid* ipp; 
	MultinomialObservationModel* obsmodel; 
	ConvolutionalLogisticPolicy* policy; 
	
	CvSize gridSize; 
	
	int useOnlyFaceCenter; 
	int useObsProbTables; 
	int dynamicsoff; 
	
	int repeatOff; 
	
	IplImage* actProb;       //has size of grid
	IplImage* fixationCount; //has size of grid
		
	virtual void runForwardBeliefDynamics(); 
	virtual void setObservationProbability(CvPoint searchPoint); 
	
	float randomFloat(); 	
	
	virtual void updateProbTableCounts(IplImage* grayFrame, CvRect objectLocation) ; 
	void normalizeProbTables() ; 
	
};



#endif

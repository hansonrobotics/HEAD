/*
 *  Feature.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/6/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */



#ifndef FEATURE_H
#define FEATURE_H

#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <vector>
#include <string>

#include "ImagePatch.h"
#include "PatchList.h"


/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A purely virtual class for providing the 
 * skeleton for specific features. This allows different features to be 
 * manipulated using a common set of tools.
 *
 * In general, we conceive of a feature as something that takes an image patch 
 * as input, and produces a scalar output. It may be necessary at times for a
 * feature to apply itself efficiently to multiple image patches, using
 * the PatchList data structure. Also, it will be necessary to find features 
 * similar to this one, and may be convenient to represent a feature in a
 * compact parameterized form. Thus, a feature should implement the following:
 * 
 * virtual double evaluateImagePatch(const ImagePatch* patch);
 * virtual void filterPatchList( PatchList* patches); 
 * virtual void setFeatureParameters(const CvMat* paramVec);
 *
 * Additionally, the following utilities are provided but may be overridden:
 * 
 * virtual vector<Feature*> getSimilarFeatures(int numFeatures);
 * virtual IplImage* visualizeFeature();
 * virtual string debugInfo() const;
 * virtual ~Feature(); 
 *
 * NOTE ON SUBCLASSING: In order for everything to work properly, a few rules 
 * need to be followed. First, subclasses should be able to completely 
 * reinitialize their own member variables from their parameter vector. This
 * allows us to save a parameter vector to disk and then reconstruct the 
 * feature later. Second, subclasses should set the protected "featureName" 
 * variable, which helps determine which what the actual type of a saved 
 * feature is. Third (finally), Feature.cpp's getFeatureOfType method must
 * be modified to be aware of your feature class, by calling the correct
 * constructor (your constructor) when a feature of your type is read from disk.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class Feature {
public:
	
	/**
	 *\brief Turn an ImagePatch into a scalar value. This is the basic job 
	 * of any feature. Must be overridden in derived classes.
	 * 
	 * @param patch The image patch to evaluate.
	 * @return The value of the Image Patch according to the feature.
	 **/
	virtual double evaluateImagePatch(const ImagePatch* patch) = 0; 
	
	
	/**
	 *\brief Turn an ImagePatch into a scalar value. This is the basic job 
	 * of any feature.
	 * 
	 * @param patch The image patch to evaluate.
	 * @return The value of the Image Patch according to the feature.
	 **/
	virtual double evaluateImagePatch(const ImagePatch &patch) ; 
	
	/**
	 * \brief Apply the feature to a PatchList data structure. Must be 
	 * overridden in derived classes. 
	 * 
	 * This is very 
	 * important for efficiently filtering a whole image. The data for the
	 * filtering operation as well as the destination are contained in that
	 * data structure. See PatchList.h for details. 
	 * 
	 * @param patches Remaining patches in a large image to be filtered.
	 **/
	virtual void filterPatchList( PatchList* patches) =0;
	
	/**
	 * \brief Basic feature initialization. Must be overridden in derived 
	 * classes.
	 *
	 * Subclasses should be able to completely 
	 * reinitialize their own member variables from their parameter vector. This
	 * allows us to save a parameter vector to disk and then reconstruct the 
	 * feature later.  
	 * 
	 * @param paramVec A set of values sufficient to reconstruct a feature, 
	 * or to construct a new one automatically. Must have type CV_64FC1
	 **/
	virtual void setFeatureParameters(const cv::Mat &paramVec)=0;
	DEPRECATED(virtual void setFeatureParameters(const CvMat* paramVec));
		
	/**
	 * \brief Get features similar to the current feature. This is useful for
	 * searching through feature space for local optima. 
	 *
	 * By default, this randomly mutates the elements of the feature parameter
	 * vector. Override this function if different behavior is desired.
	 * 
	 * @param numFeatures Number of nearby features to grab.
	 * 
	 * @return Set of nearby features.
	 **/
	virtual std::vector<Feature*> getSimilarFeatures(int numFeatures = 1);
	
	/**
	 * \brief Get a nice visual representation of the feature. 
	 *
	 * By default, this simply returns the "kernel" protected member variable, 
	 * which subclasses should set during "setFeatureParameters". 
	 * 
	 * @return An image representing the feature in a visual way.
	 **/
	virtual void getFeatureVisualization(cv::Mat &dest); 
	DEPRECATED(virtual IplImage* visualizeFeature()); 
	
	
	/**
	 * \brief Prints information about this feature for help debugging. By 
	 * default, this prints the feature type, and its parameters.
	 *
	 * @return Information about this feature useful for debugging.
	 **/
	virtual std::string debugInfo() const; 	
	
	/**
	 * \brief Destructor. By default, this cleans up the parameters, valid
	 * parameter ranges, kernel image, and name. Subclasses are responsible
	 * for cleaning up extra memory. 
	 **/
	virtual ~Feature(); 
	
	/**
	 * \brief Get a subclass based on its name. If you subclass Feature, you 
	 * are responsible for modifying Feature.cpp to make this function aware 
	 * of your feature. This function is important for saving the state of 
	 * features for classifiers.
	 * 
	 * @return An object that is a subclass of Feature.
	 **/
	static Feature* getFeatureOfType(std::string featureName, cv::Size patchSize); 
	
	
	/**
	 * \brief Get a feature with same type / dimension as this one, but random
	 * parameters.
	 * 
	 * @return A new feature.
	 **/
	Feature* getFeatureOfSameTypeAndSize() const; 
	
	/**
	 * \brief Get a feature with the same type / dimension / parameters as this
	 * one.
	 * 
	 * @return A new feature.
	 **/
	Feature* copy() const; 
	
	/**
	 * \brief Apply feature to multiple ImagePatch objects.
	 *
	 * @param imagePatches Patches to evaluate.
	 * @param scalarVals Result of evaluation, in an STL vector data structure.
	 **/
	void evaluateImagePatches(const std::vector<ImagePatch*> &imagePatches, std::vector<double>& scalarVals) ;
	
	/**
	 * \brief Apply feature to multiple ImagePatch objects.
	 *
	 * @param imagePatches Patches to evaluate.
	 * @param scalarVals Result of evaluation, in an STL vector data structure.
	 **/
	void evaluateImagePatches(const std::vector<ImagePatch> &imagePatches, std::vector<double>& scalarVals) ;
	
	/**
	 * \brief Apply feature to multiple ImagePatch objects.
	 *
	 * @param imagePatches Patches to evaluate.
	 * @param scalarVals Result of evaluation, in a CvMat data structure. If the
	 * provided CvMat is NULL or has inappropriate size, it will be freed and 
	 * recreated.
	 **/
	void evaluateImagePatches(const std::vector<ImagePatch*> &imagePatches, cv::Mat &scalarVals) ;
	
	/**
	 * \brief Apply feature to multiple ImagePatch objects.
	 *
	 * @param imagePatches Patches to evaluate.
	 * @param scalarVals Result of evaluation, in a CvMat data structure. If the
	 * provided CvMat is NULL or has inappropriate size, it will be freed and 
	 * recreated.
	 **/
	void evaluateImagePatches(const std::vector<ImagePatch> &imagePatches, cv::Mat &scalarVals) ;
	DEPRECATED(void evaluateImagePatches(const std::vector<ImagePatch*> &imagePatches, CvMat*& scalarVals)) ;
		
	
	/**
	 * \brief Get the size of patch that is "natural" for this feature. 
	 *
	 * Generally, features have a basic size. For example, a convolution-based
	 * feature has the size of the convolution kernel. Asking the feature to 
	 * filter an ImagePatch that does not match this size may result in 
	 * degraded performance.
	 *
	 * @return Preferred patch size.
	 **/
	cv::Size getPatchSize() const; 
	
	/**
	 * \brief Write to a file.
	 **/
	friend std::ostream& operator<< (std::ostream& ofs, Feature* feat); 
	
	/**
	 * \brief Read from a file.
	 **/
	friend std::istream& operator>> (std::istream& ifs, Feature*& feat); 
	
		
protected:	
	
	//static std::vector<IplImage*> imagePatchToIntegralPatch(std::vector<IplImage*> imagePatches); 
	//static std::vector<IplImage*> integralPatchToImagePatch(std::vector<IplImage*> integralPatches); 
	
	CvMat* parameters; //Must have type CV_64FC1
	CvMat* minParamVals;
	CvMat* maxParamVals;
	IplImage* kernel; 
	
	enum paramTypes {
		discrete, real
	}; 
	
	paramTypes parameterType; 
	CvSize patchSize; 
	int prefersIntegral; 
	char* featureName; 
	CvRNG rng; 
	
	Feature(cv::Size expectedPatchSize=cvSize(0,0)); 
	
	void setRandomParameterValues(); 
	void checkParameterBounds(CvMat* params); 
		
	int equals(Feature* other); 
	
}; 

std::ostream& operator<< (std::ostream& ofs, Feature *feat); 
std::istream& operator>> (std::istream& ifs, Feature *&feat); 
//istream& operator>> (istream& ifs, Feature** feat); 

#endif

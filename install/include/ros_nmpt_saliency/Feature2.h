/*
 *  Feature2.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/22/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */





#ifndef FEATURE2_H
#define FEATURE2_H

#include <opencv2/core/core.hpp>

#include <iostream>
#include <vector>
#include <string>

#include "ImagePatch2.h"
#include "PatchList2.h"

/*
class Feature2; 
template<Feature2*> cv::FileStorage& cv::operator<< (cv::FileStorage &fs, const Feature2* &feat); 
template<Feature2*> cv::FileNode& cv::operator>> (cv::FileNode &fs, Feature2* &feat); 
*/

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
class Feature2 {
public:
		
	/**
	 *\brief Turn an ImagePatch into a scalar value. This is the basic job 
	 * of any feature.
	 * 
	 * @param patch The image patch to evaluate.
	 * @return The value of the Image Patch according to the feature.
	 **/
	virtual double evaluateImagePatch(const ImagePatch2 &patch) const = 0; 
	
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
	virtual void filterPatchList( PatchList2 * patches) const =0;
	
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
	virtual std::vector<Feature2*> getSimilarFeatures(int numFeatures = 1) const;
	
	/**
	 * \brief Get a nice visual representation of the feature. 
	 *
	 * By default, this simply returns the "kernel" protected member variable, 
	 * which subclasses should set during "setFeatureParameters". 
	 * 
	 * @return An image representing the feature in a visual way.
	 **/
	virtual void getFeatureVisualization(cv::Mat &dest) const; 	
	
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
	virtual ~Feature2(); 
	
	/**
	 * \brief Get a subclass based on its name. If you subclass Feature, you 
	 * are responsible for modifying Feature.cpp to make this function aware 
	 * of your feature. This function is important for saving the state of 
	 * features for classifiers.
	 * 
	 * @return An object that is a subclass of Feature.
	 **/
	static Feature2* getFeatureOfType(std::string featureName, cv::Size patchSize); 
	
	
	/**
	 * \brief Unarchive from cv::FileStorage, when saved using 
	 * writeToFile.
	 * 
	 * @return An object that is a subclass of Feature.
	 **/
	static Feature2* readFromFile(const cv::FileNode &storage); 
	
	/**
	 * \brief Archive to cv::FileStorage,
	 * 
	 *  @param storage An instance of cv::FileStorage that is open for 
	 * writing.
	 *
	 *  @param varname Name used as a key for unarchiving.
	 **/
	void writeToFile(cv::FileStorage &storage, const std::string &varname) const; 
	
	
	/**
	 * \brief Get a feature with same type / dimension as this one, but random
	 * parameters.
	 * 
	 * @return A new feature.
	 **/
	Feature2* getFeatureOfSameTypeAndSize() const; 
	
	/**
	 * \brief Get a feature with the same type / dimension / parameters as this
	 * one.
	 * 
	 * @return A new feature.
	 **/
	Feature2* copy() const; 
	
	
	/**
	 * \brief Apply feature to multiple ImagePatch objects.
	 *
	 * @param imagePatches Patches to evaluate.
	 * @param scalarVals Result of evaluation, in a CvMat data structure. If the
	 * provided CvMat is NULL or has inappropriate size, it will be freed and 
	 * recreated.
	 **/
	void evaluateImagePatches(const std::vector<ImagePatch2> &imagePatches, cv::Mat &scalarVals) const ;
	
	
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
	//template<Feature2*> friend cv::FileStorage& cv::operator << (cv::FileStorage & fs, const Feature2* &rhs); 
	friend std::ostream& operator<< (std::ostream& ofs, const Feature2* feat); 
	
	/**
	 * \brief Read from a file.
	 **/
	//template<Feature2*> friend cv::FileNode& cv::operator >> (cv::FileNode &fs, Feature2* &feat); 
	friend std::istream& operator>> (std::istream& ifs, Feature2* &feat); 
	
	
	
	
protected:	
	cv::Mat parameters, minParamVals, maxParamVals, kernel; 
	cv::Size patchSize; 
	int prefersIntegral; 
	std::string featureName; 
	
	Feature2(cv::Size expectedPatchSize=cvSize(0,0)); 
	
	void setRandomParameterValues(); 
	void checkParameterBounds(cv::Mat &params) const; 
	
	int equals(Feature2* other); 
	
}; 

std::ostream& operator<< (std::ostream& ofs, const Feature2 *feat); 
std::istream& operator>> (std::istream& ifs, Feature2 *&feat); 
/*
template<Feature2*> cv::FileStorage& cv::operator <<  (cv::FileStorage & fs, const Feature2* &rhs) {
	if (_FEATURE_DEBUG) std::cout << "Adding Feature to Storage" << std::endl; 
	fs << "{" << "name" << rhs->featureName << "size_w" << rhs->patchSize.width
	<< "size_h" << rhs->patchSize.height << "parameters" << rhs->parameters << "}"; 
	if (_FEATURE_DEBUG) std::cout << "Finished outputting feature" << std::endl; 
	return fs; 
}


*/

#endif

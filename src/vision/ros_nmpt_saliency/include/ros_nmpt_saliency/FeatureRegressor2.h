/*
 *  FeatureRegressor2.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/23/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#ifndef FEATUREREGRESSOR2_H
#define FEATUREREGRESSOR2_H

#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
//#include "ImagePatch2.h"
//#include "Feature2.h"

class Feature2; 
class ImagePatch2; 
class PatchList2; 

/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A data structure that learns a non-linear
 * tuning curve for mapping feature outputs to image labels. This is the 
 * "weak learner" used by GentleBoost. 
 * 
 * A FeatureRegressor is initialized with a feature. It must then be trained,
 * using a set of image patches, labels indicating whether the image patches are
 * positive or negative examples (-1 for negative, +1 for positive), and weights
 * for each example, which specify how much the learner should care about 
 * getting the answer of that example correct, relative to the other examples.
 *
 * Two regularization parameters control the behavior of the feature regressor:
 * TAU describes the window-of-influence of each example, and can range from 
 * (0-1], with a default of 0.05. EPS describes what happens in regions with 
 * no data. If EPS is high, the classifier will revert to the mean label in
 * regions with no data. If EPS is 0, the classifier will use the label of
 * the nearest point. The default is 0.001. Since these are regularization 
 * parameters, it doesn't make sense to tune them on an individual basis. 
 * Therefore, they are static variables. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class FeatureRegressor2  {
public: 
	
	/**
	 * \brief Window-Size regularization Parameter. 
	 *
	 * TAU describes the window-of-influence of each example, and can range from 
	 * (0-1], with a default of 0.05. 
	 */
	static double TAU; 
	
	/**
	 * \brief No-Evidence regularization Parameter.
	 * 
	 * EPS describes what happens in regions with 
	 * no data. If EPS is high, the classifier will revert to the mean label in
	 * regions with no data. If EPS is 0, the classifier will use the label of
	 * the nearest point. The default is 0.001. 
	 */
	static double EPS; 	
	
	
	/**
	 * \brief Default Constructor.  
	 */
	FeatureRegressor2(); 
	
	/**
	 * \brief Set feature. Must call train again after setting feature. 
	 */
	void setFeature(const Feature2* feature); 
	
	/**
	 * \brief Constructor. 
	 *
	 * @param feature The feature that transforms ImagePatch data into a scalar
	 * value. 
	 */
	FeatureRegressor2(const Feature2* feature); 
	
	/**
	 * \brief Destructor. 
	 */
	~FeatureRegressor2(); 
	
	/**
	 * \brief Copy Constructor
	 **/
	FeatureRegressor2(const FeatureRegressor2 &copy); 
	
	/**
	 * \brief Assignment operator
	 **/
	FeatureRegressor2 & operator=(const FeatureRegressor2 &rhs); 
	
	/**
	 * \brief Train the regression model with data.
	 * 
	 * @param numTableElements The learned tuning curve is discretized into 
	 * a number of discrete bins, based on the range of feature outputs 
	 * observed in training. A reasonable number of bins is 100.
	 *
	 * @param patches The image patches that are or are not examples of the 
	 * class of interest.
	 *
	 * @param labels Labels indicating whether each patch is or isn't the object
	 * we're trying to detect, +1 means yes, -1 means no.
	 *
	 * @param dataWeights Specify how much the learner should care about 
	 * getting the answer of that example correct, relative to the other examples.
	 */
	void train(int numTableElements, const  std::vector<ImagePatch2> &patches, const cv::Mat &labels, const cv::Mat &dataWeights);
	
	/**
	 * \brief Try to predict whether new ImagePatch data are the object we're
	 * trying to detect.
	 * 
	 * @param patches The image patches that are or are not examples of the 
	 * class of interest.
	 *
	 * @param scalarVals A graded confidence that the patch is or is not the 
	 * object of interest (set by the algorithm), ranging from +1 (sure that it 
	 * is the object) through 0 
	 * (completely unsure) to -1 (sure that it's not an instance of the object). If the
	 * provided CvMat is NULL or has inappropriate size, it will be freed and 
	 * recreated.
	 **/
	void predict(const std::vector<ImagePatch2> &patches, cv::Mat &scalarVals) const; 
	
	/**
	 * \brief Try to predict whether a set of patches corresponding to image 
	 * locations are the object. This is used for efficiently searching through
	 * a whole image for the object.
	 * 
	 * @param patches Candidate locations in the image where it is thought the
	 * object may be. The results are stored in the PatchList data structure. 
	 **/
	void predictPatchList( PatchList2* patches) const;
	
	/**
	 * \brief Assuming all patches in an image were filtered with the feature,
	 * this will make a prediction for each pixel as to whether it is the top
	 * left corner of the trained object. Modifies the input image.
	 * 
	 * @param image Precondition: contains the filter outputs at each point in the 
	 * image. Postcondition: contains the prediction (in -1:1) about whether the
	 * top-left of the object is located at each pixel. 
	 **/
	void applyLUTToImage(cv::Mat &image) const; 
	
	/**
	 * \brief Combine predictions from two regressors. This can be used to "boost"
	 * the confidence of a single feature in a computationally efficient way.
	 * 
	 * @param other Another FeatureRegressor, trained with either different
	 * data, or a different weighting, but that has the same base feature.
	 **/
	void combineLUTs(const FeatureRegressor2 &other);
	
	
	/**
	 * \brief Gets the range of feature values computed during training (max-min).
	 * This can be useful for detecting degenerate features that produce extremely
	 * small ranges of values.
	 * 
	 * @return The range of feature values computed during training (max-min).
	 **/
	double getLUTRange() const; 	
	
	/**
	 * \brief Get the feature that this FeatureRegressor was created with. 
	 * This returns the actual feature object used by the regressor, which is
	 * a copy of the one it was trained with.
	 * 
	 * return The actual feature object of the regressor. 
	 **/
	Feature2* getFeature() const; 
	
	/**
	 * \brief Get the patch size of the FeatureRegressor's Feature.  
	 *
	 * @return Base patch size for this regressor.
	 **/
	cv::Size getFeaturePatchSize() const; 
		
	/**
	 * \brief Write to a file.
	 **/
	friend cv::FileStorage& operator << (cv::FileStorage &fs, const FeatureRegressor2 &reg); 
	friend std::ostream& operator<< (std::ostream& ofs, const FeatureRegressor2 &reg); 
	
	
	
	/**
	 * \brief Read from a file.
	 **/
	friend void operator >> (const cv::FileNode &fs, FeatureRegressor2 &reg); 
	friend std::istream& operator>> (std::istream& ifs, FeatureRegressor2 &reg); 
	
	//friend class ImagePatch; 
private:
	Feature2* patchFeature; 
	cv::Mat lookUpTable; 
	double lookUpTableMin; 
	double lookUpTableMax; 
	
};

std::ostream& operator<< (std::ostream& ofs, const FeatureRegressor2 &feat); 
std::istream& operator>> (std::istream& ifs, FeatureRegressor2 &feat); 


cv::FileStorage& operator << (cv::FileStorage &fs, const FeatureRegressor2 &reg); 
void operator >> (const cv::FileNode &fs, FeatureRegressor2 &reg); 

#endif

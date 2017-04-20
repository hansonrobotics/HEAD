/*
 *  GentleBoostClassifier.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/26/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef GENTLEBOOSTCLASSIFIER_H
#define GENTLEBOOSTCLASSIFIER_H

#include <opencv2/core/core_c.h>
#include <iostream>
#include <vector>
#include "FeatureRegressor.h"
#include "ImagePatch.h"
#include "PatchList.h" 


/**
 * \ingroup MPGroup
 * \brief <tt> <b> Machine Perception Primitive: </b> </tt> An implementation of
 * a GentleBoost classifier for image patch classification. 
 *
 * The GentleBoost approach is described in Fasel's "Learning Real-Time Object 
 * Detectors: Probabilistic Generative Approaches", 2006 (see \ref bib_sec).
 *
 * GentleBoostClassifier is only suitable for whole patch classification. It is
 * not meant to be used to search for objects in scenes. For that application, 
 * see GentleBoostCascadedClassifier. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class GentleBoostClassifier {	
public:
	/**
	 * \brief Constructor.
	 */
	GentleBoostClassifier(); 
	
	/**
	 * \brief Destructor.
	 */
	virtual ~GentleBoostClassifier(); 
	
	/**
	 * \brief Evaluate every pixel in an image for its probability that
	 * an object is located there. 
	 *
	 * Since GentleBoostClassifier is not a 
	 * cascade, this process is somewhat slow. Also, GentleBoostClassifier has
	 * no notion of "rejection," so it cannot "find" patches containing the
	 * object. It can only give a patch-by-patch estimate of the likelihood that
	 * that patch is the object. 
	 *
	 * @param patches A PatchList that has had setImage() called, and also 
	 * resetListToScale(). Only the current search scale is evaluated.
	 * 
	 * @return An image with size patches->getImageSizeAtScale(), containing
	 * probability estimates. Each pixel represents the probability that the
	 * image patch with its top-left pixel at that probability map location
	 * contains an object.
	 */
	virtual IplImage* getProbabilityMap(PatchList* patches); 
	
	
	/**
	 * \brief Apply the GentleBoostClasssifier to a classify collection of image
	 * patches. 
	 *
	 * This classification requires a lot of scratch memory, which you can 
	 * provide. The behavior of this function depends on which provided 
	 * pointers are NULL. For example, perf and weights are only computed
	 * if labels is not NULL, whereas featureSum, posterior, and predictions
	 * are always computed. If any matrices that are required are NULL or 
	 * incorrectly sized, they will be (re)allocated, and the caller of the
	 * function is responsible for managing that memory. 
	 *
	 * @param patches A list of patches to classify.
	 * @param labels Optional labels for the patches, to evaluate classifier
	 * performance. 
	 * @param featureSum The accumulated feature output that is used to 
	 * predict the label of the patch.
	 * @param posterior The probability estimate that the patch was generated
	 * by the trained class.
	 * @param predictions Binary (+1/-1) classification labels applied by the classifier.
	 * @param featureOutputs The output of each individual FeatureRegressor
	 * applied to each patch.
	 * @param perf Chi-Squared error in predicting labels (only set if labels
	 * are provided). 
	 * @param weights Weights based on boosting that can be used for training
	 * the next feature.
	 */
	void searchPatches(const std::vector<ImagePatch*>& patches,
					   const CvMat* labels,
					   CvMat*& featureSum,
					   CvMat*& posterior,
					   CvMat*& predictions,
					   std::vector<CvMat*>& featureOutputs,
					   double& perf,
					   CvMat*& weights); 
	
	/**
	 * \brief Set ImagePatch data and labels used for training the classifier.
	 * 
	 * @param trainingPatches A collection of positive and negative examples
	 * that the classifier should learn to discriminate.
	 *
	 * @param trainingLabels Binary (+1/-1) indicating the label of each patch.
	 * This should be a matrix of type CV_64FC1 with size numPatches x 1.
	 */
	virtual void setTrainingSet(const std::vector<ImagePatch*>& trainingPatches,
								const CvMat* trainingLabels); 
	
	/**
	 * \brief Set ImagePatch data and labels used for evaluating the classifier.
	 * 
	 * @param testingPatches A collection of positive and negative examples
	 * that the classifier's discrimination will be evaluated on.
	 *
	 * @param testingLabels Binary (+1/-1) indicating the label of each patch.
	 * This should be a matrix of type CV_64FC1 with size numPatches x 1.
	 */
	virtual void setTestingSet(const std::vector<ImagePatch*>& testingPatches,
							   const CvMat* testingLabels); 
	
	/**
	 * \brief Add a feature to the classifier. 
	 *
	 * This will create a 
	 * FeatureRegressor and compute its tuning curve based on the current
	 * weighting of the training examples, and then reweight the training
	 * examples according to the GentleBoost algorithm. 
	 * 
	 * @param nextFeature The feature to add.
	 */
	virtual void addFeature(Feature* nextFeature); 
	
	/**
	 * \brief Compute perfomance measures for this classifier, depending on
	 * the Feature* passed in. This is primarily used for finding the best
	 * feature to add to the classifier, using addFeature().
	 * 
	 * getPerformanceMeasures has three modes depending on the candidate feature: 
	 * \li If the feature is NULL, calculates the current chi-squared error
	 * \li If the feature is currently in the classifier, calculates the error if
	 *   we remove the feature
	 * \li Otherwise, calculates the error if we were to add this feature.
	 *
	 * @param candidate Can be NULL, one of the features in the classifier, or
	 * a candidate feature to add to the classifier. 
	 *
	 * @param chiSq The computed error measure.
	 */
	void getPerformanceMeasures(Feature* candidate, double& chiSq); 
	
	/**
	 * \brief How many of its total features is the classifier currently using?
	 * To improve speed or avoid generalization errors, it may be useful to only
	 * use the first few features of the classifier. 
	 *
	 * @return Number of features currently being used.
	 */
	int getNumFeaturesUsed() ; 
	
	/**
	 * \brief How many total available, trained features does the classifier 
	 * have?
	 *
	 * @return Number of features trained and available for use.
	 */
	int getNumFeaturesTotal();  
	
	/**
	 * \brief Number of trained and available features to use.
	 * To improve speed or avoid generalization errors, it may be useful to only
	 * use the first few features of the classifier. 
	 *
	 * @param num Number of features to use. Should be less than or equal to
	 * getNumFeaturesTotal(). 
	 */
	void setNumFeaturesUsed(int num); 
	
	/**
	 * \brief Examine the features used by this GentleBoostClassifier.
	 *
	 * @param num Index of the feature to query, numbered 0 to 
	 * getNumFeaturesTotal()-1.
	 *
	 * @return A FeatureRegressor, which has the tuning curve as well as the
	 * feature.
	 */
	FeatureRegressor* getFeatureAndTuningCurveNumber(int num); 
	
	/**
	 * \brief Size of patches used for training. All features added must have
	 * a size equal to getBasePatchSize(). 
	 *
	 * getBasePatchSize() is set implicitly by the first feature added.
	 * 
	 * @return Size of the features used by this this classifier.
	 */
	cv::Size getBasePatchSize(); 
	
	/**
	 * \brief Write to a file.
	 */
	friend std::ostream& operator<< (std::ostream& ofs, GentleBoostClassifier* feat); 
	
	/**
	 * \brief Read from a file.
	 */
	friend std::istream& operator>> (std::istream& ifs, GentleBoostClassifier*& feat); 
	
protected: 
	std::vector<ImagePatch*> trainingPatches;
	CvMat* trainingLabels; 
	std::vector<CvMat*> trainingFeatureOutputs; 
	CvMat* trainingFeatureSum; 
	CvMat* trainingPosteriors; 
	CvMat* trainingPredictions;
	CvMat* trainingWeights; 
	double trainingPerformance; 
	int numTrain; 
	
	std::vector<ImagePatch*> testingPatches; 
	CvMat* testingLabels; 
	std::vector<CvMat*> testingFeatureOutputs; 
	CvMat* testingFeatureSum; 
	CvMat* testingPosteriors; 
	CvMat* testingPredictions; 
	double testingPerformance; 
	int numTest; 
	
	std::vector<FeatureRegressor*> features; 
	int numFeatures; 
	
	cv::Size basePatchSize; 
	
	const static int numBins = 100; 
	
	virtual void readFromStream(std::istream& in); 
	virtual void addToStream(std::ostream& out); 
	
};


std::ostream& operator<< (std::ostream& ofs, GentleBoostClassifier *booster); 
std::istream& operator>> (std::istream& ifs, GentleBoostClassifier *&booster); 

#endif

/*
 *  GentleBoostClassifier22.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/21/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */



#ifndef GENTLEBOOSTCLASSIFIER2_H
#define GENTLEBOOSTCLASSIFIER2_H

#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include "StructTypes.h" 
#include "FeatureRegressor2.h"
#include "ImagePatch2.h"
#include "PatchList2.h" 
#include "ImageDataSet2.h"
#include "PatchDataset2.h"


/**
 * \ingroup MPGroup
 * \brief <tt> <b> Machine Perception Primitive: </b> </tt> An implementation of
 * a GentleBoost classifier for image patch classification. 
 *
 * The GentleBoost approach is described in Fasel's "Learning Real-Time Object 
 * Detectors: Probabilistic Generative Approaches", 2006 (see \ref bib_sec).
 *
 * GentleBoostClassifier2 is only suitable for whole patch classification. It is
 * not meant to be used to search for objects in scenes. For that application, 
 * see GentleBoostCascadedClassifier. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class GentleBoostClassifier2 {	
public:
	/**
	 * \brief Constructor.
	 */
	GentleBoostClassifier2(); 
	
	/**
	 * \brief Copy Constructor
	 **/
	GentleBoostClassifier2(const GentleBoostClassifier2 &copy); 
	
	/**
	 * \brief Assignment operator
	 **/
	GentleBoostClassifier2 & operator=(const GentleBoostClassifier2 &rhs); 
	
	/**
	 * \brief Destructor.
	 */
	virtual ~GentleBoostClassifier2(); 
	
	/**
	 * \brief Evaluate every pixel in an image for its probability that
	 * an object is located there. 
	 *
	 * Since GentleBoostClassifier2 is not a 
	 * cascade, this process is somewhat slow. Also, GentleBoostClassifier2 has
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
	void getProbabilityMap(PatchList2* patches, cv::Mat &dest) const ;
	
	
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
	virtual void searchPatches(const std::vector<ImagePatch2> &patches,
							   const cv::Mat &labels,
							   cv::Mat &featureSum,
							   cv::Mat &posterior,
							   cv::Mat &predictions,
							   std::vector<cv::Mat> &featureOutputs,
							   double& perf,
							   cv::Mat &weights, 
							   cv::Mat &survived) const; 
	
	virtual void setTrainingSet(const PatchDataset2 &dataset); 
	
	virtual void setTestingSet(const PatchDataset2 &dataset); 
	
	/**
	 * \brief Set ImagePatch data and labels used for training the classifier.
	 * 
	 * @param trainingPatches A collection of positive and negative examples
	 * that the classifier should learn to discriminate.
	 *
	 * @param trainingLabels Binary (+1/-1) indicating the label of each patch.
	 * This should be a matrix of type CV_64FC1 with size numPatches x 1.
	 */
	virtual void setTrainingSet(const std::vector<ImagePatch2>& trainingPatches,
								const cv::Mat &trainingLabels); 
	
	/**
	 * \brief Set ImagePatch data and labels used for evaluating the classifier.
	 * 
	 * @param testingPatches A collection of positive and negative examples
	 * that the classifier's discrimination will be evaluated on.
	 *
	 * @param testingLabels Binary (+1/-1) indicating the label of each patch.
	 * This should be a matrix of type CV_64FC1 with size numPatches x 1.
	 */
	virtual void setTestingSet(const std::vector<ImagePatch2>& testingPatches,
							   const cv::Mat &testingLabels); 
	
	/**
	 * \brief Train boosted classifier for one round by searching for one
	 * good feature, and adding it. 
	 *
	 * @param patience How long do you want to wait to find a good feature?
	 *
	 * @param boostRounds Train for multiple rounds of boosting on one feature:
	 * makes the classifier more discriminative but more prone to overfit. 
	 **/ 
	virtual PerformanceMetrics trainOneRound(int patience = 1, int boostRounds = 1); 
	
	/**
	 * \brief Look for a feature that would improve classifier performance. 
	 *
	 * @param patience How long do you want to wait to find a good feature?
	 *
	 * @param featureType Type of feature to look for.
	 *
	 */
	virtual Feature2* getGoodFeature(int patience = 1, const std::string &featureType="HaarFeature") const; 
	
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
	virtual void addFeature(const Feature2* nextFeature); 
	
	/**
	 * \brief Add a feature to the classifier by applying multiple rounds of 
	 * boosting. 
	 *
	 * This allows a single feature to have more discriminative power, with the
	 * danger of overfitting. 
	 *
	 * The classification result is equivalent to calling addFeature() multiple 
	 * times on the same feature, but leads to greater computational efficiency 
	 * of the overall classifier. 
	 * 
	 * @param nextFeature The feature to add.
	 * @param boostRounds Number of times to (effectively) call addFeature on
	 * this feature. 
	 */
	virtual void addFeatureBoosted(const Feature2* nextFeature, int boostRounds=1); 
	
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
	virtual void getPerformanceMeasures(const Feature2* candidate, PerformanceMetrics& perf) const; 
	
	/**
	 * \brief Total number of negative patches currently being used to train the 
	 * classifier. 
	 */
	virtual int getNumTrainingPatches() const; 
	
	/**
	 * \brief Total number of positive patches currently being used to train the 
	 * classifier. 
	 */
	virtual int getNumTestingPatches() const; 
	
	/**
	 * \brief Get an IplImage for visualizing the negative patches in the 
	 * classifier - This must be released using cvReleaseImageHeader, and not
	 * cvReleaseImage, or you will destroy the data used for training. 
	 *
	 * This is useful for, for example, seeing what negative
	 * examples are currently confusing the classifier. 
	 * 
	 * @param patchNum Must be between 0 and getNumNegPatches()
	 * 
	 * @return An IplImage pointing to the actual training data. Memory 
	 * must be managed by calling cvReleaseImageHeader, rather than 
	 * cvReleaseImage(). 
	 */
	virtual void getImageHeaderForTrainingPatch(cv::Mat & dst, int patchNum) const; 
	
	/**
	 * \brief Get an IplImage for visualizing the positive patches in the 
	 * classifier - This must be released using cvReleaseImageHeader, and not
	 * cvReleaseImage, or you will destroy the data used for training. 
	 *
	 * This is useful for, for example, seeing what positive
	 * examples are currently confusing the classifier. 
	 * 
	 * @param patchNum Must be between 0 and getNumPosPatches()
	 * 
	 * @return An IplImage pointing to the actual training data. Memory 
	 * must be managed by calling cvReleaseImageHeader, rather than 
	 * cvReleaseImage(). 
	 */
	virtual void getImageHeaderForTestingPatch(cv::Mat &dst, int patchNum) const; 
	
	int getNumPosPatches() const ;
	int getNumNegPatches() const ;
	virtual void getImageHeaderForPosPatch(cv::Mat &dst, int patchNum) const; 
	virtual void getImageHeaderForNegPatch(cv::Mat &dst, int patchNum) const; 

	/**
	 * \brief How many of its total features is the classifier currently using?
	 * To improve speed or avoid generalization errors, it may be useful to only
	 * use the first few features of the classifier. 
	 *
	 * @return Number of features currently being used.
	 */
	int getNumFeaturesUsed() const ; 
	
	/**
	 * \brief How many total available, trained features does the classifier 
	 * have?
	 *
	 * @return Number of features trained and available for use.
	 */
	int getNumFeaturesTotal() const;  
	
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
	 * \brief Examine the features used by this GentleBoostClassifier2.
	 *
	 * @param num Index of the feature to query, numbered 0 to 
	 * getNumFeaturesTotal()-1.
	 *
	 * @return A FeatureRegressor, which has the tuning curve as well as the
	 * feature.
	 */
	FeatureRegressor2 getFeatureAndTuningCurveNumber(int num) const; 
	
	/**
	 * \brief Size of patches used for training. All features added must have
	 * a size equal to getBasePatchSize(). 
	 *
	 * getBasePatchSize() is set implicitly by the first feature added.
	 * 
	 * @return Size of the features used by this this classifier.
	 */
	cv::Size getBasePatchSize() const; 
	
	/**
	 * \brief Change the type of feature added by "train"
	 **/ 
	void setTrainingFeatureType(const std::string &featureType); 
	
	
	void setSearchParams(cv::Size minSize = cv::Size(0,0), cv::Size maxSize=cv::Size(0,0), 
						 double scaleInc=1.2, double stepWidth=1, int scaleStepWidth=1); 
	
	
	/**
	 * \brief Search through and image for objects that this classifier was
	 * trained to detect. Results are passed back via the SearchResult vector
	 * keptPatches. 
	 *
	 * @param gray_image Image or Frame to search. Must be single-channel, with
	 * depth IPL_DEPTH_8U. 
	 * @param keptPatches Results of the search are recorded in this vector. 
	 * This is faster than returning a vector. 
	 * @param NMSRadius Suppress object detection results if there is a 
	 * another SearchResult with higher value nearby (within radius NMSRadius
	 * pixels). 
	 * @param thresh Suppress object detection results with a value lower than
	 * thresh. Using -INFINITY returns all results. Using thresh=0 returns
	 * only SearchResult records that the cascade has at least 50% confidence 
	 * in. 
	 */
	void searchImage(const cv::Mat &gray_image, 
					 std::vector<SearchResult>& keptPatches, 
					 int NMSRadius=0, 
					 double thresh = -INFINITY) ;
	/**
	 * \brief Set the current image to search, without actually searching. This
	 * allows you to search each scale (size) separately and manually, using
	 * searchCurrentImageAtScale(). 
	 * 
	 * @param gray_image Image or Frame to search. Must be single-channel, with
	 * depth IPL_DEPTH_8U. 
	 */
	void setCurrentImage(const cv::Mat &gray_image); 
	
	/**
	 * \brief Search through and image for objects at a single scale (size).
	 *
	 * @param keptPatches Results of the search are recorded in this vector. 
	 * This is faster than returning a vector. 
	 * @param scale Scale to search at. Must be between 0 and getNumScales()-1.
	 * @param NMSRadius Suppress object detection results if there is a 
	 * another SearchResult with higher value nearby (within radius NMSRadius
	 * pixels). 
	 * @param thresh Suppress object detection results with a value lower than
	 * thresh. Using -INFINITY returns all results. Using thresh=0 returns
	 * only SearchResult records that the cascade has at least 50% confidence 
	 * in. 
	 */
	void searchCurrentImageAtScale(std::vector<SearchResult>& keptPatches, 
								   int scale, 
								   int NMSRadius=0, 
								   double thresh = -INFINITY);
	
	/**
	 * \brief Query the number of scales available for searching for objects.
	 * See PatchList for further discussion. 
	 *
	 * @return Number of scales available for searching objects.
	 */
	int getNumScales() const; 
	
	/**
	 * \brief Query the size of object searched for at a given scale. 
	 * See PatchList for further discussion. 
	 *
	 * @param scale Search scale. 
	 *
	 * @return Patch size (object size) searched at that scale. 
	 */
	cv::Size getSizeOfScale(int scale) const;
		
	/**
	 * \brief Share the patch list of another GentleBoostCascadedClassifier, to
	 * increase efficiency of multiple classifiers processing the same image. 
	 *
	 * By sharing a patch list, calling "setImage" on one classifier will set it
	 * for all of them. Thus different classifiers can share processing on the 
	 * same image data. 
	 * 
	 * However, be careful with this. It cannot be undone (within the lifetime
	 * of a single object -- reloading the classifier from disk will undo it). 
	 *
	 * @param otherClassifier Classifier to share a PatchList with.
	 */
	void sharePatchListWithClassifier(const GentleBoostClassifier2 &otherClassifier); 
	
	/**
	 * \brief Write to a file.
	 */
	friend cv::FileStorage& operator << (cv::FileStorage &fs, const GentleBoostClassifier2 &booster); 
	
	/**
	 * \brief Read from a file.
	 */
	friend void operator >> ( const cv::FileNode &fs, GentleBoostClassifier2 &booster); 
	
	/**
	 * \brief Cost function used to compare features: By default, only uses chisq
	 * error info.
	 **/
	static double featureCost(const PerformanceMetrics& a) ; 
	
	
protected: 
	
	virtual void copy(const GentleBoostClassifier2 &copy); 
	
	virtual void accumulateEvidence(const cv::Mat &output, cv::Mat &featureSum, 
									const cv::Mat &survived=cv::Mat()) const; 
	virtual void updateWeights(const cv::Mat &output, const cv::Mat &labels, cv::Mat &weights,
							   const cv::Mat &survived=cv::Mat()) const; 
	virtual void calcPosterior(const cv::Mat &featureSum, cv::Mat &posterior) const; 
	virtual void makePredictions(const cv::Mat &featureSum, cv::Mat &predictions,
								 const cv::Mat &survived=cv::Mat()) const;
	virtual double getFractionCorrect(const cv::Mat &labels, const cv::Mat &predictions) const; 
	virtual double getChiSq(const cv::Mat &labels, const cv::Mat &posterior) const; 
	virtual void updateSurvivedList(const cv::Mat &featureSum, double threshold,
									cv::Mat &survived) const;
	
	virtual void updateValuesForFeature(const FeatureRegressor2  &feature,
										const std::vector<ImagePatch2>& patches,
										const cv::Mat &labels,
										cv::Mat &featureSum,
										cv::Mat &posterior,
										cv::Mat &predictions,
										cv::Mat &featureOutputs,
										double& perf,
										cv::Mat &weights,
										double threshold,
										cv::Mat &survived) const; 
	
	Feature2* getGoodFeatureViaTournament(const std::string &featureType,
										 int startPoolSize, 
										 int similarFeatures,
										 double keepPortion) const; 
	
	void compareFeaturesAndDeleteWorse(Feature2*& oldFeat1NewBetterFeat, 
									   Feature2*& oldFeat2NewDeletedFeat,
									   PerformanceMetrics& oldPerf1NewBetterPerf) const; 
	
	virtual void printDebugState() const; 
	
	virtual void pickRejectThreshold(const cv::Mat &output,
									 const cv::Mat &labels,
									 const cv::Mat &survived,
									 double& threshold,
									 int& totalPosRejects,
									 int& totalNegRejects) const;
	
	void searchImage(const cv::Mat &gray_image, 
					 std::vector<SearchResult>& keptPatches, 
					 int NMSRadius, 
					 double threshold,
					 std::vector<cv::Rect> blacklistPatches,
					 int spatialRadius=0, 
					 int scaleRadius=0);
	
	
	void searchCurrentImageAtScale(std::vector<SearchResult>& keptPatches, 
								   int scale, 
								   int NMSRadius, 
								   double threshold,
								   std::vector<cv::Rect> blacklistPatches,
								   int spatialRadius=0, 
								   int scaleRadius=0);
	
	
	void suppressLocalNonMaximaAcrossScales(std::vector<SearchResult>& keptPatches, 
											int NMSRadius, 
											std::vector<cv::Point>& centers, 
											std::vector<size_t>& nextScaleStart) const ; 
	 
	virtual void writeToFileStorage(cv::FileStorage &fs) const; 
	virtual void readFromFileStorage(const cv::FileNode &fs);  
	
	void setBasePatchSize(cv::Size s); 
	void setUseFastPatchList(int yesorno); 
	int updateSurvivalStats() ; 
	
	//Persistant variables
	std::vector<FeatureRegressor2> features; 
	int numFeatures; 
	cv::Size basePatchSize; 
	std::string featureName; 
	std::vector<double> featureRejectThresholds; 
	int useFast; 
	
	//Volatile variables
	std::vector<ImagePatch2> trainingPatches; 
	std::vector<ImagePatch2*> posPatches, negPatches;
	cv::Mat trainingLabels; 
	std::vector<cv::Mat> trainingFeatureOutputs; 
	cv::Mat trainingFeatureSum, trainingPosteriors, trainingPredictions, trainingWeights; 
	std::vector<bool> posSurvived; 
	std::vector<bool> negSurvived; 
	double trainingPerformance; 
	int numTrain; 
	
	std::vector<ImagePatch2> testingPatches; 
	cv::Mat testingLabels; 
	std::vector<cv::Mat> testingFeatureOutputs; 
	cv::Mat testingFeatureSum, testingPosteriors, testingPredictions; 
	double testingPerformance; 
	int numTest; 
	
	cv::Mat trainingSurvived, testingSurvived; 
	
	ImageDataSet2 posImagesDataset; 
	ImageDataSet2 negImagesDataset; 
	PatchDataset2 patchDataset; 
	
	PatchList2 pl; 
	FastPatchList2 fpl; 
	PatchList2* patchList; 
		
	const static int numBins = 100; 
		
	bool sharingPatchList; 
	
	
	bool useNMSInTraining; 
	bool keepNonRejectedBGPatches; 
	bool dontTrainRejected; 
	bool runningOutOfNegPatches;
	bool ranOutOfNegPatches; 
	bool disableNMSAcrossScales; 
	int currentBGFileNum; 
	unsigned int maxPatchesPerImage; 
	
	
	
};



cv::FileStorage& operator << (cv::FileStorage &fs, const GentleBoostClassifier2 &booster); 
void operator >> ( const cv::FileNode &fs, GentleBoostClassifier2 &booster); 

#endif

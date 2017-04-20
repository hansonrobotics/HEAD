/*
 *  GentleBoostCascadedClassifier.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/26/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef GENTLEBOOSTCASCADEDCLASSIFIER_H
#define GENTLEBOOSTCASCADEDCLASSIFIER_H

#include <opencv2/core/core_c.h>
#include <iostream>
#include <vector>
#include <string>
#include "ImagePatch.h" 
#include "ImageDataSet.h"
#include "GentleBoostClassifier.h"
#include "FastPatchList.h"
#include "PatchList.h"
#include "PatchDataset.h"
#include "StructTypes.h"

/**
 * \ingroup MPGroup
 * \brief <tt> <b> Machine Perception Primitive: </b> </tt> An implementation of
 * a GentleBoost cascaded classifier for full image object search. 
 *
 * The GentleBoost approach is described in Fasel's "Learning Real-Time Object 
 * Detectors: Probabilistic Generative Approaches", 2006 (see \ref bib_sec).
 *
 * GentleBoostCascadedClassifier is designed for full frame object
 * detection.
 *
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */
class GentleBoostCascadedClassifier : public GentleBoostClassifier {
public:
	
	/**
	 * \brief Constructor.
	 */
	GentleBoostCascadedClassifier(); 
		
	/**
	 * \brief Destructor.
	 */
	~GentleBoostCascadedClassifier(); 
	
	/**
	 * \brief Set parameters affecting how GentleBoostCascadedClassifier 
	 * searches through images for objects. 
	 * 
	 * These are all parameters that are
	 * passed on to a PatchList object. See PatchList documentation for further 
	 * details. 
	 * 
	 * @param useFast Use "FastPatchList" rather than "PatchList" for searching.
	 * This improves speed, but can significantly hamper detector performance.
	 * Using useFast=0 is highly recommended. 
	 * @param minSize Don't search for objects smaller than minSize. If minSize
	 * is (0,0) then baseObjectSize is used. 
	 * @param maxSize Don't search for objects larger than maxSize. If maxSize
	 * is (0,0), then the upper bound on the object size is the smallest 
	 * dimension of the image size. 
	 * @param scaleInc Relative size of the object in the next scale, compared
	 * to the current one. scaleInc must be greater than 1. 
	 * @param stepWidth Search interval between subsequent patches. A stepWidth
	 * of 1 means slide the object evaluation window over one pixel. 2 means 
	 * only evaluate patches at every other pixel. 1.5 means search two and 
	 * skip the third, etc.
	 * @param scaleStepWidth Only applies if useFast=1.
	 * A boolean (0 or non-zero) value, indicating whether
	 * the stepWidth should scale up with the patch size. Setting 1 gives the
	 * same relative coverage to all scales, setting to 0 gives relatively 
	 * finer coverage to larger scales, and searching will take considerably
	 * more time.
	 */
	void setSearchParams(int useFast=0, cv::Size minSize = cv::Size(0,0), cv::Size maxSize=cv::Size(0,0), 
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
					 double thresh = -INFINITY);
	
	DEPRECATED(void searchImage(IplImage* gray_image,  std::vector<SearchResult>& keptPatches,  int NMSRadius=0,  double thresh = -INFINITY));
	
	/**
	 * \brief Set the current image to search, without actually searching. This
	 * allows you to search each scale (size) separately and manually, using
	 * searchCurrentImageAtScale(). 
	 * 
	 * @param gray_image Image or Frame to search. Must be single-channel, with
	 * depth IPL_DEPTH_8U. 
	 */
	void setCurrentImage(const cv::Mat &gray_image); 
	DEPRECATED(void setCurrentImage(IplImage* gray_image)); 
	
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
	int getNumScales(); 
	
	/**
	 * \brief Query the size of object searched for at a given scale. 
	 * See PatchList for further discussion. 
	 *
	 * @param scale Search scale. 
	 *
	 * @return Patch size (object size) searched at that scale. 
	 */
	cv::Size getSizeOfScale(int scale);
	
	/**
	 * \brief Apply the GentleBoostCascadedClasssifier to a classify collection 
	 * of image patches. 
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
	 * @param survived Indicates whether or not a patch was rejected before
	 * classification finished. If rejected, it is automatically given the
	 * label prediction -1, regardless of the posterior. 
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
					   const cv::Mat &labels,
					   cv::Mat &survived,
					   cv::Mat &featureSum,
					   cv::Mat &posterior,
					   cv::Mat &predictions,
					   std::vector<cv::Mat> &featureOutputs,
					   double& perf,
					   cv::Mat &weights); 
	
	
	
	/**
	 * \brief Set parameters used to determine the rejection threshold for
	 * each cascade step. 
	 *
	 * The threshold is chosen as soon as more than a fraction maxPosRejects of
	 * the remaining positive patches have been rejected, or when a fraction
	 * desiredNegRejects of the remaining negative patches have been rejected.
	 * 
	 * @param maxPosRejects Max fraction of remaining positive patches rejected 
	 * per training round.
	 * @param desiredNegRejects Desired fraction of remaining negative patches 
	 * rejected per training round.
	 */  
	void setTrainingParams(double maxPosRejects = 0.001, double desiredNegRejects=1); 
	
	/**
	 * \brief Train boosted classifier for one round by searching for one
	 * good feature, and adding it. 
	 *
	 * @param patience How long do you want to wait to find a good feature?
	 *
	 * @param boostRounds Train for multiple rounds of boosting on one feature:
	 * makes the classifier more discriminative but more prone to overfit. 
	 **/ 
	virtual PerformanceMetrics trainOneRound(int patience = 1, int boostRounds = 0); 
	
	/**
	 * \brief Look for a feature that would improve classifier performance. 
	 *
	 * @param patience How long do you want to wait to find a good feature?
	 *
	 * @param featureType Type of feature to look for.
	 *
	 */
	virtual Feature* getGoodFeature(int patience = 1, std::string featureType="HaarFeature"); 
	
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
	
	virtual void setTrainingSet(std::string pathToPatchDatasetFile); 
	
	virtual void setTestingSet(std::string pathToPatchDatasetFile); 
	
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
								const cv::Mat &trainingLabels); 
	DEPRECATED(virtual void setTrainingSet(const std::vector<ImagePatch*>& trainingPatches, const CvMat* trainingLabels)); 
	
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
							   const cv::Mat &testingLabels); 
	DEPRECATED(virtual void setTestingSet(const std::vector<ImagePatch*>& testingPatches,  const CvMat* testingLabels)); 
	
	
	/**
	 * \brief Specify a large collection of images known not have the object
	 * you are learning a detector for. These will serve as a source of 
	 * training data. Only a portion of these will be used for 
	 * training at a time. Call setHardNegativeExamplesFromBGImages() to 
	 * swap out patches currently used for training ones for harder ones from
	 * the background image pool. 
	 * 
	 * @param datasetFileName Path to a file containing the paths to images (one per
	 * line) known to not contain the detector target. 
	 */
	void setBGTrainingFromImageDataset(const std::string &datasetFileName);
	
	/**
	 * \brief Replace the rejected negative example patches in the current 
	 * training set with patches in taken from background images that haven't
	 * yet been rejected. The background image pool is specified with 
	 * setBGTrainingImagesFromImageDataset().
	 */
	void setHardNegativeTrainingExamplesFromBGImages(); 
	
	
	/**
	 * \brief Replace the rejected negative example patches in the current 
	 * training set with patches in taken from a pool of background patches
	 * yet been rejected. The negative patch pool is specified with 
	 * setBGTrainingPatches(). 
	 */
	void setHardNegativeTrainingExamplesFromPatches(); 
	
	
	/**
	 * \brief Returns "yes" if all known negative patches have been rejected.
	 * This is a good time to stop training. 
	 */
	bool exhaustedAllNegPatches(); 
	
	
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
	void addFeatureBoosted(Feature* nextFeature, int boostRounds); 
	
	/**
	 * \brief Evaluate every pixel in an image for its probability that
	 * an object is located there. 
	 *
	 * @param patches A PatchList that has had setImage() called, and also 
	 * resetListToScale(). Only the current search scale is evaluated.
	 * 
	 * @return An image with size patches->getImageSizeAtScale(), containing
	 * probability estimates. Each pixel represents the probability that the
	 * image patch with its top-left pixel at that probability map location
	 * contains an object.
	 */
	virtual void getProbabilityMap(cv::Mat &dest, PatchList* patches); 
	DEPRECATED(virtual IplImage* getProbabilityMap(PatchList* patches)); 
	
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
	 * @param chiSq The computed error measure (lower is better). 
	 * @param posRejects The number of positive examples rejected by adding this
	 * feature (lower is better). 
	 * @param negRejects The number of negative examples rejected by adding this
	 * feature (higher is better). 
	 */
	void getPerformanceMeasures(Feature* candidate, double& chiSq, int& posRejects, int& negRejects); 
	
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
	 * @param perf Metrics of performance. See PerformanceMetrics documentation
	 * for further reference. 
	 */
	void getPerformanceMeasures(const Feature* candidate, PerformanceMetrics& perf); 	
	
	/**
	 * \brief Total number of negative patches currently being used to train the 
	 * classifier. 
	 */
	int getNumNegPatches(); 
		
	/**
	 * \brief Total number of positive patches currently being used to train the 
	 * classifier. 
	 */
	int getNumPosPatches(); 
	
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
	void getImageHeaderForNegPatch(cv::Mat & dst, int patchNum); 
	DEPRECATED(IplImage* getImageHeaderForNegPatch(int patchNum)); 
	
	
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
	void getImageHeaderForPosPatch(cv::Mat &dst, int patchNum); 
	DEPRECATED(IplImage* getImageHeaderForPosPatch(int patchNum)); 
	
	
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
	void sharePatchListWithClassifier(GentleBoostCascadedClassifier* otherClassifier); 
	
	/**
	 * \brief Write to a file.
	 */
	friend std::ostream& operator<< (std::ostream& ofs, GentleBoostCascadedClassifier* booster); 
	
	/**
	 * \brief Read from a file.
	 */
	friend std::istream& operator>> (std::istream& ifs, GentleBoostCascadedClassifier*& booster); 
	
	
	static double featureCost(const PerformanceMetrics& a); 
private: 
	CvMat* trainingSurvived; 
	CvMat* testingSurvived; 
	std::vector<double> featureRejectThresholds; 
	double maxPosRejectsPerRound; 
	double desiredNegRejectsPerRound; 
	
	ImageDataSet* posImageDataset; 
	ImageDataSet* negImageDataset; 
	PatchDataset* posPatchDataset; 
	
	bool sharingPatchList; 
	
	std::vector<ImagePatch*> negPatches; 
	std::vector<ImagePatch*> posPatches; 
	std::vector<bool> posSurvived; 
	std::vector<bool> negSurvived; 
	bool useNMSInTraining; 
	
	PatchList* patchlist; 
	int currentBGFileNum; 
	unsigned int maxPatchesPerImage; 
	bool keepNonRejectedBGPatches; 
	bool dontTrainRejected; 
	
	bool runningOutOfNegPatches;
	bool ranOutOfNegPatches; 
	bool disableNMSAcrossScales; 
	
	
	Feature* getGoodFeatureViaTournament(std::string featureType,
										 int startPoolSize, 
										 int similarFeatures,
										 double keepPortion = .5); 
	void compareFeaturesAndDeleteWorse(Feature*& oldFeat1NewBetterFeat, 
									   Feature*& oldFeat2NewDeletedFeat,
									   PerformanceMetrics& oldPerf1NewBetterPerf);
	
	
	void checkPatchListForMemoryLeaks(); 

	int updateSurvivalStats() ; 
	
	
	void searchImage(const cv::Mat &gray_image, 
					 std::vector<SearchResult>& keptPatches, 
					 int NMSRadius, 
					 double threshold,
					 std::vector<cv::Rect> blacklistPatches,
					 int spatialRadius=0, 
					 int scaleRadius=0);
	
	DEPRECATED(void searchImage(IplImage* gray_image,  std::vector<SearchResult>& keptPatches,  int NMSRadius,  double threshold, std::vector<cv::Rect> blacklistPatches, int spatialRadius=0, int scaleRadius=0));

	void searchCurrentImageAtScale(std::vector<SearchResult>& keptPatches, 
								   int scale, 
								   int NMSRadius, 
								   double threshold,
								   std::vector<cv::Rect> blacklistPatches,
								   int spatialRadius=0, 
								   int scaleRadius=0);
	
	
	void pickRejectThreshold(const CvMat* values,
							 const CvMat* labels,
							 const CvMat* survived,
							 double& threshold,
							 int& totalPosRejects,
							 int& totalNegRejects);
	
	void suppressLocalNonMaximaAcrossScales(std::vector<SearchResult>& keptPatches, 
										 int NMSRadius, 
										 std::vector<cv::Point>& centers, 
										 std::vector<unsigned int>& nextScaleStart); 
	
	virtual void readFromStream(std::istream& in); 
	virtual void addToStream(std::ostream& out); 
	
	//Make deprecated when easy to do so
	void searchPatches(const std::vector<ImagePatch*>& patches,  const CvMat* labels,  CvMat*& survived, CvMat*& featureSum,  CvMat*& posterior, CvMat*& predictions, std::vector<CvMat*>& featureOutputs, double& perf, CvMat*& weights); 
	
	void printState() ; 
	
};
std::ostream& operator<< (std::ostream& ofs, GentleBoostCascadedClassifier *booster); 
std::istream& operator>> (std::istream& ifs, GentleBoostCascadedClassifier *&booster); 

#endif

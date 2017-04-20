/*
 *  StructTypes.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 7/14/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _STRUCT_TYPES
#define _STRUCT_TYPES


#include <vector>
#include <opencv2/core/core.hpp>

class Feature;
class Feature2; 

/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> A data structure that collects statistics
 * about the performance of a GentleBoostCascadedClassifier on some ImageDataSet.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
struct EvaluationMetrics {
	/**
	 * \brief Number of objects that were known to be in a given image that the
	 * detector found. 
	 */
	double num_hits; 
	
	/**
	 * \brief Number of objects that were known to be in a given image that the
	 * detector did not find. 
	 */
	double num_misses; 
	
	/**
	 * \brief Number of objects reported by the detector which were not known
	 * to be objects in the image.
	 */
	double num_false_alarms; 
	
	/**
	 * \brief Ratio of hits to total number of known objects: hits/(hits+misses).
	 */
	double hit_ratio; 
	
	/**
	 * \brief Threshold on object detector confidence used to calculate the
	 * reported performance for a given image.
	 */
	double threshold; 
	
	/**
	 * \brief Locations in the image of hits.
	 */
	std::vector<cv::Rect> hits; 
	
	/**
	 * \brief Locations in the image of misses.
	 */
	std::vector<cv::Rect> misses; 
	
	/**
	 * \brief Locations in the image of false alarms.
	 */
	std::vector<cv::Rect> false_alarms; 
};


/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> A data structure that keeps track of 
 * the likelihood value for a particular parameter setting, and the gradient
 * of the value function with resepct to that parameter setting. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
struct likelihood {
	/**
	 * \brief The likelihood value.
	 **/
	double val; 
	
	/**
	 * \brief The likelihood gradient.
	 **/
	cv::Mat grad; 
};


/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> A data structure that keeps track of 
 * a vector-valued action, and a time at which that action was taken.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
struct actionRecord {
	/**
	 * \brief The action that was taken.
	 **/
	cv::Mat actionVals; 
	
	/**
	 * \brief The time at which the action was taken.
	 **/
	double timeStamp; 
}; 


/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> A data structure that keeps track of how
 * a classifier (specifically a cascaded classifier) is performing. This is 
 * useful for evaluating candidate features to add. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
struct PerformanceMetrics {
	/**
	 * \brief Chi-Squared Error of Classifier evaluated on training set. Lower
	 * is better.
	 */
	double chisq; 
	
	/**
	 * \brief Number of positive examples rejected by classifier -- lower is
	 * better. 
	 */ 
	int pos_rejects; 
	
	/**
	 * \brief Number of negative examples rejected by classifier -- higher is 
	 * better. 
	 */
	int neg_rejects; 
	
	/**
	 * \brief Total size of positive example pool. 
	 */
	int total_pos; 
	
	/**
	 * \brief Total size of negative example pool. 
	 */
	int total_neg; 
	
	/**
	 * \brief Number of positive examples rejected by the classifier before, 
	 * i.e. independent of the feature under consideration. 
	 */
	int prev_pos_rejects; 
	
	/**
	 * \brief Number of negative examples rejected by the classifier before, 
	 * i.e. independent of the feature under consideration. 
	 */
	int prev_neg_rejects; 
	
	/**
	 * \brief Time required to evaluate a patch with this feature. Lower is 
	 * better. 
	 */
	double time_per_patch; 
	
	/**
	 * \brief Rejection criterion used to generate these performance results.
	 */
	double threshold; 
};


/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> A data structure that keeps track of 
 * the a particular image feature setting, and performance of that feature.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
struct FeaturePerformance {
	/**
	 * \brief A feature being considered for an object detector.
	 **/
	Feature* feat; 
	
	/**
	 * \brief The performance achieved by that feature.
	 **/
	PerformanceMetrics perf; 
}; 


/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> A data structure that keeps track of 
 * the a particular image feature setting, and performance of that feature.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
struct FeaturePerformance2 {
	/**
	 * \brief A feature being considered for an object detector.
	 **/
	Feature2* feat; 
	
	/**
	 * \brief The performance achieved by that feature.
	 **/
	PerformanceMetrics perf; 
}; 

/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> A data structure that keeps track of where
 * objects were detected in the image, and how confident the classifier is in
 * those detections.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
struct SearchResult {
	/**
	 *\brief Rectangle believed to contain an object of interest.
	 */
	cv::Rect imageLocation; 
	
	/**
	 * \brief Confidence that the location contains an object of interest.
	 * There is a rough mapping between this value and a probability estimate.
	 * 
	 * Specifically, p(object) = 1/(1+exp(-2*value)), so if value is
	 * 0, p(object)=0.5, if it is 1, p(object)=0.88, and if it is -2, 
	 * p(object)=0.02. However, these probability estimates are quite rough.
	 */
	double value; 
	int _x; 
	int _y; 
	int _scale; 
}; 

bool operator<(const SearchResult& a, const SearchResult& b); 
bool operator<(const PerformanceMetrics& a, const PerformanceMetrics& b); 
bool operator<(const FeaturePerformance& a, const FeaturePerformance& b);
bool operator<(const FeaturePerformance2& a, const FeaturePerformance2& b);

/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> Set the cost function used to compare
 * features. By default GentleBoostCascadedClassifier::featureCost is used.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
void setFeatureCostFunction(double (*costFun)(const PerformanceMetrics& )) ; 

#endif
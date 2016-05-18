/*
 *  MultinomialObservationModel.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/8/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _MULTINOMIALOBSERVATIONMODEL_H
#define _MULTINOMIALOBSERVATIONMODEL_H

#include <opencv2/core/core_c.h>
#include <iostream>

/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> Multionomial Observation Model, as described in Butko and Movellan, 
 * CVPR 2009 (see \ref bib_sec). This data structure maintains the mapping between object detector output and the probability
 * that the target object is located at every grid-cell. 
 *
 * An observation model consists of grid-height*grid-width*2 multinomial distributions, each with 10 outcomes. 
 * The proportiaonal likelihood that the counts of object-detector firings in a single grid-cell was caused by
 * the object, instead of generated spontaneously, is p(count|object,xdist-to-fixation,ydist-to-fixation) / 
 * p(count|noobject,xdist-to-fixation,ydist-to-fixation). 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class MultinomialObservationModel {
public:
	/**
	 * \brief Constructor: Create an observation model with space allocated to hold parameters
	 * for a grid of size gridSize. The probabilities are set according to some simple heuristic.
	 **/
	MultinomialObservationModel(CvSize gridSize) ; 
	
	/**
	 * \brief Deep Copy Constructor: Create an observation model that is identical to 
	 * the one copied.
	 **/
	MultinomialObservationModel(MultinomialObservationModel* modelToCopy);
	
	
	/**
	 * \brief Default Destructor. 
	 * 
	 * Deallocates all memory associated with the haar detector. 
	 */
	virtual ~MultinomialObservationModel() ; 
	
	/** 
	 * \brief Given an observation (output of the object detector at each grid cell), estimate the 
	 * relative likelihood that each grid cell's output was caused by the object being 
	 * located there. The results of this computation are stored in the observationProbability 
	 * variable. 
	 *
	 * @param searchPoint The point that was the center of fixation when the object detector was 
	 * applied to the image.
	 *
	 * @param faceCount An "image" of counts with the size of the grid. This is the number of
	 * target objects detected with centers in each grid cell as a result of applying the 
	 * object detector. 
	 **/
	virtual void setObservationProbability(CvPoint searchPoint, IplImage* faceCount) ;
	
	/**
	 * \brief The results of "setObservationProbability" are stored in this variable. Each 
	 * location contains the probabilty of the entire observation vector if the object were
	 * located at that location (up to a factor that is constant for every element).  
	 **/
	IplImage* observationProbability; //has size of grid
	
	/**
	 *\brief Reset the experience-counts used to estimate the multinomial parameters.  Sets all counts
	 * to 1.
	 *
	 * The multinomial distribution probabilities are estimated from experience counts. The table of counts
	 * forms a dirichlet posterior over the parameters of the multinomials. When the counts are reset to 1,
	 * there is a flat prior over multinomial parameters, and all outcomes are seen as equally likely.
	 * This is a particularly bad model, and will make it impossible to figure out the location of the face,
	 * so only call resetCounts if you're then going to call updateProbTableCounts for enough images to 
	 * build up a good model. 
	 *
	 * In general, this functionality should be accessed indirectly, through the MIPOMDP resetModel()
	 * function. 
	 **/
	void resetCounts() ; 
	
	
	/**
	 *\brief Add evidence from one event to the count-tables that maintain the Dirichlet distribution 
	 * over the multinomial parameters. 
	 *
	 * Given a fixation point and a face location, the resulting object detector output forms a collection
	 * of random events. We add these outcomes to a table of counts of witnessed events, and build up
	 * a model of how likely each event is. 
	 *
	 * In general, this functionality should be accessed indirectly, through the MIPOMDP trainObservationModel()
	 * function or addDataToObservationModel() function. 
	 **/
	virtual void updateProbTableCounts(CvPoint searchPoint, IplImage* faceCount, CvRect faceLocation); 
	
	/**
	 *\brief Estimate the Maximum A Posterior multinomial parameters based on the events seen so far, 
	 * and use this as the probability distribution estimate in the future. 
	 *
	 * Simply updating the counts does not change the estimate of outcome probabilities. This only happens
	 * when normalizeProbTables() is called. 
	 *
	 * In general, this functionality should be accessed indirectly, through the MIPOMDP trainObservationModel()
	 * function or addDataToObservationModel() function. 
	 **/
	void normalizeProbTables() ; 
	
	/**
	 * \brief Combine the evidence from two models (merge their counts and subtract off the extra priors).
	 *
	 * @param otherModel A second MultinomialObservationModel that has been fit to different data than this one: 
	 * We can estimate the model for the combined set of data by simply adding the counts, and subtracting 
	 * duplicate priors. In this way, we can efficiently compose models fit to different subsets of a larger dataset
	 * (e.g. for cross-validation). 
	 *
	 * In general, this functionality should be accessed indirectly, through the MIPOMDP cobineModels() function. 
	 **/
	void combineEvidence(MultinomialObservationModel* otherModel);
	
	/**
	 * \brief Write the probabilities and counts to a file stream, so we can read them in later.
	 **/
	friend std::ostream& operator<< (std::ostream& ofs, MultinomialObservationModel* a);
	
	/**
	 * \brief Read the probabilities and counts from a file stream, so we don't lose our earlier 
	 * models. 
	 **/
	friend std::istream& operator>> (std::istream& ifs, MultinomialObservationModel* a);

protected:
	void fillProbTablesHeuristic() ; 
	int maxfaces; 
	
	
	void readFromStream(std::istream& in); 
	void addToStream(std::ostream& out); 
	
	
	CvMatND* countsObsGivenFace;   //Matrix i is a 3D matrix saying for |xdist| |ydist| |numFaces| at scale i, what is the
	//relative probability that a face generated numFaces responses at xdist/ydist from 
	//where we're looking.
	CvMatND* countsObsGivenNoFace; //Matrix i is a 3D matrix saying for |xdist| |ydist| |numFaces| at scale i, what is the
	//relative probability that a face generated numFaces responses at xdist/ydist from 
	//where we're looking.
	
	
	CvMatND* probObsGivenFace;   //Matrix i is a 3D matrix saying for |xdist| |ydist| |numFaces| at scale i, what is the
	//relative probability that a face generated numFaces responses at xdist/ydist from 
	//where we're looking.
	CvMatND* probObsGivenNoFace; //Matrix i is a 3D matrix saying for |xdist| |ydist| |numFaces| at scale i, what is the
	//relative probability that a face generated numFaces responses at xdist/ydist from 
	//where we're looking.
	
};

std::ostream& operator<< (std::ostream& ofs, MultinomialObservationModel* a) ; 
std::istream& operator>> (std::istream& ifs, MultinomialObservationModel* a) ; 

#endif

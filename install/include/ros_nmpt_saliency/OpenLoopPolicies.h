/*
 *  OpenLoopPolicies.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/9/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _OPENLOOPPOLICIES_H
#define _OPENLOOPPOLICIES_H

#include <opencv2/core/core_c.h>

/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A static class for some open-loop fixation strategies, which can be
 * used as points of comparison to more sophisticated search strategies. 
 *
 * Since the policies are all open loop, they can all be indexed by a point on a trajectory and so
 * no state is needed, hence the class is static. Fixation points can be generated via
 * the OpenLoopPolicies::getFixationPoint() function, which requires a fixation number (point on 
 * the trajectory), a policy type, and the size of the the visual field. 
 *
 * The following policies are implemented: 
 *
 * \li RANDOM: Pick the next point randomly. 
 * \li ORDERED: Scan from left-to-right, top-to-bottom
 * \li SPIRAL: Fixate in the order of minimum-distance-to-center.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class OpenLoopPolicies {
public:
	
	/**
	 * \brief Generate fixation point from the Random policy, picking the next point randomly.
	 **/
	const static int RANDOM = 0; 
	
	/**
	 * \brief Generate fixation point from the Ordered policy, scanning left-to-right, top-to-bottom.
	 **/
	const static int ORDERED = 1; 
	
	/**
	 * \brief Generate the next fixation as the next-closest-cell-to-center. As of the current 
	 * implementation, this does not yield a proper spiral, but some approximation. 
	 */
	const static int SPIRAL = 2; 
	
	/**
	 * \brief Get a fixation point on some open loop trajectory.
	 *
	 * @param fixationNumber The discrete point along the trajectory, indexed from 0. To
	 * get the next fixation point on the trajectory, this parameter should be incremented,
	 * keeping the other parameters fixed.
	 * @param policyType The specific trajectory to find a point on. Current choices are 
	 * OpenLoopPolicy::RANDOM, OpenLoopPolicy::ORDERED, OpenLoopPolicy::SPIRAL.
	 * @param gridSize The number of available locations that could be fixated.
	 *
	 * @return A point on the grid to fixate next.
	 **/
	static CvPoint getFixationPoint(int fixationNumber, int policyType, CvSize gridSize); 
	
	static float randomFloat(); 
private:
	static CvPoint getRandomPoint(CvSize gridSize); 
	static CvPoint getOrderedPoint(int fixationNumber, CvSize gridSize); 
	static CvPoint getSpiralingPoint(int fixationNumber, CvSize gridSize); 

};

#endif

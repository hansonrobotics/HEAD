/*
 *  MultiObjectTrackingState.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 8/12/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef MULTIOBJECTTRACKINGSTATE_H
#define MULTIOBJECTTRACKINGSTATE_H


#include <opencv2/core/core_c.h>
#include <iostream>
#include <vector>

#include "ImagePatch.h"
#include "PatchList.h"

/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> A data structure for keeping simple
 * statistics of objects, useful for trying to uniquely identify and track
 * several objects simultaneously. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
struct TrackingObject {
public:
	/**
	 * \brief Center x-location of the tracked object.
	 */
	int x;
	
	/**
	 * \brief Center y-location of the tracked object.
	 */
	int y;
	
	/**
	 * \brief Size of the tracked object.
	 */
	int radius; 
	
	/**
	 * \brief Indicates whether the object was just seen. 
	 *
	 * Invisible objects
	 * are ones that were not detected in the current frame, which may happen
	 * because of things like occlusion or poor detector performance. 
	 * They are remembered from previous experience in case the object detector
	 * finds them later.
	 */
	bool visible; 
	
	/**
	 * \brief Color information used to help discrminate and uniquely identify
	 * objects.
	 */
	CvScalar color; 
}; 

/**
 *\ingroup AuxGroup
 * \brief  <tt>Auxilliary Tool:</tt> A class for trying to uniquely identify and 
 * track several objects simultaneously. 
 *
 * This tracking is done using some heuristic rules. New object locations are
 * matched to old ones based on the color of the object, and its remembered
 * location. 
 *
 * \author Nicholas Butko
 * \date 2010
 * version 0.4
 */
class MultiObjectTrackingState {
public:
	/**
	 * \brief Constructor.
	 */
	MultiObjectTrackingState(); 
	
	/**
	 * \brief Update location of tracked objects based on the output of an
	 * object detector, and user provided color information. 
	 * 
	 * @param patches Output of an object detector applied to a frame of video
	 * or camera.
	 *
	 * @param colors A color statistic of each patch. This could be the mean
	 * patch color value, but any three numbers the user believes can be used
	 * to help discriminate one object from another can be used. 
	 */
	void updateState(const std::vector<SearchResult>& patches, const std::vector<CvScalar>& colors);
	
	/**
	 * \brief Visualize the tracking state.
	 * 
	 * Objects are given unique colors (which does not correspond to their track
	 * colors). Invisible objects are represented by a dot at their center. If 
	 * the tracker is performing well, colors should "stick" to objects.
	 *
	 * @param colorInputImage An image to draw over in order to visualize 
	 * tracker output. This must be a 3 channel, IPL_DEPTH_8U image. Calling
	 * drawState draws directly onto colorInputImage. 
	 */
	void drawState(IplImage* colorInputImage); 
	
	/**
	 * \brief Re-initialzie the tracking state, and optionally give hints about
	 * the number of objects being tracked. 
	 * 
	 * @param maxObjects Give a hint to the tracker about the maximum number 
	 * of objects in the scene, which makes the tracker more robust to false 
	 * alarms. 
	 */
	void reset(int maxObjects=-1 ); 
	
	/**
	 * \brief Query the current state of the tracker.
	 *
	 * The indices of this state
	 * vector should "stick" to objects, i.e. the same object should
	 * always be at the same position in the vector. 
	 * 
	 * @return A list of location of tracked objects. 
	 */
	std::vector<TrackingObject> getState(); 
protected:
	int maxCoins; 
	static double jet_colors[193]; 
	
	std::vector<TrackingObject> state; 
}; 

#endif 

/*
 *  ObjectDetector.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _OBJECTDETECTOR_H
#define _OBJECTDETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <iostream>
#include <string>

/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A virtual class for providing the skeleton for specific object detectors.
 *
 * In general, we conceive of an object detector as something that takes an image as input, and produces a list
 * of object locations as output. As such, an Object Detector needs to implement the method: 
 *
 * virtual CvSeq* detectObjects(IplImage* image) 
 *
 * where CvSeq* is an array of CvRect, each specifying the location of the object detector's target. Each 
 * CvRect should specify the top-left pixel corner of an object's bounding box, and also the height and
 * width of the bounding box. The Object's location is taken to be the center of this bounding box. 
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class ObjectDetector {
public:
	
	/**
	 * \brief Virtual Method for inheriting classes to override: All object detectors must have a detectOjbects
	 * method. 
	 *
	 * @param image The image to be searched for some target object.
	 * @return A CvSeq* array of CvRect, each specifying the location of the object detector's target. Each 
	 * CvRect should specify the top-left pixel corner of an object's bounding box, and also the height and
	 * width of the bounding box. The Object's location is taken to be the center of this bounding box. 
	 */
	//virtual CvSeq* detectObjects(IplImage* image) = 0; 
	virtual std::vector<cv::Rect> detectObjects(IplImage* image) = 0; 

	virtual ~ObjectDetector(); 
	
	
	/**
	 * \brief Change the file used by the object detector for doing detecting.
	 * This is critical if a weights file is located at an absolute path that 
	 * may have changed from training time. 
	 * 
	 * When an ObjectDetector is loaded from disk, it will try to reload its
	 * weights file from the same source used in training. If this fails, a
	 * warning will be printed, and the detector's source will need to be set.
	 */
	virtual void setDetectorSource(std::string newFileName) =0; 
	
	/**
	 * \brief Write to a file -- this calls addToStream(), which is what should be set in subclasses.
	 */
	friend std::ostream& operator<< (std::ostream& ofs, ObjectDetector* model); 
	
	/**
	 * \brief Read from a file -- this calls readFromStream(), which is what should be set in subclasses.
	 */
	friend std::istream& operator>> (std::istream& ifs, ObjectDetector* model); 
protected:
	/**
	 * \brief Virtual Method for inheriting classes to override: An object detector should be able to 
	 * serialize itself to be saved in a file. 
	 **/
	virtual void addToStream(std::ostream& out) = 0; 
	
	/**
	 * \brief Virtual Method for inheriting classes to override: An object detector should be able to 
	 * read in its properties from a file, and reinitialize itself appropriately.
	 **/
	virtual void readFromStream(std::istream& in) = 0; 
	
};

#endif

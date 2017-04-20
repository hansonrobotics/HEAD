/*
 *  OpenCVHaarDetector.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _OPENCVHAARDETECTOR_H
#define _OPENCVHAARDETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "ObjectDetector.h"


/**
 *\ingroup AuxGroup
 * \brief <tt>Auxilliary Tool:</tt> A specific object detector that uses OpenCV's Haar Cascade Classifier to 
 * detect objects. The performance / behavior of the object detector is stored in an XML file, which is 
 * passed in in the constructor. The XML file provided in the data directory may be used for finding faces, 
 * or you can use OpenCV to train your own.
 *
 * \author Nicholas Butko
 * \date 2010
 * \version 0.4
 */
class OpenCVHaarDetector : public ObjectDetector {
public: 
	/**
	 * \brief Placeholder Constructor. 
	 * 
	 * Used to create a placeholder for an object detector, which can then be read 
	 * from a file stream using the \>\> operator. Typical usage for this constructor is:
	 *
	 * <code>
	 * detector = new OpenCVHaarDetector(); <br>
	 * in >> detector; 
	 * </code>
	 */
	OpenCVHaarDetector(); 
	
	
	/**
	 * \brief Constructor. 
	 * 
	 * Used to create an object detector out of a previously stored XML file. 
	 *
	 * @param filename The name of the XML file that contains all of the features / weights / etc.
	 * needed by the Haar Cascaded Object Detector. The provided file, "data/haarcascade_frontalface_alt2.xml"
	 * is distributed with OpenCV. OpenCV can be used to train detectors of objects other than frontal faces.
	 */
	OpenCVHaarDetector(const char* filename); 
	
	
	/**
	 * \brief Deep Copy Constructor: Create a Haar detector that is identical to 
	 * the one copied.
	 **/
	OpenCVHaarDetector(OpenCVHaarDetector* detectorToCopy);
	
	/**
	 * \brief Default Destructor. 
	 * 
	 * Deallocates all memory associated with the haar detector. 
	 */
	~OpenCVHaarDetector(); 
	
	/**
	 * \brief Virtual Method for overridden from parent class (ObjectDetector). Applies the 
	 * cvHaarDetectObjects() method to the provided image patch.
	 *
	 * @param image The image to be searched for some target object.
	 * @return A CvSeq* array of CvRect, each specifying the location of the object detector's target. Each 
	 * CvRect should specify the top-left pixel corner of an object's bounding box, and also the height and
	 * width of the bounding box. The Object's location is taken to be the center of this bounding box. 
	 */
	//CvSeq* detectObjects(IplImage* image); 
    std::vector<cv::Rect> detectObjects(IplImage* image); 
		
	/**
	 * \brief Sets the factor by which the image-patch-search-scale is increased. Should be greater than 1.
	 * By default, the scale factor is 1.1, meaning that faces are searched for at sizes that increase by 
	 * 10%.
	 *
	 * @param factor The size-granularity of object search.
	 */
	void setHaarCascadeScaleFactor(double factor) ;
	
	/**
	 * \brief Sets the minimum patch size at which the classifier searches for the object. By default,
	 * this is 0, meaning that the smallest size appropriate to XML file is used. In the case of the
	 * frontal face detector provided, this happens to be 20x20 pixels.
	 *
	 * @param size The width/height of the smallest patches to try to detect the object.
	 */
	void setHaarCascadeMinSize(int size) ;
	
	/**
	 * \brief Change the file used by the object detector for doing detecting.
	 * This is critical if a weights file is located at an absolute path that 
	 * may have changed from training time. 
	 * 
	 * When an ObjectDetector is loaded from disk, it will try to reload its
	 * weights file from the same source used in training. If this fails, a
	 * warning will be printed, and the detector's source will need to be set.
	 */
	void setDetectorSource(std::string newFileName) ; 
protected:
	void readFromStream(std::istream& in); 
	void addToStream(std::ostream& out); 
private: 
	void reInitialize() ; 
	double vjScale; 
	int vjMinSize; 
	//CvHaarClassifierCascade* cascade;
    cv::CascadeClassifier cascade; 
    //CvMemStorage* storage;
	std::string filename; 
	
};

#endif

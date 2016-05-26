/*
 *  ObjectDetector.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/7/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "ObjectDetector.h"
using namespace std; 

ObjectDetector::~ObjectDetector(){}
ostream& operator<< (ostream& ofs, ObjectDetector* model) {
	model->addToStream(ofs); 
	return ofs; 
}
istream& operator>> (istream& ifs, ObjectDetector* model) {
	model->readFromStream(ifs); 
	return ifs; 
}
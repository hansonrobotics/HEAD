/*
 *  MultinomialObservationModel.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 3/8/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "MultinomialObservationModel.h"

using namespace std; 

MultinomialObservationModel::MultinomialObservationModel(CvSize gridSize) {
	observationProbability = cvCreateImage(cvSize(gridSize.width, gridSize.height), IPL_DEPTH_32F, 1);	
	maxfaces = 10;  
	int sizes[3] = {gridSize.width, gridSize.height, maxfaces}; 
	countsObsGivenFace = cvCreateMatND(3,sizes, CV_64FC1); 
	countsObsGivenNoFace = cvCreateMatND(3,sizes, CV_64FC1); 
	probObsGivenFace = cvCreateMatND(3,sizes, CV_64FC1); 
	probObsGivenNoFace = cvCreateMatND(3,sizes, CV_64FC1); 
	fillProbTablesHeuristic(); 
	resetCounts(); 
}

MultinomialObservationModel::MultinomialObservationModel(MultinomialObservationModel* modelToCopy) {
	observationProbability = cvCloneImage(modelToCopy->observationProbability);	
	maxfaces = modelToCopy->maxfaces;  
	countsObsGivenFace = cvCloneMatND(modelToCopy->countsObsGivenFace); 
	countsObsGivenNoFace = cvCloneMatND(modelToCopy->countsObsGivenNoFace); 
	probObsGivenFace = cvCloneMatND(modelToCopy->probObsGivenFace); 
	probObsGivenNoFace = cvCloneMatND(modelToCopy->probObsGivenNoFace); 
}

MultinomialObservationModel::~MultinomialObservationModel() {
	cvReleaseImage(&(this->observationProbability)); 
	cvReleaseMatND(&(this->probObsGivenFace)); 
	cvReleaseMatND(&(this->probObsGivenNoFace)); 
	cvReleaseMatND(&(this->countsObsGivenFace)); 
	cvReleaseMatND(&(this->countsObsGivenNoFace)); 
}

void MultinomialObservationModel::readFromStream(istream& in) {
	double matFloat; 
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxfaces; c++) {
				in >> matFloat; 
				cvSetReal3D(probObsGivenFace, y, x, c, matFloat); 
			}
		}		
	}
	
	for (int y = 0; y < observationProbability->height; y++) {	
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxfaces; c++) {
				in >> matFloat; 
				cvSetReal3D(probObsGivenNoFace, y, x, c, matFloat); 
			}
		}		
	}
	
	
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxfaces; c++) {
				in >> matFloat; 
				cvSetReal3D(countsObsGivenFace, y, x, c, matFloat); 
			}
		}		
	}
	
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxfaces; c++) {
				in >> matFloat; 
				cvSetReal3D(countsObsGivenNoFace, y, x, c, matFloat); 
			}
		}		
	}
	//cout << "Read From File." << endl; 
}

void MultinomialObservationModel::addToStream(ostream& out) {
	//int maxcount = cvGetDimSize(probObsGivenFace, 2); 
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxfaces; c++) {
				out << cvGetReal3D(probObsGivenFace, y, x, c) << " "; 
			}
			out << endl; 
		}		
	}
	
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxfaces; c++) {
				out << cvGetReal3D(probObsGivenNoFace, y, x, c) << " "; 
			}
			out << endl; 
		}		
	}
	
	
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxfaces; c++) {
				out << (long)cvGetReal3D(countsObsGivenFace, y, x, c) << " "; 
			}
			out << endl; 
		}		
	}
	
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxfaces; c++) {
				out << (long)cvGetReal3D(countsObsGivenNoFace, y, x, c) << " "; 
			}
			out << endl; 
		}		
	}
}

void MultinomialObservationModel::fillProbTablesHeuristic() {
	int maxcount = cvGetDimSize(probObsGivenFace, 2); 	
	
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxcount; c++) {
				cvSetReal3D(probObsGivenFace, y, x, c,c*5+.999); 
				cvSetReal3D(probObsGivenNoFace, y, x, c,1); 
			}
		}		
	}
}

void MultinomialObservationModel::resetCounts() {	
	int maxcount = cvGetDimSize(probObsGivenFace, 2); 
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxcount; c++) {
				cvSetReal3D(countsObsGivenFace, y, x, c,1); 
				cvSetReal3D(countsObsGivenNoFace, y, x, c,1); 
			}
		}		
	}
}

void MultinomialObservationModel::combineEvidence(MultinomialObservationModel* otherModel) {
	int maxcount = cvGetDimSize(probObsGivenFace, 2); 
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			for (int c = 0; c < maxcount; c++) {
				double v1 = cvGetReal3D(countsObsGivenFace, y, x, c); 
				double v2 = cvGetReal3D(otherModel->countsObsGivenFace, y, x, c); 
				cvSetReal3D(countsObsGivenFace, y, x, c,v1+v2-1); 
				double v3 = cvGetReal3D(countsObsGivenNoFace, y, x, c); 
				double v4 = cvGetReal3D(otherModel->countsObsGivenNoFace, y, x, c); 
				cvSetReal3D(countsObsGivenNoFace, y, x, c,v3+v4-1); 
			}
		}		
	}
	normalizeProbTables(); 
}

void MultinomialObservationModel::normalizeProbTables() {	
	int maxcount = cvGetDimSize(probObsGivenFace, 2); 
	for (int y = 0; y < observationProbability->height; y++) {
		for (int x = 0; x < observationProbability->width; x++) {
			double sum1 = 0; 
			double sum2 = 0;
			for (int c = 0; c < maxcount; c++) {
				sum1 = sum1 + cvGetReal3D(countsObsGivenFace, y, x, c); 
				sum2 = sum2 + cvGetReal3D(countsObsGivenNoFace, y, x, c); 
			}
			for (int c = 0; c < maxcount; c++) {
				double val1 = cvGetReal3D(countsObsGivenFace, y, x, c); 
				cvSetReal3D(probObsGivenFace, y, x, c, val1/sum1); 
				double val2 = cvGetReal3D(countsObsGivenNoFace, y, x, c); 
				cvSetReal3D(probObsGivenNoFace, y, x, c, val2/sum2); 
			}
		}		
	}
}

void MultinomialObservationModel::setObservationProbability(CvPoint searchPoint, IplImage* faceCount) {
	if (searchPoint.x < 0 || searchPoint.y < 0) {
		cout << "Bad Search Point: " << searchPoint.x << ", " <<searchPoint.y << endl; 
	}
	
	int maxcount = cvGetDimSize(probObsGivenFace, 2); 
	for (int gridlocy = 0; gridlocy < observationProbability->height; gridlocy++) {
		for (int gridlocx = 0; gridlocx < observationProbability->width; gridlocx++) {
			int offsetx = abs(searchPoint.x-gridlocx); 
			int offsety = abs(searchPoint.y-gridlocy); 
			int c = cvGetReal2D(faceCount, gridlocy, gridlocx); 
			c = c<maxcount?c:maxcount-1; 
			double pface = cvGetReal3D(probObsGivenFace, offsety, offsetx, c);
			double pnoface = cvGetReal3D(probObsGivenNoFace, offsety, offsetx, c); 
			cvSetReal2D(observationProbability, gridlocy, gridlocx, pface/pnoface); 
		}
	}	
}


void MultinomialObservationModel::updateProbTableCounts(CvPoint searchPoint, IplImage* faceCount, CvRect faceLocation) {
	
	int maxcount = cvGetDimSize(probObsGivenFace, 2);
	
	//i/j are the points in the grid independent of where we're looking. 
	for (int i = 0; i < observationProbability->width; i++) {
		for(int j = 0; j < observationProbability->height; j++) {
			int c = cvGetReal2D(faceCount, j, i);
			c = c<maxcount?c:maxcount-1; 
			int offsetx = abs(searchPoint.x-i); 
			int offsety = abs(searchPoint.y-j);
			if (i >= faceLocation.x && i < faceLocation.x+faceLocation.width
				&& j >= faceLocation.y && j < faceLocation.y+faceLocation.height) {
				cvSetReal3D(countsObsGivenFace, offsety, offsetx, c, 
							cvGetReal3D(countsObsGivenFace, offsety, offsetx, c )+1); 
			} else {
				cvSetReal3D(countsObsGivenNoFace, offsety, offsetx, c, 
							cvGetReal3D(countsObsGivenNoFace, offsety, offsetx, c)+1); 
			}
		}
	}
	//end processing at this fixation location/scale
}

ostream& operator<< (ostream& ofs, MultinomialObservationModel* model) {
	model->addToStream(ofs); 
	return ofs ;
}
istream& operator>> (istream& ifs, MultinomialObservationModel* model) {
	model->readFromStream(ifs); 
	return ifs; 
}

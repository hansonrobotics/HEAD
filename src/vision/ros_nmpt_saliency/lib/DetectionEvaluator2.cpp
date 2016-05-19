/*
 *  DetectionEvaluator2.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 10/25/10.
 *  Copyright 2010 UC San Diego. All rights reserved.
 *
 */

#include "DetectionEvaluator2.h"
#include <iostream>
#include <sstream>
#include "NMPTUtils.h"

using namespace std;
using namespace NMPTUtils; 
using namespace cv; 

double DetectionEvaluator2::acceptArea = .5; 

/*bool operator<(const EvaluationMetrics& a, const EvaluationMetrics& b) {
	return a.threshold < b.threshold; 
}*/


DetectionEvaluator2::DetectionEvaluator2() {
}

DetectionEvaluator2::~DetectionEvaluator2(){
} 	

DetectionEvaluator2 & DetectionEvaluator2::operator=(const DetectionEvaluator2 &rhs) {
	if (this != &rhs) {
		fileNames = rhs.fileNames; 
		targets = rhs.targets; 
	}
	return *this; 
}

DetectionEvaluator2::DetectionEvaluator2(const DetectionEvaluator2 &rhs) {
	if (this != &rhs) {
		fileNames = rhs.fileNames; 
		targets = rhs.targets; 
	} 
}

DetectionEvaluator2::DetectionEvaluator2(const ImageDataSet2 &imageDataSet) {
	setDataSet(imageDataSet); 
}

DetectionEvaluator2::DetectionEvaluator2(const string &fileListFile, const string &labelFile) {
	setDataSet(fileListFile, labelFile); 
}

void DetectionEvaluator2::setDataSet(const string &fileListFile, const string &labelFile) {
	if (fileListFile.empty() || labelFile.empty()) {
		cout << "Warining: To evaluate detection, both an images file and a labels file must be provided." << endl; 
		return; 
	}
	ImageDataSet2 labeledImages(fileListFile, labelFile); 
	setDataSet(labeledImages); 
}

void DetectionEvaluator2::setDataSet(const ImageDataSet2 &imageDataSet) {
	init(imageDataSet); 
}

size_t DetectionEvaluator2::dataSetSize() const {
	return fileNames.size(); 
}

int DetectionEvaluator2::empty() const {
	return fileNames.empty(); 
}

void DetectionEvaluator2::init(const ImageDataSet2 &labeledImages) {
	fileNames.clear(); 
	targets.clear(); 
	for (int i = 0; i < labeledImages.getNumEntries(); i++) {
		string s = labeledImages.getFileName(i); 
		vector<double> labels = labeledImages.getFileLabels(i); 
		Rect box(labels[0]-labels[2]/2, labels[1]-labels[2]/2, labels[2],labels[2]); 
		int found = 0; 
		for (unsigned int j = 0; j < fileNames.size(); j++) {
			if (fileNames[j] == s) {
				if (box.width > 0 && box.height > 0)
					targets[j].push_back(box); 
				found = 1; 
				break; 
			}
		}
		if (!found) {
			fileNames.push_back(s); 
			vector<Rect> r; 
			if (box.width > 0 && box.height > 0) r.push_back(box);
			targets.push_back(r); 
		}
	}
}

vector<string> DetectionEvaluator2::getFileNames()  const {
	return fileNames; 
}

vector<vector<Rect> > DetectionEvaluator2::getTargetLocations() const {
	return targets; 
}

vector<EvaluationMetrics> DetectionEvaluator2::evaluatePerformance(const vector<vector<SearchResult> >& imBoxes) const {
	vector<EvaluationMetrics> retval; 
	retval.clear(); 
	vector<vector<EvaluationMetrics> > ms; //ms is the list of threshold/performance for each image
	ms.clear(); 
	for (unsigned int i = 0; i < imBoxes.size(); i++) {
		ms.push_back(evaluateImagePerformance(i, imBoxes[i])); 
		/*
		 cout << "Evaluated performance of Image " << i << endl; 
		 for (unsigned int j = 0; j < ms[i].size(); j++) {
		 cout << "For threshold " << ms[i][j].threshold << " there were " 
		 << (1-ms[i][j].hit_ratio)*100 << "% misses (" << ms[i][j].num_hits 
		 << " hits & " << ms[i][j].num_misses << " misses) and " << ms[i][j].num_false_alarms
		 << " false alarms." << endl; 
		 }
		 
		 cout << "=========================================" << endl; 
		 */
	}
	//cout << "Evaluated thresholds for " << ms.size() << " images." << endl; 
	for (unsigned int i = 0; i < ms.size(); i++) { //for each image
		//cout << "In image " << i << " there are " << ms[i].size() << " thresholds." << endl; 
		for (unsigned int j = 0; j < ms[i].size(); j++) { //for each threshold
			EvaluationMetrics m; 
			m.threshold = ms[i][j].threshold; 
			m.num_hits = 0; 
			m.num_misses = 0; 
			m.num_false_alarms = 0; 
			m.hit_ratio = 0; 
			m.hits.clear(); 
			m.misses.clear();
			m.false_alarms.clear(); 
			//get average performance over all images at that threshold
			for (unsigned int k = 0; k < ms.size();  k++) {
				EvaluationMetrics n = getPerformanceAtThreshold(ms[k], m.threshold); 
				/*cout << "For image " << k << " with threshold " << m.threshold
				 << " there were " << n.num_hits << " hits, " << n.num_misses 
				 << " misses, " << n.num_false_alarms << " false alarms, and hr " 
				 << n.hit_ratio << endl; */
				m.num_hits = m.num_hits+n.num_hits; 
				m.num_misses = m.num_misses+n.num_misses; 
				m.num_false_alarms=m.num_false_alarms+n.num_false_alarms; 
				m.hit_ratio = m.hit_ratio+n.hit_ratio; 
			}
			m.num_hits = m.num_hits/ms.size(); 
			m.num_misses = m.num_misses/ms.size(); 
			m.num_false_alarms = m.num_false_alarms/ms.size(); 
			m.hit_ratio=m.hit_ratio/ms.size(); 
			retval.push_back(m); 
		}
	}
	//cout << "In all, " << retval.size() << "thresholds were evaluated." << endl; 
	sort(retval.begin(), retval.end()); 
	return retval; 
}

EvaluationMetrics DetectionEvaluator2::getPerformanceAtThreshold(const vector<EvaluationMetrics>& ms, 
																double threshold) const {
	//ms is list of performance/threshold on a given image, and we want to know
	//what the performance would be by setting an arbitrary threshold. This is
	//equivalent to setting the threshold to something inbetween two thresholds.
	//If the threshold is higher than any in the list, all patches are rejected
	//-there are no false alarms, no hits, and everything is missed.
	//If the threshold is lower than any in the list, all patches are accepted,
	//which is equivalent to setting the threshold to the lowest value in the 
	//list
	//If there threshold is inbetween, then it is equivalent to finding the 
	//in-between place, and taking the performance of the higher.
	EvaluationMetrics retval; 
	retval.threshold=threshold; 
	retval.num_hits = 0; 
	retval.num_misses = (ms[0].num_hits+ms[0].num_misses) ; //Figure this out hit_ratio = num_hits/num_total
	retval.num_false_alarms = 0; 
	retval.hit_ratio = 0; 
	retval.hits.clear();
	retval.misses.clear();
	retval.false_alarms.clear(); 
	for (int i = (int)ms.size()-1; i >= 0; i--) {
		if (ms[i].threshold < threshold) {
			//cout << " " << ms[i].threshold << "<" << threshold;  
			break;
		}
		//cout << i << "-"; 
		retval.num_hits = ms[i].num_hits; 
		retval.num_misses = ms[i].num_misses; 
		retval.num_false_alarms = ms[i].num_false_alarms; 
		retval.hit_ratio = ms[i].hit_ratio; 
	}
	//cout << endl; 
	return retval; 
}

vector<EvaluationMetrics> DetectionEvaluator2::evaluateImagePerformance(int imNum, 
																	   const vector<SearchResult>& boxes) const {
	vector<EvaluationMetrics> retval; 
	for (unsigned int i = 0; i < boxes.size(); i++) {
		EvaluationMetrics m = evaluateImagePerformanceWithThreshold(imNum, boxes, boxes[i].value); 
		m.hits.clear(); 
		m.misses.clear(); 
		m.false_alarms.clear(); 
		retval.push_back(m); 
	}
	sort(retval.begin(), retval.end()); 
	return retval; 
}

EvaluationMetrics DetectionEvaluator2::evaluateImagePerformanceWithThreshold(int imNum, 
																			vector<SearchResult> boxes,
																			double threshold) const {
	EvaluationMetrics retval; 
	retval.hits.clear(); 
	retval.misses.clear(); 
	retval.false_alarms.clear(); 
	
	sort(boxes.begin(), boxes.end()); 
	reverse(boxes.begin(), boxes.end()); 
	//boxes are sorted by threshold in descending order
	//if one box is below threshold, all remaining boxes are below threshold.
	
	for (unsigned int i = 0; i < targets[imNum].size(); i++) {
		int found = 0; 
		for (unsigned int j = 0; j < boxes.size(); j++) {
			if (boxes[j].value < threshold) break; 
			if (rectAreaOverlapRatio(targets[imNum][i], boxes[j].imageLocation) >= acceptArea) {
				retval.hits.push_back(boxes[j].imageLocation); 
				boxes.erase(boxes.begin()+j); 
				found = 1; 
				break; 
			}
		}
		if (!found) retval.misses.push_back(targets[imNum][i]); 
	}
	for (unsigned int i = 0; i < boxes.size(); i++) {
		if (boxes[i].value < threshold) break; 
		retval.false_alarms.push_back(boxes[i].imageLocation); 
	}
	
	retval.num_hits = retval.hits.size(); 
	retval.num_misses = retval.misses.size(); 
	retval.num_false_alarms = retval.false_alarms.size(); 
	retval.hit_ratio = retval.num_hits*1.0/targets[imNum].size(); 
	retval.threshold = threshold; 
	
	return retval; 
}

string DetectionEvaluator2::outputOpenCVDescriptionFormatForHaarTraining() const {
	
	stringstream retval; 
	
	for (unsigned int i = 0; i < fileNames.size(); i++) {
		retval << fileNames[i] << " "; 
		retval << targets[i].size() ; 
		for (unsigned int j = 0; j < targets[i].size(); j++) {
			retval << " " << (int)targets[i][j].x << " " << (int)targets[i][j].y 
			<< " " << (int)targets[i][j].width << " " << (int)targets[i][j].height; 
		}
		retval << endl; 
	}
	return retval.str(); 
}


FileStorage& operator << (cv::FileStorage &fs, const DetectionEvaluator2 &rhs) {
	fs << "{" << "empty" << rhs.empty(); 
	if (!rhs.empty()) {
		fs << "entries" << "[";  
		for (size_t i = 0; i < rhs.fileNames.size(); i++) {
			//vector<double> labels = rhs.numLabelsPerImage() > 0? rhs.labels[i] :vector<double>(0); 
			fs << "{" << "name" <<  rhs.fileNames[i] ;
			if (rhs.targets.size() > 0){
				fs << "targets" << "[" ; 
				for (size_t j = 0; j < rhs.targets[i].size(); j++) {
					fs << "{" << "x" << rhs.targets[i][j].x <<  "y" << rhs.targets[i][j].y 
					<<  "width" << rhs.targets[i][j].width <<  "height" << rhs.targets[i][j].height << "}"; 
				}
				fs << "]";
			}
			fs << "}"; 
		}
		fs << "]"; 
	}
	fs << "}" ; 
	return fs; 
}

void operator >> ( const cv::FileNode &fs, DetectionEvaluator2 &rhs)  {
	int empty; 
	fs["empty"] >> empty; 
	if (empty) return; 
	FileNode node = fs["entries"]; 
	for (size_t i = 0; i < node.size(); i++) {
		string fname;
		vector<Rect> targets; 
		node[i]["name"] >> fname; 
		if (node[i].size() > 1) {
			FileNode n = node[i]["targets"] ; 
			for (size_t j = 0; j < fname.size(); j++) {
				Rect r; 
				n[j]["x"] >> r.x; 
				n[j]["y"] >> r.y; 
				n[j]["width"] >> r.width; 
				n[j]["height"] >> r.height; 
				targets.push_back(r); 
			}
		}
		rhs.fileNames.push_back(fname); 
		rhs.targets.push_back(targets); 
	}
}

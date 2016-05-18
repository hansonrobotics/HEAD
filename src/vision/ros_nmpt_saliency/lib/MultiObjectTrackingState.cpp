/*
 *  MultiObjectTrackingState.cpp
 *  OpenCV
 *
 *  Created by Nicholas Butko on 8/12/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "MultiObjectTrackingState.h"
#include "DebugGlobals.h"
#include <iostream>

using namespace std; 

double MultiObjectTrackingState::jet_colors[193]={0.000000, 0.000000, 0.562500, 
0.000000, 0.000000, 0.625000, 0.000000, 0.000000, 0.687500, 0.000000, 0.000000, 
0.750000, 0.000000, 0.000000, 0.812500, 0.000000, 0.000000, 0.875000, 0.000000, 
0.000000, 0.937500, 0.000000, 0.000000, 1.000000, 0.000000, 0.062500, 1.000000, 
0.000000, 0.125000, 1.000000, 0.000000, 0.187500, 1.000000, 0.000000, 0.250000, 
1.000000, 0.000000, 0.312500, 1.000000, 0.000000, 0.375000, 1.000000, 0.000000, 
0.437500, 1.000000, 0.000000, 0.500000, 1.000000, 0.000000, 0.562500, 1.000000, 
0.000000, 0.625000, 1.000000, 0.000000, 0.687500, 1.000000, 0.000000, 0.750000, 
1.000000, 0.000000, 0.812500, 1.000000, 0.000000, 0.875000, 1.000000, 0.000000, 
0.937500, 1.000000, 0.000000, 1.000000, 1.000000, 0.062500, 1.000000, 0.937500, 
0.125000, 1.000000, 0.875000, 0.187500, 1.000000, 0.812500, 0.250000, 1.000000, 
0.750000, 0.312500, 1.000000, 0.687500, 0.375000, 1.000000, 0.625000, 0.437500, 
1.000000, 0.562500, 0.500000, 1.000000, 0.500000, 0.562500, 1.000000, 0.437500, 
0.625000, 1.000000, 0.375000, 0.687500, 1.000000, 0.312500, 0.750000, 1.000000, 
0.250000, 0.812500, 1.000000, 0.187500, 0.875000, 1.000000, 0.125000, 0.937500, 
1.000000, 0.062500, 1.000000, 1.000000, 0.000000, 1.000000, 0.937500, 0.000000, 
1.000000, 0.875000, 0.000000, 1.000000, 0.812500, 0.000000, 1.000000, 0.750000, 
0.000000, 1.000000, 0.687500, 0.000000, 1.000000, 0.625000, 0.000000, 1.000000, 
0.562500, 0.000000, 1.000000, 0.500000, 0.000000, 1.000000, 0.437500, 0.000000, 
1.000000, 0.375000, 0.000000, 1.000000, 0.312500, 0.000000, 1.000000, 0.250000, 
0.000000, 1.000000, 0.187500, 0.000000, 1.000000, 0.125000, 0.000000, 1.000000, 
0.062500, 0.000000, 1.000000, 0.000000, 0.000000, 0.937500, 0.000000, 0.000000, 
0.875000, 0.000000, 0.000000, 0.812500, 0.000000, 0.000000, 0.750000, 0.000000, 
0.000000, 0.687500, 0.000000, 0.000000, 0.625000, 0.000000, 0.000000, 0.562500, 
0.000000, 0.000000, 0.500000, 0.000000, 0.000000, -1};



void MultiObjectTrackingState::reset(int maxCoins ) {
	this->maxCoins = maxCoins; 
	state.clear(); 
	
	if (maxCoins > 0) {
		TrackingObject o; 
		o.x = 1; 
		o.y = 1; 
		o.radius = 30; 
		o.visible = 0; 		
		o.color = CV_RGB(0,0,0); 
		state.resize(maxCoins,  o); 
	}
}

MultiObjectTrackingState::MultiObjectTrackingState() {
	maxCoins=-1; 
	reset(maxCoins);
}

void MultiObjectTrackingState::updateState(const vector<SearchResult>& patches, const vector<CvScalar>& colors) {
	//state.clear(); 
	vector<TrackingObject> newstates; 
	if (_TRACKING_DEBUG) cout << "Size of Patches: " << patches.size() << " ; size of colors: " << colors.size() << endl; 
	for (unsigned int i = 0; i < patches.size(); i++) {
		TrackingObject o; 		
		CvRect box = patches[i].imageLocation; 		
		o.x = box.x+box.width/2; 
		o.y = box.y+box.height/2; 
		o.radius = box.width/2; 
		o.visible = 1; 
		if (colors.size() == patches.size())
			o.color = colors[i]; 
		else
			o.color = CV_RGB(0,0,0); 
		//color is <0,0,0> or a unit-length vector. 
		double clen = sqrt(o.color.val[0]*o.color.val[0]
						+ o.color.val[1]*o.color.val[1]
						+ o.color.val[2]*o.color.val[2]); 
		if (_TRACKING_DEBUG) cout << "State " << i << "'s color vector length is " << clen << endl; 
		clen = clen>0?1.0/clen:1; 
		o.color.val[0] = o.color.val[0]*clen; 
		o.color.val[1] = o.color.val[1]*clen; 
		o.color.val[2] = o.color.val[2]*clen; 
		newstates.push_back(o); 
		if (_TRACKING_DEBUG) cout << "State " << i << " has color " ;
		if (_TRACKING_DEBUG) cout << o.color.val[0] << ", " << o.color.val[1];
		if (_TRACKING_DEBUG) cout << ", " << o.color.val[2] << endl; 
	}
	
	/*Step 1: Find objects that haven't moved*/
	for (unsigned int i = 0; i < state.size(); i++) {
		state[i].visible = 0; 
		for (unsigned int j = 0; j < newstates.size(); j++) {			
			int xdist = state[i].x-newstates[j].x; 
			int ydist = state[i].y-newstates[j].y; 
			xdist = xdist<0?-xdist:xdist; 
			ydist = ydist<0?-ydist:ydist; 
			if (xdist < 15 && ydist < 15) {
				state[i] = newstates[j]; 
				newstates.erase(newstates.begin()+j); 
				break; 
			}
		}				
	}
	
	/*Step 2: Assign remaining objects to new ones*/
	vector<TrackingObject*> unmatched; 
	for (unsigned int i = 0; i < state.size(); i++) {
		if (state[i].visible) continue; 
		unmatched.push_back(&state[i]);
	}
	if (!unmatched.empty()) {
		for (unsigned int i = 0; i < unmatched.size(); i++) {			
			if (newstates.empty()) break;
			int minind = -1; 
			double mindist = INFINITY; 
			for (unsigned int j = 0; j < newstates.size(); j++)  {
				double cdist = (*unmatched[i]).color.val[0]*newstates[j].color.val[0]
								+ (*unmatched[i]).color.val[1]*newstates[j].color.val[1]
				                + (*unmatched[i]).color.val[2]*newstates[j].color.val[2]; 
				cdist = 1-cdist; 
				if (cdist < 0 || cdist > 1) cerr << "Check color dot-product code." << endl; 
				
				 int xdist = (*unmatched[i]).x-newstates[j].x; 
				 int ydist = (*unmatched[i]).y-newstates[j].y; 
				 if (xdist+ydist < mindist) {
				 minind = j; 
				 mindist = xdist+ydist; 
				 }
				/*
				if (cdist < mindist) {
					minind = j; 
					mindist = cdist; 
				}*/
					
			}
			*unmatched[i] = newstates[minind];
			newstates.erase(newstates.begin()+minind); 
		}
	}
	
	/*Step 3: Create new slots for remaining new ones*/	
	for (unsigned int i = 0; i < newstates.size(); i++) {
		if (maxCoins > 0 && (int)state.size() >= maxCoins) break; 
		state.push_back(newstates[i]); 
	}
}
vector<TrackingObject> MultiObjectTrackingState::getState() {
	return state; 
}

void MultiObjectTrackingState::drawState(IplImage* colorInputImage) {
	int primeskip = 3; //17
	for (unsigned int i = 0; i < state.size(); i++) {
		//if (!state[i].visible) continue; 
		int ind = (primeskip*3*i)%192; 
		CvScalar color = CV_RGB(255*jet_colors[ind],
							   255*jet_colors[ind+1],
							   255*jet_colors[ind+2]); 
		cvCircle(colorInputImage, cvPoint(state[i].x,state[i].y),
				 3,color,-1,8,0);
		if (state[i].visible)
		cvCircle(colorInputImage, cvPoint(state[i].x,state[i].y),
				 state[i].radius,color,3,8,0);
	}
}

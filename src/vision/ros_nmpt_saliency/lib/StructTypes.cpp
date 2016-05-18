#include "StructTypes.h"
#include "GentleBoostClassifier2.h"

double (*_featureCost)(const PerformanceMetrics& ) = &(GentleBoostClassifier2::featureCost);

bool operator<(const SearchResult& a, const SearchResult& b) {
	return a.value < b.value; 
}

bool operator<(const PerformanceMetrics& a, const PerformanceMetrics& b) {
	//	return GentleBoostCascadedClassifier::featureCost(a) > GentleBoostCascadedClassifier::featureCost(b); 
	return _featureCost(a) > _featureCost(b); 
}


bool operator<(const FeaturePerformance& a, const FeaturePerformance& b){
	//if (b.chisq < a.chisq) 
	return a.perf < b.perf; 
}
bool operator<(const FeaturePerformance2& a, const FeaturePerformance2& b){
	//if (b.chisq < a.chisq) 
	return a.perf < b.perf; 
}

void setFeatureCostFunction(double (*costFun)(const PerformanceMetrics&  )) {
	_featureCost = costFun; 
}
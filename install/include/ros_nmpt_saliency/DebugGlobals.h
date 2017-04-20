/*
 *  DebugGlobals.h
 *  OpenCV
 *
 *  Created by Nicholas Butko on 00/20/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __DEBUG_GLOBALS
#define __DEBUG_GLOBALS

#ifdef __GNUC__
#define DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: DEPRECATED messages suppressed for this compiler")
#define DEPRECATED(func) func
#endif



#define _ALL_DEBUG 0

#define _SEARCH_DEBUG       _ALL_DEBUG || 0

#define _POMDP_DEBUG        _ALL_DEBUG || 0

#define _SUBJECTIVE_DEBUG   _ALL_DEBUG || 0

#define _FEATS_DEBUG        _SEARCH_DEBUG || 0

#define _BOOST_DEBUG        _SEARCH_DEBUG || 0

#define _FEATURE_DEBUG      _FEATS_DEBUG || 0

#define _BOXFEATURE_DEBUG   _FEATS_DEBUG || 0 

#define _HAARFEATURE_DEBUG  _FEATS_DEBUG || 0 

#define _REGRESSOR_DEBUG    _FEATS_DEBUG || 0

#define _GENTLEBOOST_DEBUG  _BOOST_DEBUG || 0 

#define _CASCADE_DEBUG      _BOOST_DEBUG || 0 

#define _TRAINING_DEBUG     _BOOST_DEBUG || 0

#define _PATCHLIST_DEBUG    _BOOST_DEBUG || 0 

#define _DATASET_DEBUG      _BOOST_DEBUG || 0

#define _TRACKING_DEBUG     _BOOST_DEBUG || 0 

#define _RBF_DEBUG          _TRAINING_DEBUG || 0

#define _BLACKOUT_DEBUG     _PATCHLIST_DEBUG || 1

#define _MIPOMDP_DEBUG      _POMDP_DEBUG || 0

#define _IPP_DEBUG          _POMDP_DEBUG || 0

#define _IMAGEOFFSET_DEBUG  _SUBJECTIVE_DEBUG || 0

#define _IMM_DEBUG          _SUBJECTIVE_DEBUG || 0

#define _IPOMDP_DEBUG       _ALL_DEBUG || 0

#define _SALIENCE_DEBUG     _ALL_DEBUG || 0

#endif

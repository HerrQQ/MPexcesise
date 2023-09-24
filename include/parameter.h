#pragma once
// fixed value
#define MAXPOINTNUMBER 180 
#define STEPLENGTH 1
#define DEFALUTVALUE 100000
#define CYCLETIME 0.02
#define EPSILON 1e-6
#define CARWIDTH 2.018
#define CARRA2FRONT 3.45

// calibration  
#define LateralDiffThreshold 1. 
#define HeadingDiffThreshold 0.1
#define PMSwitch 1 //0

#define PreStepTimeGap 0.5
#define LatticeNumber 8
#define SpeedThreshold 5.


// cost relative threshold
#define MaxAccOffset 0.
#define MaxJerkOffset 0. 
#define OvershootOffset 0.
#define BordDistance 1.8
#define SafeDistace 0.

//cost weight 

#define TimeCost 0.3
#define JerkCost 1.
#define VelCost 1.
#define MaxJerkCost 5.
#define MaxAccCost 5.
#define OvershootCost 1.
#define BorderCost 100.

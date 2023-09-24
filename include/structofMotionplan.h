#pragma once
#include <vector>


typedef struct PathCoef_tag

{
    double c0;
    double c1;
    double c2;
    double c3;
} PathCoef;

typedef struct PathPara_tag

{
    float startPoint_x;
    float endPoint_x;
    float startPoint_y;
    float endPoint_y;
    int roadType;
    int validFlag;
} PathPara;

typedef struct Point_cart_tag

{
 float x;
 float y;
 float heading; 
 float kappa;
 float dkappa;
} Point_cart;

typedef struct Point_fret_tag

{
 float s;
 float l;
 float dl;
 float ddl; 
 float dddl;
 float sdot;
} Point_fre;

typedef struct FiveOrderPolyCoeff_tag

{
    float d0;
    float d1;
    float d2;
    float d3; 
    float d4;
    float d5;
} FiveOrderPolyCoeff;


typedef struct VehInfo_tag
{
    float vehSpeed;
    float yawRate;    
}VehInfo;




enum MPstatus 
{
    OFF =0,
    INIT =1,
    REINIT =2,
    NORMAL=3
};

enum PathPos 
{
    LeftCtrlLine =0,
    EgoCtrlLine =1,
    RightCtrlLine =2
};




typedef struct PathInfo_tag
{
    PathCoef pathCoef;
    PathPara pathPara;/* data */
    PathPos pathPos;    
}PathInfo;

typedef struct LatticeInfo_tag
{

    float cost_jerk;
    float cost_latteralVelocity;
    float cost_time;
    float cost_maxAccerelation;
    float cost_maxJerk;
    float cost_border;
    float cost_overShoot;
    float cost_sum;
    float sEnd;
    FiveOrderPolyCoeff fiveOrderPolyCoeff;

}LatticeInfo;






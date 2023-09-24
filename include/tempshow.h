#pragma once

typedef struct test_tag
{
 int a=0;
 int b=0;

}LCCDataflow;

typedef struct test_tag
{
 int a=0;
 int b=0;

}ACCDataflow;

typedef struct test_tag
{
 int a=0;
 int b=0;

}TSRDataflow;

typedef struct test_tag
{
 int a=0;
 int b=0;

}VehicalInfo;

typedef struct test_tag
{
 int a=0;
 int b=0;

}TraceInfo;



typedef struct test_tag
{
    LCCDataflow lccdataflow;
    ACCDataflow Accdataflow;
    TSRDataflow tsrdataflow;

}ADASDataflow;

class ACC{

};
class LCC{

};
class TSR{

};



class xxx {

public:

xxx()=default;
~xxx()=default;

private:
//interface
LCCDataflow lccDataflow;
ACCDataflow accDataflow;
TSRDataflow tsrDataflow;
// input from outside 
const VehicalInfo* vehicalInfo ;
const TraceInfo* TraceInfo;
// sub module
LCC lcc;
ACC acc;
TSR tsr;

};
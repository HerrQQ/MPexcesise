#pragma once 
#include "structofMotionplan.h"
#include "parameter.h"
#include <vector>
#include "glog/logging.h"
#include "Eigen/Eigen"

namespace mp
{

    class MotionPlan
    {
        public:

        MotionPlan()
        {
            LOG(INFO)<<"*******************MP start*******************";
        }
        ~ MotionPlan()
        {
            LOG(INFO)<<"*******************MP end*******************"; 
        }
        void init(const VehInfo * const vehInfo,const std::vector<PathInfo>* const CentralLines,const PathPos& changeCommoend);
//main 
        std::vector <Point_cart> run_mp();

        private:
//sub
        void fPathselection();
        void fPathdiscretion();
        void fFindNearstPoint();
        void fCal_S();
        void fCal_PlanningStartPoint();
        void fCartesian2Frenet(const Point_cart& point_v,const Point_cart& point_p,Point_fre& point_fre_v_);
        void fJudgeMPstatus();
        void fFindMPStartPoint();
        void fLatticeInit();
        void fCostCal(LatticeInfo & poly5Order) ;
        void fFrenet2Cartesian();
        //void fPolyfit();
        

        void fValueReset();



        //debug info

        int pathEnd_qkf;
        std::vector <Point_cart> centralPath_qkf;




        private:
        std::vector <Point_cart> centralPath_discreted_ ;
        std::vector <Point_fre> centralPath_inFrenet_ ;
        std::vector <PathInfo> pathPool_ ;
        std::vector <LatticeInfo> line5Orders_;
        const std::vector<PathInfo>* centralLines_;//nok wait for input data
        const PathInfo* refCentralLine_;
        PathPos changeCommend_;
        int pathLength_real_;//ok
        int pointIndex_nearest_;//ok
        Point_fre point_fre_v_;//ok
        Point_fre point_start_;//ok
        Point_cart point_nearest_; //ok
        const VehInfo * vehInfo_;//input
        MPstatus MP_status_; // nok
        LatticeInfo bestLine5Order_;//ok
        std::vector <Point_fre> bestLine5Order_discreated_inFre_;//ok
        std::vector <Point_cart> ans_final_;
        bool bSwtich_ =PMSwitch;


        //output 

        bool valid_mp_ =false;

        std::vector <Point_cart> point_output_;

        float pathlength_output_;





    };




}
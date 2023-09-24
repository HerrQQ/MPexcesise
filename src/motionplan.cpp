# include "motionplan.h"
# include <iostream>
#include <cmath>

using namespace std;//temp use


namespace mp{


    void MotionPlan:: init(const VehInfo * vehInfo,const std::vector<PathInfo>* const centralLines, const PathPos& changeCommand  )
    {
       this->vehInfo_= vehInfo;
       this->centralLines_=centralLines;
       this->changeCommend_=changeCommand;
    }

    // main 

    std::vector <Point_cart> MotionPlan:: run_mp()
    {   google::LogSeverity severity = FLAGS_v;
        LOG(INFO) << "Current log severity level: " << severity;

        //Point_cart point_t ={DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE};
        //(MAXPOINTNUMBER,point_t);
        //int nearestIndex =DEFALUTVALUE;
        fPathselection();
        fPathdiscretion();
        fFindNearstPoint();
        LOG(INFO)<<"test run here";
        fCal_S();
        LOG(INFO)<<"test run here";
        float kappa_t=DEFALUTVALUE;
        if (vehInfo_->vehSpeed>1.f)
        {
            kappa_t= vehInfo_->yawRate/vehInfo_->vehSpeed;
        }
        else 
        {
            LOG(ERROR)<<"Speed is too low";
            kappa_t=0.0001;
        }
        
        Point_cart point_v ={0.f,0.f,0.f,kappa_t,0.f};

        fCartesian2Frenet(point_v, point_nearest_, point_fre_v_);
        // LOG(ERROR)<<"point_fre_v_.l: "<<point_fre_v_.l;
        // LOG(ERROR)<<"point_fre_v_.s: "<<point_fre_v_.s;

        LOG(INFO)<<"test run here";
        fFindMPStartPoint();
        LOG(INFO)<<"test run here";
        fLatticeInit();
        LOG(INFO)<<"test run here";
        int index=0;
        double maxCost=DEFALUTVALUE;
        for (auto& line:line5Orders_)
        {
            static int index_t=0;

            fCostCal(line);
            if (line.cost_sum<maxCost)
            {
               maxCost= line.cost_sum;
               index=index_t;
            }  
            index_t++;          
        }
        bestLine5Order_=line5Orders_[index];// no pointer
        LOG(INFO)<<"test run here";
        fFrenet2Cartesian();
        LOG(INFO)<<"test run here";
        return ans_final_;

    
    }
    //sub

    void MotionPlan:: fPathselection()
    {
        if (centralLines_)
        {
            for (const auto&  x: *centralLines_)
            {
                if (changeCommend_==x.pathPos)
                {
                    refCentralLine_=&x;
                }

            }
        }
    }

    void MotionPlan:: fPathdiscretion()
    {
        Point_cart point_t = {0.f,0.f,0.f,0.f};
        float c0_t=refCentralLine_->pathCoef.c0;
        float c1_t=refCentralLine_->pathCoef.c1;
        float c2_t=refCentralLine_->pathCoef.c2;
        float c3_t=refCentralLine_->pathCoef.c3;
        float x_t=0.f;
        bool validLane=refCentralLine_->pathPara.validFlag;
        float pathEnd=refCentralLine_->pathPara.endPoint_x;
        pathLength_real_=std::min(static_cast<int>(MAXPOINTNUMBER*STEPLENGTH),static_cast<int>(std::floor(pathEnd)));
        

        if (false==validLane||pathLength_real_<10)
        {
            LOG(ERROR)<<"path to short";
            return ;
        }
        else
        {
            centralPath_discreted_.clear();
            for (int i=0; i<=pathLength_real_;++i)
            {
                Point_cart point_t ={DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE};
                
                centralPath_discreted_.push_back(point_t);
                centralPath_discreted_.at(i).x=i*STEPLENGTH;
                x_t=i*STEPLENGTH;
                centralPath_discreted_.at(i).y=c0_t+c1_t*x_t+c2_t*pow(x_t,2)+c3_t*pow(x_t,3);
                centralPath_discreted_.at(i).heading=atan(c1_t+2*c2_t*x_t+3*c3_t*pow(x_t,2));
                centralPath_discreted_.at(i).kappa=2*c2_t+6*c3_t*x_t;
                centralPath_discreted_.at(i).dkappa=6*c3_t;
            }          
        }
        pathEnd_qkf=pathLength_real_;
        centralPath_qkf=centralPath_discreted_;
    }


    void MotionPlan:: fFindNearstPoint()
    {
        // find index 
        int size_t =pathLength_real_;
        float distance=DEFALUTVALUE;
        for (int i=0;i<=size_t;++i)
        {
            float x_t=i;
            float y_t=centralPath_discreted_.at(i).y;
            float distance_t=sqrt(pow(x_t,2)+pow(y_t,2));
            if (distance_t<distance)
            {
                distance=distance_t;
                pointIndex_nearest_=i;
            }
        }
        if (distance!=DEFALUTVALUE)
        {
            point_nearest_=centralPath_discreted_.at(pointIndex_nearest_);
        }
    }
    void MotionPlan:: fCal_S()
    {
        Point_fre point_t {0.f,0.f,0.f,0.f,0.f,0.f};
        centralPath_inFrenet_.clear();
        centralPath_inFrenet_.resize((pathLength_real_+1),point_t);
        //centralPath_inFrenet_.at(0).s=0.f;
        for (int i=1;i<=pathLength_real_;++i)
        {

            float x_t=i;
            float y_t=centralPath_discreted_.at(i).y;
            float x_told=i-1;
            float y_told=centralPath_discreted_.at(i-1).y;

            float distance_t=sqrt(1+pow(y_t-y_told,2));

            centralPath_inFrenet_.at(i).s=centralPath_inFrenet_.at(i-1).s+distance_t;
        }
        for (auto& x :centralPath_inFrenet_)
        {
            x.s=x.s-centralPath_inFrenet_.at(pointIndex_nearest_).s;
        }

    }

    void MotionPlan :: fCartesian2Frenet(const Point_cart& point_v,const Point_cart& point_p,Point_fre& point_fre_v)
    {
        double l =DEFALUTVALUE;
        double dl=DEFALUTVALUE;
        double dll=DEFALUTVALUE;
        double sdot=DEFALUTVALUE;
        double X_v =point_v.x;
        double Y_v = point_v.y;
        double X_p =point_p.x;
        double Y_p =point_p.y;
        double theta_v=point_v.heading;
        double theta_p=point_p.heading;
        double kappa_v=point_v.kappa;
        double kappa_p=point_p.kappa;
        double dKappa_v=point_v.dkappa;
        double dKappa_p=point_p.dkappa;   
        if (1)
        {
            double dx =X_v-X_p;
            double dy =Y_v-Y_p;
            double cos_p=cos(theta_p);
            double sin_p=sin(theta_p);
            int sign = (dy*cos_p-dx*sin_p)>=0.f?1:-1; 
            l=sign*sqrt(pow(dx,2)+pow(dy,2));
            double delta_theta =theta_v-theta_p;
            double tan_delta=tan(delta_theta);
            double cos_delta=cos(delta_theta);
            double one_kl=1-kappa_p*l;
            dl=one_kl*tan_delta;
            double temp=dKappa_p*l+kappa_p*dl;
            dll=(-temp*tan_delta+one_kl/cos_delta/cos_delta*(kappa_v*one_kl/cos_delta-kappa_p));
            sdot=vehInfo_->vehSpeed*cos_delta/one_kl;
            
        }
        else{
            int sign = ((X_v -X_p)*cos(theta_p)+(Y_v-Y_p)*cos(theta_v))>=0.f?1:-1;
            l =sign *sqrt(pow((X_v -X_p),2)+pow((Y_v-Y_p),2));
            dl=(1-kappa_p*l)*tan(theta_v-theta_p);
            dll=-( dKappa_p*l+kappa_p*dl)*tan(theta_v-theta_p)+((1-kappa_p*l))/pow(cos(theta_v-theta_p),2)*
            ((1-kappa_p*l)/cos(theta_v-theta_p)*kappa_v-kappa_p);
            sdot=vehInfo_->vehSpeed*cos(theta_v-theta_p)/(1-dKappa_p*l);
        }    

        point_fre_v.l=l;
        point_fre_v.dl=dl;
        point_fre_v.ddl=dll;
        point_fre_v.sdot=sdot;
    }



    void MotionPlan:: fJudgeMPstatus()// index to be define
    {
        if (1==bSwtich_)
        {
            MP_status_=NORMAL;
            if (point_output_.at(0).y>LateralDiffThreshold&&
            point_output_.at(0).heading>HeadingDiffThreshold)
                {
                    MP_status_=REINIT;
                }
            else 
                {

                }
        }        
    }

    void MotionPlan::  fFindMPStartPoint()
    {
        double d0=bestLine5Order_.fiveOrderPolyCoeff.d0;
        double d1=bestLine5Order_.fiveOrderPolyCoeff.d1;
        double d2=bestLine5Order_.fiveOrderPolyCoeff.d2;
        double d3=bestLine5Order_.fiveOrderPolyCoeff.d3;
        double d4=bestLine5Order_.fiveOrderPolyCoeff.d4;
        double d5=bestLine5Order_.fiveOrderPolyCoeff.d5;
        double s=CYCLETIME*point_fre_v_.sdot;
        if (NORMAL==MP_status_)
        {
            point_start_.s=s;
            point_start_.l=d0+d1*s+d2*pow(s,2)+d3*pow(s,3)+d4*pow(s,4)+d5*pow(s,5);
            point_start_.dl=d1+2*d2*s+3*d3*pow(s,2)+4*d4*pow(s,3)+5*d5*pow(s,4);
            point_start_.ddl=2*d2+6*d3*s+12*d4*pow(s,2)+20*d5*pow(s,3);
        }
        else if (NORMAL!=MP_status_)
        {
            point_start_=point_fre_v_;
            // LOG(ERROR)<<"point_start_.l "<<point_start_.l;
            // LOG(ERROR)<<"point_start_.l "<<point_start_.s;
            // LOG(ERROR)<<"point_start_.l "<<point_start_.sdot;
            // LOG(ERROR)<<"point_start_.l "<<point_start_.dl;
            // LOG(ERROR)<<"point_start_.l "<<point_start_.ddl;
        }
        else
        {
            point_start_={DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE};
        }


    }
    void MotionPlan::  fLatticeInit()
    {
        
        float speed_t=std::max(5.f,point_fre_v_.sdot);
        //LOG(ERROR)<<"point_fre_v_.sdot"<<point_fre_v_.sdot;
        for (int i=1;i<=LatticeNumber;i++)
        {
            LatticeInfo Path={DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE
            ,DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,{DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE,
            DEFALUTVALUE,DEFALUTVALUE,DEFALUTVALUE}};
    
            Path.sEnd=i*speed_t*PreStepTimeGap;

            // LOG(ERROR)<<"Path.sEnd "<<Path.sEnd;

            Path.fiveOrderPolyCoeff.d0=point_start_.l;
            Path.fiveOrderPolyCoeff.d1=point_start_.dl;
            Path.fiveOrderPolyCoeff.d2=point_start_.ddl/2;
            Path.fiveOrderPolyCoeff.d3=-(10*point_start_.l)/pow(Path.sEnd,3)-(3*point_start_.ddl)/(2*Path.sEnd)-(6*point_start_.dl)/pow(Path.sEnd,2);
            Path.fiveOrderPolyCoeff.d4=(15*point_start_.l)/pow(Path.sEnd,4)+(3*point_start_.ddl)/(2*pow(Path.sEnd,2))+(8*point_start_.dl)/pow(Path.sEnd,3);
            Path.fiveOrderPolyCoeff.d5=-(6*point_start_.l)/pow(Path.sEnd,5)-(point_start_.ddl)/(2*pow(Path.sEnd,3))-(3*point_start_.dl)/pow(Path.sEnd,4);
            line5Orders_.emplace_back(Path);
            // LOG(ERROR)<<"Path.d0"<<Path.fiveOrderPolyCoeff.d0;
            // LOG(ERROR)<<"Path.d1"<<Path.fiveOrderPolyCoeff.d1;
            // LOG(ERROR)<<"Path.d2"<<Path.fiveOrderPolyCoeff.d2;
            // LOG(ERROR)<<"Path.d3"<<Path.fiveOrderPolyCoeff.d3;
            // LOG(ERROR)<<"Path.d4"<<Path.fiveOrderPolyCoeff.d4;
            // LOG(ERROR)<<"Path.d5"<<Path.fiveOrderPolyCoeff.d5;
            // LOG(ERROR)<<"Path.sEnd"<<Path.sEnd;  
        }
    }

    void MotionPlan:: fCostCal(LatticeInfo & poly5Order) // 8 times
    {
        float time =DEFALUTVALUE;
        float jerk =DEFALUTVALUE;
        float speed = DEFALUTVALUE;
        float maxAcc =DEFALUTVALUE;
        float maxJerk=DEFALUTVALUE;
        float d0=poly5Order.fiveOrderPolyCoeff.d0;
        float d1=poly5Order.fiveOrderPolyCoeff.d1;
        float d2=poly5Order.fiveOrderPolyCoeff.d2;
        float d3=poly5Order.fiveOrderPolyCoeff.d3; 
        float d4=poly5Order.fiveOrderPolyCoeff.d4;
        float d5=poly5Order.fiveOrderPolyCoeff.d5;
        float sdot=point_fre_v_.sdot;
        float sEnd=poly5Order.sEnd;

        if (sEnd==0)
        {
            LOG(ERROR)<<"sEnd=0";
            return;
        }

        //1-1 timecost
        if (sdot>SpeedThreshold)
        {
            time=sEnd/sdot;
        }
        else{
            LOG(ERROR)<<" speed to low";
        }

        poly5Order.cost_time= TimeCost* time;
        //1-2 jerkcost
        double dEnd=d0+d1*sEnd+d2*pow(sEnd,2)+d3*pow(sEnd,3)+d4*pow(sEnd,4)+d5*pow(sEnd,5);
        poly5Order.cost_jerk=JerkCost*(-pow(point_fre_v_.sdot,5)*(12*d3*d2-24*d4*d1+120*d5*(d0-dEnd)));

        //1-3 velocity cost 
        poly5Order.cost_latteralVelocity = VelCost*(1/sdot*(pow(d1,2)*sEnd*pow(sdot,2) + 1/3*pow(sEnd,3)*pow(sdot,2)*(4*pow(d2,2) + 6*d1*d3) 
        + 1/7*pow(sEnd,7)*pow(sdot,2)*(16*pow(d4,2) + 30*d3*d5) 
        + 1/4*pow(sEnd,4)*pow(sdot,2)*(8*d1*d4 + 12*d2*d3) 
        + 1/6*pow(sEnd,6)*pow(sdot,2)*(20*d2*d5 + 24*d3*d4)
        + 1/9*25*pow(d5,2)*pow(sEnd,9)*pow(sdot,2) 
        + 1/5*pow(sEnd,5)*pow(sdot,2)*(9*pow(d3,2) + 10*d1*d5 +16*d2*d4) 
        + 2*d1*d2*pow(sEnd,2)*pow(sdot,2) + 5*d4*d5*pow(sEnd,8)*pow(sdot,2)));

        //1-4 max lateral acc cost 
        //root of ddd(l) 50d5xx 24d4x 6d3
        double a= 60*d5;
        double b=24*d4;
        double c=6*d3;
        double delta=b*b-4*a*c;
        double ans_1=0.0,ans_2=0.0;

        if (fabs(a)<EPSILON&&fabs(b)<EPSILON)
        {
            ans_1=ans_2=sEnd;
        }
        else if (fabs(a)<EPSILON)
        {
            ans_1=-c/b;
            ans_2=sEnd;//could be zero but it is processed later 
        }
        else 
        {
            if (delta<0.)
            {
                ans_1=ans_2=sEnd;
            }
            else 
            {
                ans_1=(-b-sqrt(delta))/(-2*a);
                ans_2=(-b+sqrt(delta))/(-2*a);
            }
        }
        //1-4-1 max curvature
        double max_cur1=0.f;
        double max_cur2=0.f;
        if (ans_1>0.&&ans_1<sEnd)
        {
            max_cur1=2*d2+6*d3*ans_1+12*d4*pow(ans_1,2)+20*d5*pow(ans_1,3);
        }
        if (ans_2>0.&&ans_2<sEnd)
        {
            max_cur2=2*d2+6*d3*ans_2+12*d4*pow(ans_2,2)+20*d5*pow(ans_2,3);
        }        
        double max_cur=max(max_cur1,max_cur2);
        double a_max=vehInfo_->vehSpeed*vehInfo_->vehSpeed*(2*refCentralLine_->pathCoef.c2);// ? c2 thought as a constan here
        double temp_t=a_max-MaxAccOffset;
        poly5Order.cost_maxAccerelation=MaxAccCost*max(temp_t,0.);
        


        // float root1=0.f;
        // float root2=sEnd;
        // float root3=sEnd;
        // if (d5!=0.f)
        // {
        //     root3=-1/5*d4*d5;
        // }
        // else 
        // {

        // }
        // 1-5 find max jerk value 
        // 100d5 24d4 
        float root1=0.f;
        float root2=sEnd;
        float root3=sEnd;
        if (d5!=0.f)
        {
            root3=-1/5*d4*d5;
        }
        else 
        {
            root3=sEnd;
        }

        float jerk1=fabsf(6*d3);
        float jerk2=fabsf(60*d5*pow(root2,2)+24*d4*root2+6*d3);
        float jerk3=0.f;
        if (root3>0.f&&root3<sEnd)
        {
            jerk3=fabsf(60*d5*pow(root3,2)+24*d4*root3+6*d3);
        }
        poly5Order.cost_maxJerk= MaxJerkCost*(pow(vehInfo_->vehSpeed,3)*max (jerk1,max(jerk2,jerk3))-MaxJerkOffset,0.f);
        // 1-6 overshoot cost 
        // reform d'
        double a_over=5*d5;
        double b_over=4*d4+10*d5*sEnd;
        double c_over=d1/sEnd/sEnd;
        double delta_over=b_over*b_over-4*a_over*c_over;
        double ans_over1=sEnd, ans_over2=sEnd;
        if (fabs(a_over)<EPSILON&&fabs(b_over)<EPSILON)
        {
            //do nothing
        }
        else if (fabs(a_over)<EPSILON)
        {
            ans_over1=-c_over/b_over;
            ans_over2=sEnd;
        }
        else
        {
            if (delta_over<0.f)
            {
                //do nothing
            }
            else 
            {
                ans_over1=(-b_over+(sqrt(delta_over)))/(2*a_over);
                ans_over2=(-b_over-(sqrt(delta_over)))/(2*a_over);
            }
        }
        // cal min max value of d

        double maxJerk1=0.0,maxJerk2=0.0;
        if (ans_over1>0.0&&ans_over1<sEnd)
        {
            maxJerk1=d0+d1*ans_over1+d2*pow(ans_over1,2)+d3*pow(ans_over1,3)+d4*pow(ans_over1,4)+d5*pow(ans_over1,5);
        }
        else 
        {
            //do nothing
        }
        if (ans_over2>0.0&&ans_over2<sEnd)
        {
            maxJerk2=d0+d1*ans_over2+d2*pow(ans_over2,2)+d3*pow(ans_over2,3)+d4*pow(ans_over2,4)+d5*pow(ans_over2,5);
        }
        else 
        {
            //do nothing
        }
        // judge sign 
        double Overshoot1=0.0,Overshoot2=0.0;
        if (maxJerk1*d0<0.0)
        {
            Overshoot1=abs(maxJerk1)-OvershootOffset;
            Overshoot1=max(0.0,Overshoot1);
        }
        else 
        {
            Overshoot1=0.0;

        }
        if (maxJerk2*d0<0.0)
        {
            Overshoot2=abs(maxJerk2)-OvershootOffset;
            Overshoot2=max(0.0,Overshoot2);
        }
        else 
        {
            Overshoot2=0.0;
        }
        poly5Order.cost_overShoot=OvershootCost*max(Overshoot1,Overshoot2);
        //1-7 cal exceed dis
        double c0=refCentralLine_->pathCoef.c0;
        double c1=refCentralLine_->pathCoef.c1;
        double c2=refCentralLine_->pathCoef.c2;
        double c3=refCentralLine_->pathCoef.c3;
        double carFrontPos=0.0;
        carFrontPos=c0+CARRA2FRONT*c1+pow(CARRA2FRONT,2)*c2+pow(CARRA2FRONT,3)*c3;
        double leftDis=0.0, rightDis=0.0;
        leftDis=BordDistance-carFrontPos;
        rightDis=-BordDistance-carFrontPos;
        // border check
        double ans_border1=sEnd,ans_border2=sEnd;
        double nearestAns=sEnd;
        double nearestMax=0.0;
        double ref2Veh=0.5*CARWIDTH+SafeDistace;
        double realLimitBorder=0.;// carwidth length heading are introduced
        if (ans_over1>0.0&&ans_over1<sEnd)
        {
            ans_border1=ans_over1;
        }
        else 
        {
            //do nothing
        }
        if (ans_over2>0.0&&ans_over2<sEnd)
        {
            ans_border2=ans_over2;
        }
        else 
        {
            //do nothing
        }
        nearestAns=min(ans_border1,ans_border2);
        nearestMax=d0+d1*nearestAns+d2*pow(nearestAns,2)+d3*pow(nearestAns,3)+d4*pow(nearestAns,4)+d5*pow(nearestAns,5);
        if (nearestMax>0.)
        {
            realLimitBorder=leftDis-ref2Veh;
            poly5Order.cost_border=BorderCost*max((nearestMax-realLimitBorder),0.);
        }
        else 
        {
            realLimitBorder=rightDis+ref2Veh;
            poly5Order.cost_border=BorderCost*max((realLimitBorder-nearestMax),0.);
        }

        poly5Order.cost_sum=poly5Order.cost_time+poly5Order.cost_border+poly5Order.cost_jerk+poly5Order.cost_maxAccerelation
        +poly5Order.cost_maxJerk+poly5Order.cost_latteralVelocity+poly5Order.cost_overShoot;
        LOG(ERROR)<<"poly5Order.cost_time"<<poly5Order.cost_time;
        LOG(ERROR)<<"poly5Order.cost_border"<<poly5Order.cost_border;
        LOG(ERROR)<<"poly5Order.cost_jerk"<<poly5Order.cost_jerk;
        LOG(ERROR)<<"poly5Order.cost_maxAccerelation"<<poly5Order.cost_maxAccerelation;
        LOG(ERROR)<<"poly5Order.cost_maxJerk"<<poly5Order.cost_maxJerk;
        LOG(ERROR)<<"poly5Order.cost_latteralVelocity"<<poly5Order.cost_latteralVelocity;
        LOG(ERROR)<<"poly5Order.cost_overShoot"<<poly5Order.cost_overShoot;
        LOG(ERROR)<<"cost_sum"<<poly5Order.cost_sum;
    }

    void MotionPlan::  fFrenet2Cartesian()
    {
        // 0 discrete the best trace 
        double d0=bestLine5Order_.fiveOrderPolyCoeff.d0;
        double d1=bestLine5Order_.fiveOrderPolyCoeff.d1;
        double d2=bestLine5Order_.fiveOrderPolyCoeff.d2;
        double d3=bestLine5Order_.fiveOrderPolyCoeff.d3;
        double d4=bestLine5Order_.fiveOrderPolyCoeff.d4;
        double d5=bestLine5Order_.fiveOrderPolyCoeff.d5;
        double sEnd=bestLine5Order_.sEnd;
        // LOG(ERROR)<<"d0 "<<d0;
        // LOG(ERROR)<<"d1 "<<d1;
        // LOG(ERROR)<<"d2 "<<d2;
        // LOG(ERROR)<<"d3 "<<d3;
        // LOG(ERROR)<<"d4 "<<d4;
        // LOG(ERROR)<<"d5 "<<d5;
        //  LOG(ERROR)<<"bestLine5Order_.sEnd "<<bestLine5Order_.sEnd;
        int size_centralLine=centralPath_inFrenet_.size();
        int size_5orderLine=180;
        std::vector<int > match_index;
        for (int i=0;i<=size_5orderLine;++i)
        {
            // 0-1 cal discrete points 
            Point_fre point_t;
            if (i<=sEnd)
            {
                point_t.s=i;
                point_t.l=d0+d1*i+d2*pow(i,2)+d3*pow(i,3)+d4*pow(i,4)+d5*pow(i,5);
                point_t.dl=d1+2*d2*i+3*d3*pow(i,2)+4*d4*pow(i,3)+5*d5*pow(i,4);
                point_t.ddl=2*d2+6*d3*i+12*d4*pow(i,2)+20*d5*pow(i,3);
                point_t.dddl=6*d3+24*d4*i+60*d5*pow(i,2); 
            }
            else 
            {
                point_t.s=sEnd;
                point_t.l=d0+d1*sEnd+d2*pow(sEnd,2)+d3*pow(sEnd,3)+d4*pow(sEnd,4)+d5*pow(sEnd,5);
                point_t.dl=d1+2*d2*sEnd+3*d3*pow(sEnd,2)+4*d4*pow(sEnd,3)+5*d5*pow(sEnd,4);
                point_t.ddl=2*d2+6*d3*sEnd+12*d4*pow(sEnd,2)+20*d5*pow(sEnd,3);
                point_t.dddl=6*d3+24*d4*sEnd+60*d5*pow(sEnd,2); 
            }
            bestLine5Order_discreated_inFre_.emplace_back(point_t);
            // 0-2 find match index
            double min_t=DEFALUTVALUE;
            int index=0;
            for (int j=0;j<size_centralLine;++j)
            {
                
                double temp=abs(centralPath_inFrenet_[j].s-point_t.s);
               // LOG(ERROR)<<"point_t.s "<<size_5orderLine;
                if (temp<min_t)
                {
                    min_t=temp;
                    index =j;
                }
            }
//LOG(ERROR)<<"match_index.emplace_back(index) "<<index;
            match_index.emplace_back(index);        
        }
        // 0-3 get matched info in vcs 
        // 181 points in total
        std::vector <Point_cart> matched_vsc;
        for (int i=0;i<=size_5orderLine;++i)
        { 
            Point_cart point_t={0.f,0.f,0.f,0.f,0.f };
            point_t.x       =centralPath_discreted_.at(match_index.at(i)).x;
            point_t.y       =centralPath_discreted_.at(match_index.at(i)).y;
            point_t.heading =centralPath_discreted_.at(match_index.at(i)).heading;
            point_t.kappa   =centralPath_discreted_.at(match_index.at(i)).kappa;
            point_t.dkappa  =centralPath_discreted_.at(match_index.at(i)).dkappa;
            matched_vsc.emplace_back(point_t);
        }
        // frenet 2 vcs 
        // now all needed infos are collected : matched_vsc bestLine5Order_discreated_inFre_


        for (int i=0;i<=size_5orderLine;++i)
        {
            Point_cart point_ans={0.f,0.f,0.f,0.f,0.f };

            double l=bestLine5Order_discreated_inFre_[i].l;
            double dl=bestLine5Order_discreated_inFre_[i].dl;
            double ddl=bestLine5Order_discreated_inFre_[i].ddl;
            double x=matched_vsc[i].x;
            double y=matched_vsc[i].y;
            double heading=matched_vsc[i].heading;
            double kappa=matched_vsc[i].kappa;
            double dkappa=matched_vsc[i].dkappa;

            double one_kl=1-kappa*l;
            if (one_kl<EPSILON)
            {
                LOG(ERROR)<<"one_kl is too small";
                one_kl=EPSILON;
            }
            double tan_delta_theta=dl/one_kl;
            double delta_theta=atan(tan_delta_theta); 
            double cos_delta_theta=cos(delta_theta);


            //2-1 cal 
            point_ans.x=x-l*sin(heading);
            //LOG(ERROR)<<"test run here "<<i<<" x: "<<x;
            point_ans.y=y+l*cos(heading);
            double heading_t=heading+delta_theta;
            // angle needs to be norminized
            heading_t = std::fmod(heading_t, 2.0 * M_PI); 
            if (heading_t < -M_PI) {
                heading_t += (2.0 * M_PI);
            } else if (heading_t >= M_PI) { 
                heading_t -= (2.0 * M_PI);
            }
            point_ans.heading=heading_t;
            point_ans.kappa=((ddl+((dkappa*l+kappa*dl))*tan_delta_theta)*cos_delta_theta*cos_delta_theta/(one_kl)
            +kappa)*cos_delta_theta/one_kl;

            ans_final_.emplace_back(point_ans);
            //LOG(ERROR)<<"point_ans.x"<<point_ans.x;
            //LOG(ERROR)<<"point_ans.y"<<point_ans.y;
        }       
    }


}
#include <iostream>
// #include "string.h"
#include "motionplan.h"

int main (int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::GLOG_ERROR); 
    FLAGS_colorlogtostderr = true;  

    std::cout<<"******************* MP test start*******************"<<std::endl;
    mp::MotionPlan planner;
    //test data
    VehInfo vehInfo={20.,0.01};
    PathInfo pathInfo_l,pathInfo_r,pathInfo_ego;
    std::vector<PathInfo> pathInfos;
    std::vector <Point_cart> ans_test;
    PathPos changeCommand=EgoCtrlLine;
    memset(&pathInfo_l,0,sizeof(PathInfo));
    memset(&pathInfo_r,0,sizeof(PathInfo));
    memset(&pathInfo_ego,0,sizeof(PathInfo));

    pathInfo_ego.pathPara.validFlag=1;
    pathInfo_ego.pathPara.endPoint_x=80;
    pathInfo_ego.pathCoef.c0=0.5;
    pathInfo_ego.pathCoef.c1=0.02;
    pathInfo_ego.pathCoef.c2=5e-3;
    pathInfo_ego.pathCoef.c3=1e-6;
    pathInfo_ego.pathPos=EgoCtrlLine;
    pathInfos.emplace_back(pathInfo_ego);
    pathInfo_l.pathPara.validFlag=1;
    pathInfo_l.pathPara.endPoint_x=80;
    pathInfo_l.pathCoef.c0=4;
    pathInfo_l.pathCoef.c1=0.02;
    pathInfo_l.pathCoef.c2=5e-3;
    pathInfo_l.pathCoef.c3=1e-6;
    pathInfo_l.pathPos=LeftCtrlLine;
    pathInfos.emplace_back(pathInfo_l);
    pathInfo_r.pathPara.validFlag=1;
    pathInfo_r.pathPara.endPoint_x=80;
    pathInfo_r.pathCoef.c0=-3;
    pathInfo_r.pathCoef.c1=0.02;
    pathInfo_r.pathCoef.c2=5e-3;
    pathInfo_r.pathCoef.c3=1e-6;
    pathInfo_r.pathPos=RightCtrlLine;
    pathInfos.emplace_back(pathInfo_r);



    planner.init(&vehInfo,&pathInfos,changeCommand);

    ans_test=planner.run_mp();

    for (const auto& ans:ans_test)
    {
        //LOG(ERROR)<<ans.x<<",";
    }
    LOG(ERROR)<<"*********************";
    for (const auto& ans:ans_test)
    {
        //LOG(ERROR)<<ans.y<<",";
    }



    google::ShutdownGoogleLogging();


}

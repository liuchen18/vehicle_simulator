#include "obstacle.h"


namespace collision
{
obstacle::obstacle(const int num,std::vector<OBB2d> *obstacles_ptr):obstacle_num_(num),obstacles_ptr_(obstacles_ptr){
    if(num==0){
        obstacle_num_=0;
    }
    else if(num==1){
        obstacle1=OBB(point2d(0.0,20.0),0.0,5.0,4.0) //center:(0,5),angle:0,length:5,width:4
        obstacles_ptr_->pushback(obstacle1);
    }
    else if(num==2){
        obstacle1=OBB(point2d(0.0,20.0),0.0,5.0,4.0) //center:(0,20),angle:0,length:5,width:4
        obstacle2=OBB(point2d(0.0,28.0),0.0,5.0,4.0) //center:(0,28),angle:0,length:5,width:4
        obstacles_ptr_->pushback(obstacle1);
        obstacles_ptr_->pushback(obstacle2);
    }
    else if(num==3){
        obstacle1=OBB(point2d(0.0,20.0),0.0,5.0,4.0) //center:(0,20),angle:0,length:5,width:4
        obstacle2=OBB(point2d(0.0,28.0),0.0,5.0,4.0) //center:(0,28),angle:0,length:5,width:4
        obstacle3=OBB(point2d(-1,24.0),0.0,1.0,8.0) //center:(-1,24),angle:0,length:1,width:8
        obstacles_ptr_->pushback(obstacle1);
        obstacles_ptr_->pushback(obstacle2);
        obstacles_ptr_->pushback(obstacle3);
    }
    
}
obstacle::~obstacle(){
    delete obstacles_ptr_;
    obstacles_ptr_=nullptr;
}
}

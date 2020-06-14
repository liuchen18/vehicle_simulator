#include "point2d.h"
#include "linesegment2d.h"
#include <iostream>
#include "AABB2d.h"
#include "OBB2d.h"

int main(){
    collision::point2d point1(0,0);
    collision::point2d point2(10,10);
    collision::point2d point3(20,10);
    collision::linesegment2d lineseg(point2,point3);

    collision::OBB2d box1(point1,0,10,10),box2(point2,0,100,100);

    std::cout<<"box collision test: "<<box1.has_overlap(box2)<<std::endl;

    std::cout<<"box distance test: "<<box1.distance_to_box(box2)<<std::endl;

    std::cout<<"point distance test: "<<box1.distance_to_point(point2)<<std::endl;
    std::cout<<"lineseg distance test: "<<box1.distance_to_lineseg(lineseg)<<std::endl;

    /*
    std::cout<<point1.distance_to_point(point2)<<std::endl;

    collision::AABB2d box1(point1,point2);
    collision::point2d point3(20,20);
    std::cout<<"box distance test: "<<box1.distance_to_point(point3)<<std::endl;

    collision::point2d point4(5,5);
    std::cout<<"in box test: "<<box1.is_point_in(point3)<<" "<<box1.is_point_in(point4)<<std::endl;
    */
    /*collision::linesegment2d lineseg(point1,point2);
    collision::point2d point3(10,0);
    std::cout<<"distance to point test: "<<lineseg.distance_to_point(point3)<<std::endl;
    std::cout<<"inline test: "<<lineseg.is_point_in(collision::point2d(5,5))<<std::endl;

    collision::point2d point4(0,10);
    collision::linesegment2d lineseg2(point3,point4);
    std::cout<<"intersect test: "<<lineseg.has_intersect(lineseg2)<<std::endl;
    */
    return 0;
}
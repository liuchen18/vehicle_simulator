#include "point2d.h"
#include "linesegment2d.h"
#include <iostream>

int main(){
    collosion::point2d point1(0,0);
    collosion::point2d point2(10,10);
    std::cout<<point1.distance_to_point(point2)<<std::endl;

    collosion::linesegment2d lineseg(point1,point2);
    collosion::point2d point3(10,0);
    std::cout<<"distance to point test: "<<lineseg.distance_to_point(point3)<<std::endl;
    std::cout<<"inline test: "<<lineseg.is_point_in(collosion::point2d(5,5))<<std::endl;

    collosion::point2d point4(0,10);
    collosion::linesegment2d lineseg2(point3,point4);
    std::cout<<"intersect test: "<<lineseg.has_intersect(lineseg2)<<std::endl;
    return 0;
}
#include "linesegment2d.h"
#include <cmath>
#include <iostream>
#include <limits>

namespace collision
{
linesegment2d::linesegment2d(const point2d& start,const point2d& end)
        :start_(start),end_(end),direction_unit_(0,0){
    double dx=end_.get_x()-start_.get_x();
    double dy=end_.get_y()-start_.get_y();
    length = std::hypot(dy,dx);

    if(length > kMathEpsilon){
        direction_unit_.set_xy(dx/length,dy/length);
    }

    direction_angle=direction_unit_.get_angle();
}

double linesegment2d::distance_to_point(const point2d& given_point) const{
    double dx = given_point.get_x()-start_.get_x();
    double dy = given_point.get_y()-start_.get_y();
    double projection = dx*direction_unit_.get_x()+dy*direction_unit_.get_y();
    if(projection<0){
        return std::hypot(dx,dy);
    }
    if(projection>length){
        return end_.distance_to_point(given_point);
    }
    double distance = std::abs(dx*direction_unit_.get_y()-dy*direction_unit_.get_x());
    return distance;
}

bool linesegment2d::is_point_in(const point2d& given_point) const{
    if(distance_to_point(given_point)<kMathEpsilon){
        return true;
    }
    return false;
}

bool linesegment2d::has_intersect(const linesegment2d& another_line) const{
    point2d point;
    return get_intersect(another_line,&point);
}

bool linesegment2d::get_intersect(const linesegment2d& another_line,point2d* intersect_point) const{
    if(is_point_in(another_line.get_start())){
        *intersect_point = another_line.get_start();
        return true;
    }
    if(is_point_in(another_line.get_end())){
        *intersect_point = another_line.get_end();
        return true;
    }
    if(another_line.is_point_in(start_)){
        *intersect_point=start_;
        return true;
    }
    if(another_line.is_point_in(end_)){
        *intersect_point=end_;
        return true;
    }
    if(length<kMathEpsilon || another_line.length<kMathEpsilon){
        return false;
    }
    const double cc1=(end_-start_).crossproduct(another_line.get_start()-start_);
    const double cc2=(end_-start_).crossproduct(another_line.get_end()-start_);
    if(cc1*cc2>=0){return false;}
    const double cc3=(another_line.get_end()-another_line.get_start()).crossproduct(start_-another_line.get_start());
    const double cc4=(another_line.get_end()-another_line.get_start()).crossproduct(end_-another_line.get_start());
    if(cc3*cc4>=0){return false;}
    double ratio=cc4/(cc4-cc3);

    *intersect_point=point2d(start_.get_x()*ratio+end_.get_x()*(1-ratio),start_.get_y()*ratio+end_.get_y()*(1-ratio));
    return true;

}

point2d linesegment2d::get_intersect(const linesegment2d& another_line) const{
    point2d intersect_point(std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
    if(is_point_in(another_line.get_start())){
        intersect_point.set_xy(another_line.get_start());
        return intersect_point;
    }
    if(is_point_in(another_line.get_end())){
        intersect_point.set_xy(another_line.get_end());
        return intersect_point;
    }
    if(another_line.is_point_in(start_)){
        intersect_point.set_xy(start_);
        return intersect_point;
    }
    if(another_line.is_point_in(end_)){
        intersect_point.set_xy(end_);
        return intersect_point;
    }
    if(length<kMathEpsilon || another_line.length<kMathEpsilon){
        return intersect_point;
    }
    const double cc1=(end_-start_).crossproduct(another_line.get_start()-start_);
    const double cc2=(end_-start_).crossproduct(another_line.get_end()-start_);
    if(cc1*cc2>=0){return intersect_point;}
    const double cc3=(another_line.get_end()-another_line.get_start()).crossproduct(start_-another_line.get_start());
    const double cc4=(another_line.get_end()-another_line.get_start()).crossproduct(end_-another_line.get_start());
    if(cc3*cc4>=0){return intersect_point;}
    double ratio=cc4/(cc4-cc3);

    intersect_point.set_xy(point2d(start_.get_x()*ratio+end_.get_x()*(1-ratio),start_.get_y()*ratio+end_.get_y()*(1-ratio)));
    return intersect_point;
}





}

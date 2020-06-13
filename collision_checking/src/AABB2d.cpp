#include "AABB2d.h"
#include <cmath>
#include <iostream>

namespace collision
{
AABB2d::AABB2d(const point2d& center,const double length,const double width)
        :center_(center){
    length_=length;
    width_=width;
    conner1_.set_xy(center.get_x()-length/2,center.get_y()-width/2);
    conner2_.set_xy(center.get_x()+length/2,center.get_y()-width/2);
    conner3_.set_xy(center.get_x()+length/2,center.get_y()+width/2);
    conner4_.set_xy(center.get_x()-length/2,center.get_y()+width/2);
    if(length<kMathEpsilon || width<kMathEpsilon){
        std::cerr<<"invalid input! the length or the width is zero"<<std::endl;
    }
}

AABB2d::AABB2d(const point2d& conner1,const point2d& conner3)
        :conner1_(conner1),conner3_(conner3){
    if(conner3.get_x()>conner1.get_x()){
        conner4_.set_xy(conner1.get_x(),conner3.get_y());
        conner2_.set_xy(conner3.get_x(),conner1.get_y());
        length_=std::abs(conner3.get_x()-conner1.get_x());
        width_=std::abs(conner3.get_y()-conner1.get_x());
        center_.set_xy((conner1.get_x()+conner3.get_x())/2,(conner1.get_y()+conner3.get_y())/2);
    }
    else{
        conner2_.set_xy(conner1.get_x(),conner3.get_y());
        conner4_.set_xy(conner3.get_x(),conner1.get_y());
        length_=std::abs(conner3.get_x()-conner1.get_x());
        width_=std::abs(conner3.get_y()-conner1.get_x());
        center_.set_xy((conner1.get_x()+conner3.get_x())/2,(conner1.get_y()+conner3.get_y())/2);
    }
    if(length_<kMathEpsilon || width_<kMathEpsilon){
        std::cerr<<"invalid input! the length or the width is zero"<<std::endl;
    }
}

bool AABB2d::get_all_conners(std::vector<point2d> *conners) const{
    std::vector<point2d> temp;
    conners->swap(temp);
    conners->push_back(conner1_);
    conners->push_back(conner2_);
    conners->push_back(conner3_);
    conners->push_back(conner4_);
    return true;
    
}

bool AABB2d::is_point_in(const point2d& given_point) const{
    return std::abs(center_.get_x()-given_point.get_x()) < length_/2 + kMathEpsilon &&
            std::abs(center_.get_y()-given_point.get_y()) < width_/2 + kMathEpsilon;
}

bool AABB2d::is_point_on_boundary(const point2d& given_point) const{
    double dx=std::abs(center_.get_x()-given_point.get_x());
    double dy=std::abs(center_.get_y()-given_point.get_y());
    return (std::abs(dx-length_/2)<kMathEpsilon && dy<width_/2+kMathEpsilon) ||
            (dx < length_/2+kMathEpsilon && std::abs(dy-width_/2)<kMathEpsilon);
}

double AABB2d::distance_to_point(const point2d& given_point) const{
    double dx=std::abs(center_.get_x()-given_point.get_x())-length_/2;
    double dy=std::abs(center_.get_y()-given_point.get_y())-width_/2;
    if(dx<=0){
        return std::max(0.0,dy);
    }
    if(dy<=0){
        return dx;
    }
    return std::hypot(dx,dy);
}

double AABB2d::distance_to_box(const AABB2d& another_box) const{
    double dx=std::abs(center_.get_x()-another_box.get_center().get_x())-length_/2-another_box.get_length()/2;
    double dy=std::abs(center_.get_y()-another_box.get_center().get_y())-width_/2-another_box.get_width()/2;
    if(dx<=0){
        return std::max(0.0,dy);
    }
    if(dy<=0){
        return dx;
    }
    return std::hypot(dx,dy);
}

bool AABB2d::has_overlap(const AABB2d& another_box) const{
    if(distance_to_box(another_box)<kMathEpsilon){
        return true;
    }
    return false;
}

void AABB2d::move(const point2d& vector){
    center_+=vector;
}

void AABB2d::merge_with_point(const point2d& given_point){
    double max_x=std::max(get_max_x(),given_point.get_x());
    double min_x=std::min(get_min_x(),given_point.get_x());
    double max_y=std::max(get_max_y(),given_point.get_y());
    double min_y=std::min(get_min_y(),given_point.get_y());
    center_.set_xy((max_x+min_x)/2,(max_y,min_y)/2);
    length_=max_x-min_x;
    width_=max_y-min_y;
    conner1_.set_xy(min_x,min_y);
    conner2_.set_xy(max_x,min_y);
    conner3_.set_xy(max_x,max_y);
    conner4_.set_xy(min_x,max_y);
}

void AABB2d::merge_with_box(const AABB2d& box){
    double max_x=std::max(get_max_x(),box.get_max_x());
    double min_x=std::min(get_min_x(),box.get_min_x());
    double max_y=std::max(get_max_y(),box.get_max_y());
    double min_y=std::min(get_min_y(),box.get_min_y());
    center_.set_xy((max_x+min_x)/2,(max_y,min_y)/2);
    length_=max_x-min_x;
    width_=max_y-min_y;
    conner1_.set_xy(min_x,min_y);
    conner2_.set_xy(max_x,min_y);
    conner3_.set_xy(max_x,max_y);
    conner4_.set_xy(min_x,max_y);
}



}

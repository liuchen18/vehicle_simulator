#include "OBB2d.h"
#include <cmath>
#include <limits>
#include <iostream>


namespace collision
{
void OBB2d::compute_conners(){
    double dx1=length_*std::cos(angle_)/2;
    double dx2=width_*std::sin(angle_)/2;
    double dy1=length_/2*std::sin(angle_);
    double dy2=width_/2*std::cos(angle_);
    conners_.clear();
    conners_.emplace_back(center_.get_x()+dx1+dx2,center_.get_y()+dy1-dy2);
    conners_.emplace_back(center_.get_x()+dx1-dx2,center_.get_y()+dy1+dy2);
    conners_.emplace_back(center_.get_x()-dx1-dx2,center_.get_y()-dy1+dy2);
    conners_.emplace_back(center_.get_x()-dx1+dx2,center_.get_y()-dy1-dy2);

    for(auto conner:conners_){
        max_x_=std::max(conner.get_x(),max_x_);
        min_x_=std::min(conner.get_x(),min_x_);
        max_y_=std::max(conner.get_y(),max_y_);
        min_y_=std::min(conner.get_y(),min_y_);
    }
}

OBB2d::OBB2d(const point2d& center,const double direction_angle,const double length,const double width)
        :center_(center),angle_(direction_angle),length_(length),width_(width){
    max_x_=std::numeric_limits<double>::lowest(); //negtive min doube
    min_x_=std::numeric_limits<double>::max();
    max_y_=std::numeric_limits<double>::lowest();
    min_y_=std::numeric_limits<double>::max();
    
    compute_conners();
    direction_unit_.set_xy(std::cos(angle_),std::sin(angle_));
    

}

OBB2d::OBB2d(const point2d& center,const point2d& direction_unit,const double length,const double width)
        :center_(center),direction_unit_(direction_unit),length_(length),width_(width){
    angle_=direction_unit_.get_angle();
     
    max_x_=std::numeric_limits<double>::lowest(); //negtive min doube
    min_x_=std::numeric_limits<double>::max();
    max_y_=std::numeric_limits<double>::lowest();
    min_y_=std::numeric_limits<double>::max();

    compute_conners();
}

OBB2d::OBB2d(const linesegment2d& central_lineseg, const double width)
        :center_(central_lineseg.get_center()),direction_unit_(central_lineseg.get_direction_unit()),
        width_(width){
    angle_=direction_unit_.get_angle();
    length_=central_lineseg.get_length();

    max_x_=std::numeric_limits<double>::lowest(); //negtive min doube
    min_x_=std::numeric_limits<double>::max();
    max_y_=std::numeric_limits<double>::lowest();
    min_y_=std::numeric_limits<double>::max();

    compute_conners();    
}

OBB2d::OBB2d(const AABB2d& AABBbox)
        :center_(AABBbox.get_center()),length_(AABBbox.get_length()),width_(AABBbox.get_width()),
        max_x_(AABBbox.get_max_x()),max_y_(AABBbox.get_max_y()),min_x_(AABBbox.get_min_x()),
        min_y_(AABBbox.get_min_y()){
    direction_unit_.set_xy(1.0,0.0);
    angle_=0.0;
    compute_conners();
}

void OBB2d::get_conners(std::vector<point2d> *const connersptr) const{
    *connersptr=conners_;
}

std::vector<point2d> OBB2d::get_conners() const{
    return conners_;
}

bool OBB2d::is_point_in(const point2d& given_point) const{
    double x0=given_point.get_x()-center_.get_x();
    double y0=given_point.get_y()-center_.get_y();
    double dx=x0*direction_unit_.get_x()+y0*direction_unit_.get_y();
    double dy=-x0*direction_unit_.get_y()+y0*direction_unit_.get_x();

    return std::abs(dx)-length_/2<=0+kMathEpsilon && std::abs(dy)-width_/2<=0+kMathEpsilon;
}

bool OBB2d::is_point_on_boundary(const point2d& given_point) const{
    double x0=given_point.get_x()-center_.get_x();
    double y0=given_point.get_y()-center_.get_y();
    double dx=std::abs(x0*direction_unit_.get_x()+y0*direction_unit_.get_y());
    double dy=std::abs(-x0*direction_unit_.get_y()+y0*direction_unit_.get_x());

    return (std::abs(dx-length_/2)<kMathEpsilon && dy<width_/2+kMathEpsilon) ||
            (std::abs(dy-width_/2)<kMathEpsilon && dx<length_/2+kMathEpsilon);
}

double OBB2d::distance_to_point(const point2d& given_point) const{
    double x0=given_point.get_x()-center_.get_x();
    double y0=given_point.get_y()-center_.get_y();
    double dx=std::abs(x0*direction_unit_.get_x()+y0*direction_unit_.get_y())-length_/2;
    double dy=std::abs(-x0*direction_unit_.get_y()+y0*direction_unit_.get_x())-width_/2;

    if(dx <kMathEpsilon){
        return std::max(0.0,dy);
    }
    if(dy<kMathEpsilon){
        return dx;
    }
    return std::hypot(dx,dy);
}

bool OBB2d::has_overlap(const linesegment2d& lineseg)const{

    if(is_point_in(lineseg.get_start()) || is_point_in(lineseg.get_end())){
        return true;
    }
    std::vector<point2d> conners=conners_;
    linesegment2d line1(conners[0],conners[1]),line2(conners[1],conners[2]),line3(conners[2],conners[3]),line4(conners[3],conners[0]);
    if(lineseg.has_intersect(line1) || lineseg.has_intersect(line2) || lineseg.has_intersect(line3) || lineseg.has_intersect(line4)){
        return true;
    }
    return false;
}

bool OBB2d::has_overlap(const OBB2d& another_box) const{
    if(another_box.get_min_x()>get_max_x() || another_box.get_max_x() < get_min_x() || another_box.get_max_y()<get_min_y() ||
        another_box.get_min_y()>get_max_y()){
        return false;
    }
    double d_x_x=another_box.get_center().get_x()-get_center().get_x();
    double d_y_y=another_box.get_center().get_y()-get_center().get_y();

    double dx1=cos_angle_*length_/2;
    double dy1=sin_angle_*length_/2;
    double dx2=-sin_angle_*width_/2;
    double dy2=cos_angle_*width_/2;

    double dx3=another_box.get_cos()*another_box.get_length()/2;
    double dy3=another_box.get_sin()*another_box.get_length()/2;
    double dx4=-another_box.get_sin()*another_box.get_width()/2;
    double dy4=another_box.get_cos()*another_box.get_width()/2;

    if(std::abs(d_x_x*cos_angle_+d_y_y*sin_angle_) > 
        std::abs(dx3*cos_angle_+dy3*sin_angle_)+
        std::abs(dx4*cos_angle_+dy4*sin_angle_)+
        length_/2){
        return false;
    }
    if(std::abs(-d_x_x*sin_angle_+d_y_y*cos_angle_) >
        std::abs(-dx3*sin_angle_+dy3*cos_angle_)+
        std::abs(-dx4*sin_angle_+dy4*cos_angle_)+
        width_/2){
        return false;
    }
    if(std::abs(d_x_x*another_box.get_cos()+d_y_y*another_box.get_sin()) >
        std::abs(dx1*another_box.get_cos()+dy1*another_box.get_sin())+
        std::abs(dx2*another_box.get_cos()+dy2*another_box.get_sin()+
        another_box.get_length()/2)){
        return false;
    }
    if(std::abs(-d_x_x*another_box.get_sin()+d_y_y*another_box.get_cos()) >
        std::abs(-dx1*another_box.get_sin()+dy1*another_box.get_cos())+
        std::abs(-dx2*another_box.get_sin()+dy2*another_box.get_cos()+
        another_box.get_width()/2)){
        return false;
    }
    return true;
}

double OBB2d::distance_to_lineseg(const linesegment2d& lineseg) const{
    std::vector<point2d> conners=conners_;
    if(has_overlap(lineseg)){
        return 0.0;
    }
    double distance=std::numeric_limits<double>::max();
    for(auto conner: conners){
        distance=std::min(distance,lineseg.distance_to_point(conner));
    }
    return distance;
}

double OBB2d::distance_to_box(const OBB2d& another_box) const{
    if(is_point_in(another_box.get_center()) || another_box.is_point_in(center_)){
        return 0.0;
    }
    double distance = std::numeric_limits<double>::max();
    std::vector<point2d> conners=conners_;
    linesegment2d line1(conners[0],conners[1]),line2(conners[1],conners[2]),line3(conners[2],conners[3]),line4(conners[3],conners[0]);
    std::vector<linesegment2d> lines;
    lines.push_back(line1);
    lines.push_back(line2);
    lines.push_back(line3);
    lines.push_back(line4);
    for(auto line:lines){
        distance=std::min(distance,another_box.distance_to_lineseg(line));
    }
    /*
    std::cout<<" "<<another_box.distance_to_lineseg(line1)<<" "<<another_box.distance_to_lineseg(line2)<<" "<<another_box.distance_to_lineseg(line3)<<" "<<another_box.distance_to_lineseg(line4)<<std::endl;
    distance=std::min((((distance,another_box.distance_to_lineseg(line1)),another_box.distance_to_lineseg(line2)),
                    another_box.distance_to_lineseg(line3)),another_box.distance_to_lineseg(line4));
    */
    return distance;

}

void OBB2d::translate(const point2d& vector){
    center_+=vector;
    compute_conners();
}

void OBB2d::rotate(const double angle){
    angle_+=angle;
    angle_=std::fmod(angle_,2*M_PI);
    if(angle_<0.0){
        angle_+=2*M_PI;
    }
    direction_unit_.set_x(std::cos(angle_));
    direction_unit_.set_y(std::sin(angle_));
    cos_angle_=direction_unit_.get_x();
    sin_angle_=direction_unit_.get_y();
    compute_conners();
}



}

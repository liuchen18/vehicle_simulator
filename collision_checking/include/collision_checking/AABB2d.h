#ifndef _AABB2D
#define _AABB2D

#include "point2d.h"
#include <vector>

namespace collision{
class AABB2d{
private:
    point2d center_;
    double length_,width_; //length: x axis;width: y axis
    point2d conner1_,conner2_,conner3_,conner4_;

public:
    /*construction function*/
    AABB2d()=default;
    AABB2d(const point2d& center, const double length, const double width);
    AABB2d(const point2d& conner1,const point2d& conner3);

    point2d get_center() const{return center_;} 
    double get_length() const {return length_;}
    double get_width() const {return width_;}
    double get_area() const {return width_*length_;}
    double get_min_x() const{return center_.get_x()-length_/2;}
    double get_max_x() const{return center_.get_x()+length_/2;}
    double get_min_y() const{return center_.get_y()-width_/2;}
    double get_max_y() const{return center_.get_y()+width_/2;}

    /*return 4 conners of the box
    @param conners is the pointer to the vector which contains the conners*/
    bool get_all_conners(std::vector<point2d> * conners) const;

    /*decide whether the given point in the box*/
    bool is_point_in(const point2d& given_point)const;

    /*decide whether the given point on the boundary of the box*/
    bool is_point_on_boundary(const point2d& given_point) const;

    /*compute distance to given point*/
    double distance_to_point(const point2d& given_point) const;

    /*compute distance to another box */
    double distance_to_box(const AABB2d& another_box) const;

    /*determine whether there is overlap*/
    bool has_overlap(const AABB2d& another_box) const;

    /*move the box
    @param vector is the vector we wish how to move the box*/
    void move(const point2d& vector);

    /*merge the box with another box*/
    void merge_with_box(const AABB2d& box);

    /*merge the box with another point*/
    void merge_with_point(const point2d& given_point);





};
}

#endif
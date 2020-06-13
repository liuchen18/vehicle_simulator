#ifndef _LINESEGMENT2D
#define _LINESEGMENT2D

#include "point2d.h"

namespace collision{
class linesegment2d{
private:
    point2d start_,end_,direction_unit;
    double direction_angle;//angle to the positive x semi axis
    double length;

public:
    linesegment2d(point2d& start,point2d& end);

    const point2d& get_start() const{return start_;}
    const point2d& get_end() const{return end_;}
    const point2d& get_direction_unit() const{return direction_unit;}
    double get_length() const{return length;}
    double get_angle() const{return direction_angle;}
    const point2d get_center() const{return (start_+end_)/2;}

    /*compute distance to a point2d*/
    double distance_to_point (const point2d& given_point) const;

    /*decide whether the given in the line segment*/
    bool is_point_in(const point2d& given_point) const;

    /*check whether the line and another line have intersect*/
    bool has_intersect(const linesegment2d& another_line) const;

    /*
    @brief compute the intersect point if there is any
    @param another_line : ahother line segment
    @param intersect_point: the pointer of the computed intersect point
    @return bool, whether there is the intersect point 
    */
    bool get_intersect(const linesegment2d& another_line, point2d* intersect_point) const;

};


}



#endif
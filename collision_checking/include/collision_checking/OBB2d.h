#ifndef _OBB2D
#define _OBB2D

#include "point2d.h"
#include <vector>
#include "linesegment2d.h"
#include "AABB2d.h"

namespace collision
{
class OBB2d
{
public:
    /*construction function*/
    OBB2d()=default;
    virtual ~OBB2d(){};

    /*
    @brief construction function
    @param center center point of the OBB
    @param direction_unit the vector of the direction
    @param length length along the OBB axis
    @param width length perpendicular to the OBB axis
    */
    OBB2d(const point2d& center,const point2d& direstion_unit,const double length,const double width);

    /*
    @brief construction function
    @param center center point of the OBB
    @param direction_angle the angle to the positive x semi axis
    @param length length along the OBB axis
    @param width length perpendicular to the OBB axis
    */
    OBB2d(const point2d& center,const double direction_angle,const double length,const double width);

    /*
    @brief construction function
    @param central_lineseg central line segment of the OBB, contains direction and length and center
    @param width length perpendicular to the central line segment
    */
    OBB2d(const linesegment2d& central_lineseg, const double width);

    /*
    @brief construction function
    @param AABBbox box with the type of AABB2d
    */
    OBB2d(const AABB2d& AABBbox);

    /*
    @brief conpute all conners of the OBB
    */
    void compute_conners();

    /*get center point*/
    point2d get_center()const{return center_;}

    /*get angle*/
    double get_angle() const {return angle_;}

    /*get cos angle*/
    double get_cos() const{return cos_angle_;}

    /*get sin angle*/
    double get_sin() const{return sin_angle_;}

    /*get length*/
    double get_length() const{return length_;}

    /*get width*/
    double get_width() const {return width_;}

    /*get direction unit*/
    point2d get_direction_unit() const{return direction_unit_;}

    /*get min x*/
    double get_min_x() const{return min_x_;}

    /*get max x*/
    double get_max_x() const{return max_x_;}

    /*get min y*/
    double get_min_y() const{return min_y_;}

    /*get min x*/
    double get_max_y() const{return max_y_;}

    /*@brief get the 4 conner
    @param connersptr the pointer to save the addrass of the vector of conner
    */
    void get_conners(std::vector<point2d> *const connersptr) const;

    /*brief get the 4 conners
    @return conners in vector
    */
    std::vector<point2d> get_conners() const;

    /*@brief determine whether the given point in the obb box
    @param given_point
    @return bool
    */
    bool is_point_in(const point2d& given_point) const;

    /*@brief determine whether the given point on the boundary of the box
    #param given_point
    return bool
    */
    bool is_point_on_boundary(const point2d& given_point) const;

    /*@brief compute the distance to given point from the box
    @param given_point
    @return double distance
    */
    double distance_to_point(const point2d& given_point) const;

    /*@brief determine whether the given line and the box bas overlap
    @param lineseg
    @return bool
    */
    bool has_overlap(const linesegment2d& lineseg) const;

    /*@brief determine whether the given box and the box has overlap
    @param another_box
    @return bool
    */
    bool has_overlap(const OBB2d& another_box) const;

    /*@brief compute distance to line segment
    @param lineseg
    return double distance*/
    double distance_to_lineseg(const linesegment2d& lineseg) const;

    /*@brief compute distance to another box
    @param another_box
    @return double distance
    */
    double distance_to_box(const OBB2d& another_box) const;

    /*@brief move the box along a vector
    @param vector vector to move along
    */
    void translate(const point2d& vector);

    /*
    @brief rotate the box from center
    @param angle the angle to rotate
    */
    void rotate(const double angle);


private:
    point2d center_;
    point2d direction_unit_;
    double cos_angle_,sin_angle_;
    double angle_;
    double length_,width_;
    double max_x_,max_y_,min_x_,min_y_;
    std::vector<point2d> conners_;
};

}





#endif
#ifndef _VEHICLE_STATE
#define _VEHICLE_STATE

#include "geometry_msgs/Twist.h"
#include "OBB2d.h"
#include "geometry_msgs/TransformStamped.h"

namespace vehicle
{

#define Car_STEERING_RATIO      17.3  // Ratio between steering wheel angle and tire angle
#define Car_LOCK_TO_LOCK_REVS   3   // Number of steering wheel turns to go from lock to lock
#define Car_MAX_STEER_ANGLE     (M_PI * Car_LOCK_TO_LOCK_REVS / Car_STEERING_RATIO)
#define Car_WHEELLENGTH           2.65  // Distance between front and rear axles
#define Car_WIDTH              1.638 // Distance between front wheels

class vehicle_state{
    private:
    geometry_msgs::Twist current_velocity_;
    double yaw_;
    double x_,y_;
    double vel_;
    int gear_state_; //0 : forward;1 backward
    double steer_angle_;
    collision::OBB2d current_box_;

    public:
    /*constructor*/
    vehicle_state(const double car_length,const double car_width);

    /*@brief subscribe vehicle velocity to update state
    @param msg the topic message we subscribe
    */
    void velocity_callback(const geometry_msgs::Twist& msg);

    /*@brief compute vehicle velocity state according to the current_velocity_
    @kinematics model
    */
    void compute_velocity_state();

    /*@brief subscribe the tf to get global position and orientation
    @ param 
    */
    void position_callback(const geometry_msgs::TransformStamped& msg);

    double get_x() const {return x_;}

    double get_y() const {return y_;}

    double get_yaw() const {return yaw_;}

    int get_gear_state() const {return gear;}

    double get_velocity() const {return vel_;}

    double get_steer_angle() const {return steer_angle_;}

    /*@brief check collision with linesegment
    @param lineseg linesegment2d 
    @return bool true means collision
    */
    bool check_collision(const collision::linesegment2d& lineseg) const;

    /*@brief check collision with point
    @param point point2d 
    @return bool true means collision
    */
    bool check_collision(const collision::point2d& point) const;

    /*@brief check collision with box
    @param box OBB2d 
    @return bool true means collision
    */
    bool check_collision(const collision::OBB2d& box) const;


};
}




#endif
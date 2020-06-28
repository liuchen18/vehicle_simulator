#include "vehicle_state.h"
#include <cmath>

namespace vehicle
{
vehicle_state::vehicle_state(const double car_length, const double car_width){
    current_box_.set_length(car_length);
    current_box_.set_width(car_width);
}

void vehicle_state::velocity_callback(const geometry_msgs::Twist& msg){
    current_velocity_=msg.twist;
    compute_velocity_state();
}

void vehicle_state::compute_velocity_state(){
    double vt=std::hypot(current_velocity_.linear.x,current_velocity_.linear.y);
    double vt_angle=std::atan2(current_velocity_.linear.y,current_velocity_.linear.x);
    vel_=vt;
    if(wheel_vel_ > 0){
        gear_state_=0;
    }
    else{
        gear_state_=1;
    }
    double beita=std::arcsin(current_velocity_.angular.z*CAR_WHEELLENGTH/2/vt);
    steer_angle_=std::atan(2*std::tan(beita));

}

void vehicle_state::positon_callback(const geometry_msgs::TransformStamped& msg){
    if(msg.transforms[0].header.frame_id="world" && msg.transforms[0].child_frame_id="base_footprint"){
        x_=msg.transforms[0].transform.translation.x;
        y_=msg.transforms[0].transform.translation.x;
        double x=msg.transforms[0].transform.rotation.x;
        double y=msg.transforms[0].transform.rotation.y;
        double z=msg.transforms[0].transform.rotation.z;
        double w=msg.transforms[0].transform.rotation.w;
        yaw_=std::atan2(2*(w*z+x*y),1-2*(std::pow(y,2)+std::pow(z,2)));
        current_box_.translate_to(x_+CAR_WHEELLENGTH/2*std::cos(yaw),y_+CAR_WHEELLENGTH/2*std::sin(yaw));
        current_box_.rotate_to(yaw);
    }
}

bool vehicle_state::check_collision(const collision::linesegment2d& lineseg) const{
    return current_box_.has_overlap(lineseg);
}

bool vehicle_state::check_collision(const collision::point2d& point) const {
    return current_box_.is_point_in(point);
}

bool vehicle_state::check_collision(const collision::OBB2d& box) const {
    return current_box_.has_overlap(box);
}

}

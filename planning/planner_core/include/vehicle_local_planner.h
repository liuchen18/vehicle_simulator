#ifndef _VEHICLE_LOCAL_PLANNER
#define _VEHICLE_LOCAL_PLANNER

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

namespace planner
{
class vehicle_local_planner{
public:
    /*destructor*/
    virtual ~vehicle_local_planner(){}

    /*
    @brief compute velocity command
    @param vel a pointer which will be filled by the function, contains the computed velocity command
    @return bool if a valid velocity was found
    */
    virtual bool compute_velocity_command(geometry_msgs::Twist& vel)=0;

    /*
    @brief check is the goal reached
    return bool
    */
    virtual bool is_goal_reached()=0;

    /*
    @brief initialize
    @param name the name of the planner
    @param map a pointer to the map
    @return*/
    virtual void initialize(std::string name, nav_msgs::OccupancyGrid* map) = 0;

    /**
    @brief  Set the plan that the local planner is following
    @param plan The plan to pass to the local planner
    @return True if the plan was updated successfully, false otherwise
    */
    virtual bool set_plan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;


protected:
    /*construction function*/
    vehicle_local_planner(){}

};
}


#endif
#ifndef _VEHICLE_PNC
#define _VEHICLE_PNC

#include "geometry_msgs/PoseStamped.h"
#include <vector>

namespace velicle_pnc
{
enum vehicle_pnc_state
{
    PLANNING,
    RUNNING,
    CLEARING
};

class vehicle_pnc
{
public:
    vehicle_pnc();
    virtual ~vehicle_pnc();

private:

    /**
    @brief control cycle for the vehicle
    @param goal the goal for the vehicle to achieve
    @param global_plan the planned path for the vehicle
    @return bool true if goal achieved
    */
    bool control_cycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    /**
    @brief  Make a new global plan
    @param  goal The goal to plan to
    @param  plan Will be filled in with the plan made by the planner
    @return  True if planning succeeds, false otherwise
    */
    bool make_global_plan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    /**
    @brief subscribe the goal
    @param goal the planning goal
    return void
    */
    void goal_callback(const geometry_msgs::PoseStamped& goal);

    /**
    @brief the thread for the global planning algorithm
    */
    void plan_thread();
    
};


}



#endif
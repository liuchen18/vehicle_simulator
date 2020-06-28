#ifndef _VEHICLE_GLOBAL_PLANNER
#define _VEHICLE_GLOBAL_PLANNER

#include <nav_msgs/OccupancyGrid.h>

namespace planner
{
class vehicle_global_planner{
protected:
vehicle_global_planner(){}

public:
    /**
    @brief  Virtual destructor for the interface
    */
    virtual ~vehicle_global_planner(){}

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose 
     * @param goal The goal pose 
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    virtual bool make_plan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) = 0;
    
    /*
    @brief the initialize function for the planner
    @param name the name of the planner
    @param map a pointer to the map
    */
    virtual void initialize(std::string name, nav_msgs::OccupancyGrid* map) = 0;
};

}



#endif
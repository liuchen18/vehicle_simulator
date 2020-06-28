#ifndef _GLOBAL_PLANNER
#define _GLOBAL_PLANNER

#include "nav_msgs/OccupancyGrid.h"

namespace planner
{
class global_planner:public vehicle_global_planner{
    private:

    nav_msgs::OccupancyGridPtr map;


    public:
    
    /**
    @brief rewrite the makeplan function
    @param start The start pose 
    @param goal The goal pose 
    @param plan The plan... filled by the planner
    @return True if a valid plan was found, false otherwise
    */
    virtual make_plan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);


    /**
    @brief the initialize function for the planner
    @param map a pointer to the map
    */
    virtual void initialize(nav_msgs::OccupancyGridPtr map);

};
}



#endif
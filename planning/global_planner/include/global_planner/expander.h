#ifndef _EXPANDER
#define _EXPANDER

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

namespace planner
{
class expander{
    protected:
    /**
    @brief constructor
    */
    expander()

    public:
    


    /**
    @brief rewrite the makeplan function
    @param start The start pose 
    @param goal The goal pose 
    @param plan The plan... filled by the planner
    @return True if a valid plan was found, false otherwise
    */
    virtual void plan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)=0;


    /**
    @brief the initialize function for the planner
    @param map a pointer to the map
    */
    virtual void update_map(nav_msgs::OccupancyGridPtr map)=0;

};
}





#endif
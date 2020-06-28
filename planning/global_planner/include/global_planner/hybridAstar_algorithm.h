#ifndef _HYBRIDASTAR_ALGORITHM
#define _HYBRIDASTAR_ALGORITHM

#include "node.h"
#include <unordered_map>
#include "node2d.h"
#include "dubins.h"

namespace planner
{
class hybrid_A_star{
    private:
    nav_msgs::OccupancyGridPtr map;
    double map_width,map_height;
    std::unordered_set<node2d> astar_close_set;
    std::unordered_set<int> astar_close_index_set;



    public:
    /** constructor*/
    hybrid_A_star();

    /**
    @brief rewrite the makeplan function
    @param start The start pose 
    @param goal The goal pose 
    @param plan The plan... filled by the planner
    @return True if a valid plan was found, false otherwise
    */
    virtual node* plan(const node& start, const node& goal, node* nodes3d, node2d* nodes2d);


    /**
    @brief the initialize function for the planner
    @param map a pointer to the map
    */
    virtual void update_map(nav_msgs::OccupancyGridPtr mapptr);

    /**
    @brief origin a star algorithm to compute the h function
    @param start the start node2d 
    @param goal the goal node2d
    @param nodes2d a list of all node2d. after origin a star , it will be the closed list with value functions
    */
    float origin_A_star(const node2d& start, const node2d& goal,node2d* nodes2d);

    /**
    @brief update node using hybrid a star method
    @param start the node needed to be updated
    @param goal the goal node
    @param nodes2d a list of all node2d
    */
    void update_h(node& start,const node& goal,node2d* nodes2d)


    /**
    @brief use dubins to get a posible collision free path
    @param start current point
    @oaram goal goal point
    @return 
    */
    node* dubinsShot(node& start, const node& goal);

    /**
    @brief check is collision free
    @param mapptr the point of the map
    @param cur_node a pointer of the node to be checked
    @return true if collision free
    */
    bool is_collision_free(nav_msgs::OccupancyGridPtr mapptr, node* cur_node);

    /**
    @brief check is collision free
    @param mapptr the point of the map
    @param cur_node a pointer of the node to be checked
    @return true if collision free
    */
    bool is_collision_free(nav_msgs::OccupancyGridPtr mapptr, node2d* cur_node);

    /**
    @brief sample vehicle steer angle and get vehicle motion
    @param cur_node current position of the vehicle
    @return a vector of possible motion*/
    std::vector<std::vector<double>> hybrid_A_star::get_vehicle_motion(node cur_node);


};
}





#endif
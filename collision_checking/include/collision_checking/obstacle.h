#ifndef _OBSTACLE
#define _OBSTACLE

#include <vector>
#include "OBB2d.h"
#include "point2d.h"

namespace collision
{
class obstacles{
    private:
    int obstacle_num_;
    std::vector<OBB2d> *obstacles_ptr_;

    public:
    /*constructor and destructor*/
    obstacles()=default;
    obstacles(const int num,std::vector<OBB2d> *obstacles_ptr);
    ~obstacles();

    /*@brief get current obstacle vector
    @return std::vector<OBB2d>* a pointer to the obstacle vector
    */
    std::vector<OBB2d>* get_obstacles() const{return obstacles_ptr_;}

    /*@brief get current obstacle num
    @return int obstacle number
    */
    int get_obstacle_num() const{return obstacle_num_;}
    
};
}


#endif
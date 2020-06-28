//
// Created by chen on 6/23/20.
//
#include "hybridAstar_algorithm.h"
#include <boost/heap/binomial_heap.h>
#include "constance.h"
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

#include "OBB2d.h"

typedef ompl::base::SE2StateSpace::StateType State;

namespace planner
{
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const node* lhs, const node* rhs) const {
        return lhs->get_f() > rhs->get_f();
    }
    /// Sorting 2D nodes by increasing C value - the total estimated cost
    bool operator()(const node2d* lhs, const node2d* rhs) const {
        return lhs->get_f() > rhs->get_f();
    }
};

virtual void hybridAstar_algorithm::update_map(nav_msgs::OccupancyGridPtr map){
    map=mapptr;
    map_width=map->info.width;
    map_height=map->info.height;

}

double hybrid_A_star::origin_A_star(const node2d& start, const node2d& goal,node2d* nodes2d){

    for (int i = 0; i < map_width * map_height; ++i) {
        nodes2d[i]->reset();
    }

    boost::heap::binomial_heap<Node2D*,boost::heap::compare<CompareNodes>> open_list;

    //put the syart into open list
    start.update_h(goal);
    start.in_open();
    open_list.push(&start);
    int node_index=start.update_index(map_width);
    nodes2d[node_index].in_open();
    nodes2d[node_index]=start;
    
    while(!open_list.empty()){
        node2d* cur_node=open_list.top();
        node_index=cur_node->update_index(map_width);

        //if the node is already in closed list, continue
        if(nodes2d[node_index].is_in_closed()){
            open_list.pop();
            continue;
        }
        //if the new node is in open list(the node is taken from open list)
        else if(nodes2d[node_index].is_in_open()){
            nodes2d[node_index].in_closed();
            nodes2d[node_index].visit();

            open_list.pop();

            //if the current node is the goal. the operator == has been rewritten in node2d.h
            if(*cur_node == goal){
                return cur_node->get_g();
            }
            else{
                //if the node is not goal, then expend its son node
                for(int i=0;i<node2d::dir_num;i++){
                    node2d* new_node=cur_node->create_son_node(i);
                    int new_index=new_node->update_index(map_width);

                    //the new node is in the grid map and is collision free and not in closed list
                    if(new_node->is_on_grid() && is_collision_free(map,new_node) && !nodes2d[new_index].is_in_closed()){ //check map collision!!!
                        new_node.update_g();
                        double new_g=new_node->get_g();

                        //if the node is not in open list of the new g is smaller than the older g,
                        if(!nodes2d[new_index].is_in_open() || new_g<nodes2d[new_index].get_g()){
                            new_node->update_h(goal);
                            new_node->in_open();
                            nodes2d[new_index]=*new_node;

                            open_list.push(&nodes2d[new_index]);
                            delete new_node;
                        }
                        else{
                            delete new_node;
                        }
                    }
                    else{
                        delete new_node;
                    }
                }
            }
        }
    }
    return 1000;
}

virtual node* hybrid_A_star::plan(const node& start, const node& goal, node* nodes3d, node2d* nodes2d){
    int cur_index,son_index;
    double new_g;

    //if the vehicle can move reverse, the number of direction is constance::sampling_num
    int dir_num=constance::reverse? constance::sampling_num: 2*constance::sampling_num;
    int iterations=0;

    typedef boost::heap::binomial_heap<Node3D*,boost::heap::compare<CompareNodes>> priorityQueue;
    priorityQueue open_list;
    //compute the h value of the start node
    update_h(start, goal, nodes2d, dubinsLookup);

    start.in_open();
    open_list.push(&start);
    cur_index=start.update_index(map_width,map_height);
    nodes3d[cur_index]=start;

    node* cur_node;
    while(!open_list.empty()){
        cur_node=open_list.top();
        iterations+=1;
        cur_index=cur_node.update_index(map_width,map_height);

        //the node is already in closed list
        if(nodes3d[cur_index].is_in_closed()){
            open_list.pop();
            continue;
        }
        //if the node is in open list,exapnd it
        else{
            nodes3d[cur_index].in_closed();
            open_list.pop();
            if(*cur_node==goal){
                return cur_node;
            }
            if(iterations > constance::max_iterations){
                return nullptr;
            }
            //if the vehicle move forward, use dubins first
            if(constance::DubinsShot && cur_node->has_analycal_solution(goal) && cur_node->is_forward()){
                node* son_node=dubinsShot(*cur_node,goal);
                //if the dubins method can make it, just return
                if(son_node != nullptr && *son_node==goal){
                    return son_node;
                }
            }

            //expand the node
            std::vector<std::vector<double>> vehicle_motion=get_vehicle_motion(cur_node);
            for(int i=0;i<vehicle_motion.size();i++){
                double dx=vehicle_motion[i][0];
                double dy=vehicle_motion[i][1];
                double dt=vehicle_motion[i][2];
                node* son_node=cur_node->create_son_node(dx,dy,dt);
                son_index=son_node->update_index(map_width,map_height);
                
                //check the new node: on the grid map and collision free
                if(son_node->is_on_grid() && is_collision_free(map,son_node)){
                    //if the node is not in the closed list or the son node and current node in the same grid
                    if(!nodes3d[son_index].is_in_closed() || cur_index==son_index){
                        son_node->update_g();
                        son_g=son_node->get_g();

                        //if the son node is not in the open list or the g is smaller or the son node and current node in the same grid
                        if(!nodes3d[son_index].is_in_open || son_g < nodes3d[son_index].get_g() || cur_index==son_index){
                            update_(son_node, goal, nodes2d, dubinsLookup);

                            //if the son node and the current node are in the same grid but the f of son node is larger, continue
                            if(cur_index==son_index && son_node->get_f()>cur_node->get_f()+constance::tieBreaker){
                                delete son_node;
                                continue;
                            }
                            //if in the same grid but the f of son node is smaller,set the father of the current node as the father of the son node
                            else if(cur_index==son_index && son_node->get_f()<=cur_node->get_f()+constance::tieBreaker){
                                son_node->set_father_node(cur_node->get_father_node());
                            }

                            if(son_node->get_father_node() == son_node){
                                std::cout<<"the node is not moving!"<<std::endl;
                            }
                            son_node->in_open();
                            nodes3d[son_index]=*son_node;
                            open_list.push(&nodes3d[son_index]);
                            delete son_node;
                        }
                        else{
                            delete son_node;
                        }

                    }
                    else{
                        delete son_node;
                    }

                }
                else{
                    delete son_node;
                }
            }

        }

    }
    return nullptr;

}

// 计算到目标的启发值(cost)
// 这里的cost由三项组成：《Practical Search Techniques in Path Planning for Autonomous Driving》
// 1) "non-holonomic-without-obstacles" heuristic:（用于指导搜索向目标方向前进）
//    受运动学约束的无障碍启发式值。论文的计算建议为： max(Reed-Shepp距离/Dubins距离, 欧氏距离) 表示
//    至于用Reed-Shepp距离还是Dubins距离取决于车辆是否可倒退
// 2) "holonomic-with-obstacles" heuristic：（用于发现U形转弯(U-shaped obstacles)/死路(dead-ends)）
//    （不受运动学约束的）有障约束启发式值(即：A*)
// 注1： 实际计算时，优先考虑运动学启发式值，A*作为可选项。至于是否启用欧氏距离和A*的启发式值，取决于计算
//      的精度和CPU性能（可作为调优手段之一）
// 注2： 实际计算与论文中的描述存在差异：
//      （1）实际计算的第一步用的启发式值为“Reed-Shepp距离/Dubins距离”，而论文为“max(Reed-Shepp距离/Dubins距离, 欧氏距离)”
//      （2）实际计算的第二步用的启发式值为A*的启发式值 减去 “start与goal各自相对自身所在2D网格的偏移量(二维向量)的欧氏距离”
//          该步计算的意义还需仔细分析，目前我还没想明白代码这样设计的理由。

void hybrid_A_star::update_h(node& start,const node& goal,node2d* nodes2d,float* dubinsLookup){
    double dubinsCost = 0;
    double reedsSheppCost = 0;
    double Astar_cost = 0;
    double euclidean_distance = 0;
    //dubins
    if(constance::dubins){
        ompl::base::DubinsStateSpace dubinsPath(constance::radius);
        State* dbStart = (State*)dubinsPath.allocState();
        State* dbEnd = (State*)dubinsPath.allocState();
        dbStart->setXY(start.get_x(), start.get_y());
        dbStart->setYaw(start.get_heading());
        dbEnd->setXY(goal.get_x(), goal.get_y());
        dbEnd->setYaw(goal.get_heading());
        dubinsCost = dubinsPath.distance(dbStart, dbEnd);
    }
    //reeds shepp
    if(constance::reverse && !constance::dubins){
        ompl::base::ReedsSheppStateSpace reedsSheppPath(constance::radius);
        State* rsStart = (State*)reedsSheppPath.allocState();
        State* rsEnd = (State*)reedsSheppPath.allocState();
        rsStart->setXY(start.get_x(), start.get_y());
        rsStart->setYaw(start.get_heading());
        rsEnd->setXY(goal.get_x(), goal.get_y());
        rsEnd->setYaw(goal.get_heading());
        reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    }
    //if the node has not been visited, using origin a star to search its g
    if(constance::twoD && !nodes2d[(int)start.get_y()*map_width+(int)start.get_x()].is_visited()){
        node2d start_node2d(start.get_x(),start.get_y(),0,0,nullptr);
        node2d goal_node2d(goal.get_x(),goal.get_y(),0,0,nullptr);

        double cur_g=origin_A_star(goal_node2d,start_node2d,nodes2d)
        int cur_index=(int)start_node2d.get_y()*map_width+(int)start_node2d.get_x();
        nodes2d[cur_index].set_g(cur_g);

    }
    //get twoDCost
    if(constance::twoD){
        int cur_index=(int)start_node2d.get_y()*map_width+(int)start_node2d.get_x();
        Astar_cost=nodes2d[cur_index].get_f();
    }
    euclidean_distance=std::hypot(start.get_x()-goal.get_x(),goal.get_y()-start.get_y());
    double h=std::max(dubinsCost,reedsSheppCost);
    h=std::max(h,Astar_cost);
    start.set_h(h);
}


node* hybrid_A_star::dubinsShot(node& start, const node& goal) {
  // start
  double q0[] = { start.get_x(), start.get_y(), start.get_heading() };
  // goal
  double q1[] = { goal.get_x(), goal.get_y(), goal.get_heading() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, Constants::radius, &path);

  int i = 0;
  float x = 0.0;
  float length = dubins_path_length(&path);

  node* dubinsNodes = new node [(int)(length / Constants::dubinsStepSize) + 1];

  while (x <  length) {//这是跳出循环的条件之一：生成的路径没有达到所需要的长度
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].set_x(q[0]);
    dubinsNodes[i].set_x(q[1]);
    dubinsNodes[i].set_heading(common::normalizeHeadingRad(q[2]));

    // collision check
    //跳出循环的条件之二：生成的路径存在碰撞节点
    if (is_collision_free(map,&dubinsNodes[i])) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].set_father_node(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].set_father_node(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].get_father_node()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    } 
  }

  std::cout << "Dubins shot connected, returning the path" << "\n";
  //返回末节点，通过getPred()可以找到前一节点。
  return &dubinsNodes[i - 1];
}

bool hybrid_A_star::is_collision_free(nav_msgs::OccupancyGridPtr mapptr, node* cur_node){
    double x=cur_node->get_x();
    double y=cur_node->get_y();
    double heading=cur_node->get_heading();

    point2d center(x,y);

    OBB2d cur_box(center,heading,constance::length,constance::width);
    int length_num=(int)(constance::length/constance::cellSize);
    int width_num=(int)(constance::width/constance::cellSize);
    for(int i=-length_num;i<length_num+1;i++){
        for(int j=-width_num;j<width_num+1,j++){

            node temp_node(x+i*constance::cellSize,y+j*constance::cellSize,0,0,0,nullptr);
            point2d temp_point(x+i*constance::cellSize,y+j*constance::cellSize);
            int index=temp_node.update_index(map_width,map_height);
            if(mapptr->data[index]!=0 && cur_box.is_point_in(temp_point)){
                return false;
            }
        }
    }
    return true;

}

bool hybrid_A_star::is_collision_free(nav_msgs::OccupancyGridPtr mapptr, node2d* cur_node){
    double x=cur_node->get_x();
    double y=cur_node->get_y();

    point2d center(x,y);

    OBB2d cur_box(center,0.0,constance::length,constance::width);
    int length_num=(int)(constance::length/constance::cellSize);
    int width_num=(int)(constance::width/constance::cellSize);
    for(int i=-length_num;i<length_num+1;i++){
        for(int j=-width_num;j<width_num+1,j++){

            node temp_node(x+i*constance::cellSize,y+j*constance::cellSize,0,0,0,nullptr);
            point2d temp_point(x+i*constance::cellSize,y+j*constance::cellSize);
            int index=temp_node.update_index(map_width,map_height);
            if(mapptr->data[index]!=0 && cur_box.is_point_in(temp_point)){
                return false;
            }
        }
    }
    return true;
}

std::vector<std::vector<double>> hybrid_A_star::get_vehicle_motion(node cur_node){
    std::vector<std::vector<double>> res;
    std::vector<double> temp_motion;
    for(int i=0;i<constance::sampling_num;i++){
        double cur_steer_angle=common::toRad(constance::max_steer_angle)*2/constance::sampling_num*(i+1);
        double beita=std::atan(0.5*std::tan(cur_steer_angle));
        temp_motion.emplace_back(constance::velocity*std::cos(beita)*constance::sample_time);
        temp_motion.emplace_back(constance::velocity*std::sin(beita)*constance::sample_time);
        temp_motion.emplace_back(constance::velocity/constance::length*2*std::sin(beita)*constance::sample_time);
        res.push_back(temp_motion);
        temp_motion.clear();
    }
    return res;

}

}





/*TODO
vehicle_motion
*/
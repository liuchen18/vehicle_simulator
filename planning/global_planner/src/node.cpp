#include "node.h"
#include "constance.h"

namespace planner
{
bool node::is_on_grid(const int width, const int height) const{
    return x >= 0 && x < width && y >= 0 && y < height && 
            (int)(t / Constants::deltaHeadingRad) >= 0 && 
            (int)(t / Constants::deltaHeadingRad) < Constants::headings;
}

bool node::has_analytical_solution(){
    int random = rand() % 10 + 1;//产生位于[1, 10]的随机数
    float dx = std::abs(x - goal.x) / random;
    float dy = std::abs(y - goal.y) / random;
    return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;//距离的平方和在100以内则认为可达
}

node* node::create_son_node(const double& dx,const double& dy,const double& dt) const{
    double x=_x+dx*std::cos(_theta)-dy*std::sin(_theta);
    double y=_y+dx*std::sin(_theta)+dy*std::cos(_theta);
    bool is_forward=dx>0? true:false;
    if(is_forward){
        double theta=common::normalizeHeadingRad(_theta+dt);
    }
    else{
        double theta=common::normalizeHeadingRad(_theta-dt);
    }
    return new node(x,y,theta,g,0,this,is_forward);
}

void node::update_g(){
    double penalty=(std::abs(dx)+std::abs(dy)+std::abs(dt))
    if(std::abs(dt)<0.00001){
        if(_is_forward!= _father_node->_is_forward){
            if(_is_forward){
                penalty*=constance::penaltyCOD;
            }
            else{
                penalty*=constance::penaltyCOD*constance::penaltyReversing;
            }
        }
        else{
            if(_is_forward){
                penalty*=1.0;
            }
            else{
                penalty*=constance::penaltyReversing;
            }
        }
    }
    else{
        if(_is_forward!= _father_node->_is_forward){
            if(_is_forward){
                penalty*=constance::penaltyCOD*constance::penaltyTurning;
            }
            else{
                penalty*=constance::penaltyCOD*constance::penaltyTurning*constance::penaltyReversing;
            }
        }
        else{
            if(_is_forward){
                penalty*=constance::penaltyTurning;
            }
            else{
                penalty*=constance::penaltyReversing*constance::penaltyTurning;
            }
        }
    }
    _g+=penalty;
}

bool node::operator == (const node& another) const {
  return (int)_x == (int)another.get_x() &&
         (int)_y == (int)another.get_y() &&
         (std::abs(_theta - another.get_heading()) <= Constants::deltaHeadingRad ||
          std::abs(_theta - another.get_heading()) >= Constants::deltaHeadingNegRad);
}


}

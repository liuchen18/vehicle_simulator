#ifndef _PATH_NODE3D
#define _PATH_NODE3D

namespace planner
{
class node{
private:
    double _x; //x_position
    double _y; //y_position
    double _theta; //heading rad
    double _cost_so_far; //real cost(g)
    double _cost_to_go; //heuristic value
    double _is_in_open; //is in the open set
    double _is_in_closed;//is in the closed set
    //int _index; //index of the node
    node *_father_node;//the predecessor pointer
    double dx,dy,dt;//
    bool _is_forward;

public:
    /*
    @brief the default constructor
    */
    node():node(0,0,0,0,0,NULL){}

    /*
    @brief constructor with arguments
    @param x x position of the node
    @param y y position of the node
    @param theta heading of the node
    @parma g real cost of the node
    @param h heuristic value of the node
    @param pre_node a pointer to the father node
    */
    node(double x,double y,double theta,double g,double h,node* pre_node,bool is_forward=true)
        :_x(x),_y(y),_theta(theta),_cost_so_far(g),_cost_to_go(h),_father_node(pre_node),_is_forward(is_forward){
        index=-1;
        _is_in_open=false;
        _is_in_closed=false;
        dx=_x-_father_node->get_x();
        dy=_y-_father_node->get_y();
        dt=_theta-_father_node->get_heading();
    }
    /*
    @brief constructor with arguments
    @param x x position of the node
    @param y y position of the node
    @param theta heading of the node
    @parma g real cost of the node
    @param h heuristic value of the node
    @param pre_node a pointer to the father node
    @param dx the distance from the current node to the fathern node in x axis
    @param dy the distance from the current node to the fathern node in y axis
    @param dt the distance from the current node to the fathern node in theta axis
    */
    node(double x,double y,double theta,double g,double h,node* pre_node,double dx,double dy,double dt,bool is_forward=true)
        :_x(x),_y(y),_theta(theta),_cost_so_far(g),_cost_to_go(h),_father_node(pre_node),_is_forward(is_forward)
        {
        index=-1;
        _is_in_open=false;
        _is_in_closed=false;
        this->dx=dx;
        this->dy=dy;
        this->dt=dt;
    }

    /*get x position*/
    double get_x() const{return _x;}

    /*get y positon*/
    double get_y() const {return _y;}

    /*get heading*/
    double get_heading() const {return _theta;}

    /*get cost so far*/
    double get_g() const {return _cost_so_far;}

    /*get cost to go*/
    double get_h() const {return _cost_to_go;}

    /*get cost total*/
    double get_f() const {return _cost_so_far+_cost_to_go;}

    /* get whether in open*/
    bool is_in_open() const {return _is_in_open;}

    /*get whether in closed*/
    bool is_in_closed() const {return _is_in_closed;}

    /*get father node*/
    node* get_father_node() const {return _father_node;}

    /*get index*/
    int get_index() const {return _index;}

    /*get delta x*/
    int get_dx() const {return dx;}

    /*get delta y*/
    int get_dy() const {return dy;}

    /*get delta t*/
    int get_dt() const {return dt;}

    /*get running direction*/
    bool is_forward() const {return _is_forward;}

    /*set x position*/
    void set_x(const double& x){_x=x;}

    /*set y position*/
    void set_y(const double& y){_y=y;}

    /*set heading*/
    void set_heading(const double& heading){_theta=heading;}

    /*set _cost_so_far*/
    void set_g(const double& g){_cost_so_far=g;}

    /*set cost_to_go*/
    void set_h(const double& h){_cost_to_go=h;}

    /*set index*/
    int update_index(int width, int height) { _index = (int)(_theta / Constants::deltaHeadingRad) * width * height + (int)(_y) * width + (int)(_x); return _index;}

    /*put the node into open set*/
    void in_open(){_is_in_closed=false;_is_in_open=true;}

    /*put the node into closed set*/
    void in_closed(){_is_in_closed=true;_is_in_open=false;}

    /*set the father node of the current node*/
    void set_father_node(const node* father_node){_father_node=father_node;}

    /*update the cost so far fot the node from its father node*/
    void update_g();

    /*rewrite ==*/
    bool operator == (const node& another) const;

    /*check whether has a analytical solution*/
    bool has_analytical_solution(const node& goal) const;

    /*check whether the node on the map*/
    bool is_on_grid(const int width, const int height) const;

    /*create son node,dx and dy are measured in car coordinate*/
    node* create_son_node(const double& dx,const double& dy,const double& dt) const;


    

};
}



#endif
#ifndef _NODE2D
#define _NODE2D

#include <cmath>

#include "constance.h"
namespace planner {

class node2d {
    private:
    int _x;/// the x position, along width of the map
    int _y;/// the y position, along height of the map
    double _g;/// the cost-so-far
    double _h;/// the cost-to-go
    int _index;/// the index of the node in the 2D array
    bool _is_in_open;/// the open value
    bool _is_in_closed;/// the closed value
    bool _is_visited;
    node2d* _father_node; /// the predecessor pointer

    public:
    /// The default constructor for 2D array initialization.
    node2d(): node2d(0, 0, 0, 0, nullptr) {}
    /**
    @brief Constructor for a node with the given arguments
    */
    node2d(int x, int y, double g, double h, node2d* father_node) {
        _x=x;
        _y=y;
        _g=g;
        _h=h;
        _father_node=father_node;
        _is_in_closed=false;
        _is_in_open=false;
        _is_visited=false;
        _index=-1;
    }

    /**get x position*/
    int get_x() const { return _x; }

    /**get the y position*/
    int get_y() const { return _y; }

    /** get the cost-so-far (real value)*/
    double get_g() const { return _g; }

    /** get the cost-to-come (heuristic value)*/
    double get_h() const { return _h; }

    /** get the total estimated cost*/
    double get_f() const { return _g + _h; }

    /** get the index of the node in the 2D array*/
    int get_index() const { return _index; }

    /** determine whether the node is open*/
    bool  is_in_open() const { return _is_in_open; }

    /** determine whether the node is closed*/
    bool  is_in_closed() const { return _is_in_closed; }

    /** determine whether the node is discovered*/
    bool  is_visited() const { return _is_visited; }

    /// get a pointer to the father node
    node2d* get_father_node() const { return _father_node; }

    /** set the x position*/
    void set_x(const int& x) { _x = x; }

    /** set the y position*/
    void set_y(const int& y) { _y = y; }

    /** set the cost-so-far (real value)*/
    void set_g(const double& g) { _g = g; }

    /** set the cost-to-come (heuristic value)*/
    void set_h(const double& h) { _h = h; }

    /** set and get the index of the node in the 2D array*/
    int update_index(const int width) { _index = _y * width + _x; return _index;}

    /** open the node*/
    void in_open() { _is_in_open = true; _is_in_closed = false; }

    /** close the node*/
    void in_close() { _is_in_closed = true; _is_in_open = false; }

    /** set the node neither open nor closed*/
    void reset() { _is_in_closed = false; _is_in_open = false; }

    /** discover the node*/
    void visit() { _is_visited = true; }

    /** set a pointer to the predecessor of the node*/
    void set_fathernode(node2d* father_node) { _father_node = father_node; }

    // UPDATE METHODS
    /** Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.*/
    void update_g() { _g += movementCost(*pred); _is_visited = true; }

    /** Updates the cost-to-go for the node x' to the goal node.*/
    void update_h(const node2d& goal) { _h = movementCost(goal); }

    /** The heuristic as well as the cost measure.*/
    double movementcost(const node2d& pred) const { return std::sqrt((_x - pred.get_x()) * (_x - pred.get_x()) + (_y - pred.get_y()) * (_y - pred.get_y())); }

    // CUSTOM OPERATORS
    /// Custom operator to compare nodes. Nodes are equal if their x and y position is the same.
    bool operator == (const node2d& rhs) const;

    // GRID CHECKING
    /** Validity check to test, whether the node is in the 2D array.*/
    bool is_on_grid(const int width, const int height) const;


    // SUCCESSOR CREATION
    /// Creates a successor on a eight-connected grid.
    node2d* create_son_node(const int i);

  // CONSTANT VALUES
    /// Number of possible directions
    static const int dir_num; //可能的方向 
    /// Possible movements in the x direction
    static const int dx[]; //X-方向可能的移动方向
    /// Possible movements in the y direction
    static const int dy[]; //y-方向可能的移动方向

};
}
#endif // NODE2D_H

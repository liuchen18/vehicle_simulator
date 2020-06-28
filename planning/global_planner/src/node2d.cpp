#include "node2d.h"

namespace planner
{



// possible directions
const int node2d::dir_num = 8;
// possible movements：
const int node2d::dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
const int node2d::dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

bool node2d::is_on_grid(const int width, const int height) const {
  return  _x >= 0 && _x < width && _y >= 0 && _y < height;
}

node2d* node2d::create_son_node(const int i) {
  int xSucc = _x + node2d::dx[i];
  int ySucc = _y + node2d::dy[i];
  return new node2d(xSucc, ySucc, _g, 0, this);
}


bool node2d::operator == (const node2d& rhs) const {
  return _x == rhs.get_x() && _y == rhs.get_y();
}

}
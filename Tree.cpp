#include <stdio.h>

#include "Tree.hpp"

Node::Node(double x = 0, double y = 0, Node* parent = nullptr){
  _coord.x = x;
  _coord.y = y;
  _parent = parent;
}


Point& Node::getCoord() {
  return _coord;
}

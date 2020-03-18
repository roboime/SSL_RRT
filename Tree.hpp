#pragma once

#include "util.hpp"

class Node{
private:
  Point _coord;
  Node* _parent;

public:
  Node(double x, double y, Node* parent);
  Point& getCoord();
};

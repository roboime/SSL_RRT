#include <stdio.h>

#include <math.h>

#include "Tree.hpp"

Node::Node(double x = 0, double y = 0, Node* parent = nullptr) : x(x), y(y), _parent(parent){}

Node Node::operator + (const Node& obj){
  Node operation;
  operation.x = this->x + obj.x;
  operation.y = this->y + obj.y;
  return operation;
}

Node Node::operator - (const Node& obj){
  Node operation;
  operation.x = this->x - obj.x;
  operation.y = this->y - obj.y;
  return operation;
}

double Node::operator * (const Node& obj){
  return ((this->x * obj.x) + (this->y * obj.y));
}

Node Node::multiplyByConstant(double c) const{
  Node operation;
  operation.x = c * this->x;
  operation.y = c * this->y;
  return operation;
}

double Node::modulus() const{
  return(sqrt(pow(this->x, 2) + pow(this->y, 2)));
}

Node Node::calculateProjection(const Node& base){
  return (base).multiplyByConstant(((*this) * base)/(pow(base.modulus(), 2)));
}

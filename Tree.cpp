#include <stdio.h>
#include<ctime>
#include <math.h>
#include <float.h>

#include "Tree.hpp"
#include "Environment.hpp"

Node::Node(double x = 0, double y = 0, Node* parent = nullptr) : _x(x), _y(y), _parent(parent){}

Node Node::operator + (const Node& obj){
  Node operation;
  operation._x = this->_x + obj._x;
  operation._y = this->_y + obj._y;
  return operation;
}

Node Node::operator - (const Node& obj){
  Node operation;
  operation._x = this->_x - obj._x;
  operation._y = this->_y - obj._y;
  return operation;
}

double Node::operator * (const Node& obj){
  return ((this->_x * obj._x) + (this->_y * obj._y));
}

Node Node::multiplyByConstant(double c) const{
  Node operation;
  operation._x = c * this->_x;
  operation._y = c * this->_y;
  return operation;
}

double Node::modulus() const{
  return(sqrt(pow(this->_x, 2) + pow(this->_y, 2)));
}

Node Node::makeUnitary(){
  return Node(_x/this->modulus(), _y/this->modulus(), this->_parent);
}


Tree::Tree(double probGoal){
  if (probGoal >=0 && probGoal <= 1) _probGoal = probGoal;
  else _probGoal = 1;
}

Node Tree::extend(Environment& env, ObstacleGrid& obs, Node& current, Node& target, double step){
    srand(time(NULL));
    //gera um valor aleatório para comparar com _probGoal
    double p = (double(rand() % 100 ) / (100));
    //Expandir em torno do obj
    if(p <= _probGoal){
       Node temp = (current + ((target-current).makeUnitary()).multiplyByConstant(step));
       temp._parent = &current;
       if(env.checkWallColision(temp) || obs.checkObstacleColision(current, temp)) return Node(DBL_MAX,DBL_MAX,nullptr);
       else return temp;
    }
    //Exapandir aleatoriamente
    if(p > _probGoal){
      Node temp = env.randomState();
      temp._parent = &current;
      if(env.checkWallColision(temp) || obs.checkObstacleColision(current, temp)) return Node(DBL_MAX,DBL_MAX,nullptr);
      else return temp;
    }
}

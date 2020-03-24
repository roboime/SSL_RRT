#include <stdio.h>
#include<ctime>
#include <math.h>
#include <float.h>

#include "Tree.hpp"
#include "Environment.hpp"

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

Node Node::makeUnitary(){
  return Node(x/this->modulus(), y/this->modulus(), this->_parent);
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

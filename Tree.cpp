#include <stdio.h>
#include<ctime>
#include <math.h>
#include <float.h>

#include "Tree.hpp"
#include "Environment.hpp"

Node::Node(double x = 0, double y = 0, Node* parent = nullptr) : _x(x), _y(y), _parent(parent), _vec(2){
  _vec[0] = _x;
  _vec[1] = _y;
}

Node Node::operator + (const Node& obj){
  Node operation;
  operation._x = this->_x + obj._x;
  operation._y = this->_y + obj._y;
  return Node(operation._x, operation._y, nullptr);
}

Node Node::operator - (const Node& obj){
  Node operation;
  operation._x = this->_x - obj._x;
  operation._y = this->_y - obj._y;
  return Node(operation._x, operation._y, nullptr);
}

double Node::operator * (const Node& obj){
  return ((this->_x * obj._x) + (this->_y * obj._y));
}

bool Node::operator == (const Node& obj){
  if(this->_x == obj._x && this->_y == obj._y && this->_parent == obj._parent) return true;
  else return false;
}

Node Node::multiplyByConstant(double c) const{
  Node operation;
  operation._x = c * this->_x;
  operation._y = c * this->_y;
  return Node(operation._x, operation._y, nullptr);
}

double Node::modulus() const{
  return(sqrt(pow(this->_x, 2) + pow(this->_y, 2)));
}

Node Node::makeUnitary(){
  if(this->modulus() == 0) return Node(0,0,this->_parent);
  else return Node(_x/this->modulus(), _y/this->modulus(), this->_parent);
}


Tree::Tree(Node& root, Node& goal, double probGoal, double probWaypoint, Environment& env, ObstacleGrid& obs, vector<Node>& waypointCache) :
 _root(root), _goal(goal), _env(env), _obs(obs), _waypointCache(waypointCache), _kdtree(flann::KDTreeSingleIndexParams()){
  //iniciar kdtree com o primeiro Node
  _kdtree.buildIndex(flann::Matrix<double>(root._vec.data(), 1, 2));
  //adicionar primeiro node na fila
  _nodemap[root._vec] = root;
  //verificar se probGoal é válido
  if (probGoal >=0 && probGoal <= 1) _probGoal = probGoal;
  else _probGoal = 1;
  //verificar se probWaypoint é válido
  if (probWaypoint >=0 && probWaypoint <= 1) _probWaypoint = probWaypoint;
  else _probWaypoint = 0;
}

Node Tree::chooseTarget(){
  //gera um valor aleatório para comparar com _probGoal
  double p = (double(rand() % 100 ) / (100));
  int i = rand() % (_waypointCache).size();
  //target sendo o objetivo
  if(p>=0 && p <_probGoal) return _goal;
  //target sendo waypoint da iteração passada
  else if(p >=_probGoal && p < _probGoal + _probWaypoint) return (_waypointCache)[i];
  //target aleatório
  else return _env.randomState();
}

Node Tree::calcNN(Node& target){
  //querry da pesquisa
  flann::Matrix<double> query(target._vec.data(), 1, 2);
  //indices em ordem de menor distância
  vector<int> i(query.rows);
  flann::Matrix<int> indices(i.data(), query.rows, 2);
  //distâncias(não usado ainda)
  vector<double> d(query.rows);
  flann::Matrix<double> dists(d.data(), query.rows, 2);
  //fazer pesquisa
  _kdtree.knnSearch(query, indices, dists, 1, flann::SearchParams());
  //retornar Node com coordenadas correspondentes
  double* point = _kdtree.getPoint(indices[0][0]);
  vector<double> temp(point, point+2);
  return _nodemap[temp];
}

bool Tree::extend(double step, Node* last){
  Node target = chooseTarget();
  Node nearest = calcNN(target);
  //calcular proximo node
  Node temp = (nearest + ((target-nearest).makeUnitary()).multiplyByConstant(step));
  //verificar colisão com parede ou obstáculo
  if(_env.checkWallColision(temp) || _obs.checkObstacleColision(nearest, temp)) return false;
  else{
    //setar o parent do ponto
    temp._parent = &_nodemap[nearest._vec];
    addPoints(temp);
    *last = temp;
    return true;
  }
}

bool Tree::grow(double step, double threshold){
  Node* last = new Node(0, 0, nullptr);
  bool valid = false;
  do{
    if(extend(step, last)) valid = _env.distance(*last, _goal) > threshold;
    else valid = true;
  }while(valid);
  //adicionar goal no _nodemap com parent setado
  _goal._parent = &_nodemap[last->_vec];
  addPoints(_goal);
  //setar _parent do _root como nullptr(muito importante)
  _nodemap[_root._vec]._parent = nullptr;
}

void Tree::addPoints(Node& current){
  //Adicionar no nodemap
  _nodemap[current._vec] = current;
  //Adicionar na kdtree
  _kdtree.addPoints(flann::Matrix<double>(_nodemap[current._vec]._vec.data(), 1, 2));
}

vector<Node> Tree::backtrack(){
  Node temp = _goal;
  vector<Node> path = {_goal};
  //repetir até encontrar a origem
  while(!(temp == _root)){
    //encontrar parent do vetor
    temp = _nodemap[(temp._parent)->_vec];
    //adicionar no vetor
    path.push_back(temp);
  }
  return path;
}

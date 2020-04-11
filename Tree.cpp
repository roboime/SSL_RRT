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
  return Node(_x/this->modulus(), _y/this->modulus(), this->_parent);
}


Tree::Tree(Node& root, Node& goal, double probGoal) : _root(root), _goal(goal), _kdtree(flann::KDTreeSingleIndexParams()){
  //iniciar kdtree com o primeiro Node
  _kdtree.buildIndex(flann::Matrix<double>(root._vec.data(), 1, 2));
  //adicionar primeiro node na fila
  _nodemap[root._vec] = root;
  //verificar se probGoal é válido
  if (probGoal >=0 && probGoal <= 1) _probGoal = probGoal;
  else _probGoal = 1;
}

Node Tree::chooseTarget(Environment& env){
  //gera um valor aleatório para comparar com _probGoal
  double p = (double(rand() % 100 ) / (100));
  if(p <= _probGoal) return _goal;
  else return env.randomState();
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

bool Tree::extend(Environment& env, ObstacleGrid& obs, double step){
  Node target = chooseTarget(env);
  Node nearest = calcNN(target);
  //calcular proximo node
  Node temp = (nearest + ((target-nearest).makeUnitary()).multiplyByConstant(step));
  //verificar colisão com parede ou obstáculo
  if(env.checkWallColision(temp) || obs.checkObstacleColision(nearest, temp)) return false;
  else{
    //setar o parent do ponto
    temp._parent = &_nodemap[nearest._vec];
    addPoints(temp);
    return true;
  }

}

void Tree::addPoints(Node& current){
  //Adicionar no nodemap
  _nodemap[current._vec] = current;
  //Adicionar na kdtree
  _kdtree.addPoints(flann::Matrix<double>(_nodemap[current._vec]._vec.data(), 1, 2));
}

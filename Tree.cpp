#include <stdio.h>
#include<ctime>
#include <math.h>
#include <float.h>

#include "Tree.hpp"
#include "Environment.hpp"

Node::Node(double x = 0, double y = 0, Node* parent = nullptr) : _x(x), _y(y), _parent(parent), _vec{x,y}{
}

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
  srand(time(NULL));
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

Node Tree::extend(Environment& env, ObstacleGrid& obs, double step){
  Node target = chooseTarget(env);
  Node nearest = calcNN(target);
}

void Tree::addPoints(Node& current){
  //Adicionar no nodemap
  _nodemap[current._vec] = current;
  //Adicionar na kdtree
  _kdtree.addPoints(flann::Matrix<double>(_nodemap[current._vec]._vec.data(), 1, 2));

}

/*
int main(){
  Node start(0,0,nullptr);
  Node goal(100,100,nullptr);
  Node p1(3,5,nullptr);
  Node p2(9,10,nullptr);
  Node p3(20,50,nullptr);
  Node p4(50,50,nullptr);
  Node p5(80,70,nullptr);
  Node p6(8,1,nullptr);
  Tree tr(start, goal, 1);
  tr.addPoints(p1);
  tr.addPoints(p2);
  tr.addPoints(p3);
  tr.addPoints(p4);
  tr.addPoints(p5);
  tr.addPoints(p6);
  Node res = tr.calcNN(goal);
  cout << res._x << "\n" << res._y;
}
*/

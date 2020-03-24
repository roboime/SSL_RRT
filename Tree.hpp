#pragma once

//Forward declarations
class Environment;
class ObstacleGrid;

class Node{
public:
  double x;
  double y;
  Node* _parent;
  Node(double x, double y, Node* parent);
  //Definir operação de soma
  Node operator + (const Node& obj);
  //Definir operação de subtração
  Node operator - (const Node& obj);
  //Definir operação de produto escalar
  double operator * (const Node& obj);
  //Módulo do vetor;
  double modulus() const;
  //Multiplicar por constante
  Node multiplyByConstant(double c) const;
  //Calcular unitário
  Node makeUnitary();
};

class Tree{
private:
    double _probGoal;
public:
  Tree(double probGoal);
  //função para determinar a extensão da árvore
  Node extend(Environment& env, ObstacleGrid& obs, Node& current, Node& target, double step);
};

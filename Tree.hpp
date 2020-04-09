#pragma once

#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>
#include <vector>
#include <functional>
#include <unordered_map>

using namespace std;

//Forward declarations
class Environment;
class ObstacleGrid;

//Hash-function vector
struct hash_vector {
    template <class T>
    size_t operator()(const vector<T>& p) const
    {
        auto hash1 = hash<T>()(p[0]);
        auto hash2 = hash<T>()(p[1]);
        return hash1 ^ hash2;
    }
};

class Node{
public:
  double _x;
  double _y;
  Node* _parent;
  vector<double> _vec;
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
  //Retornar coordenadas
};

class Tree{
private:
    double _probGoal;
    Node _root;
    Node _goal;
    unordered_map<vector<double>, Node, hash_vector> _nodemap;
    flann::Index<flann::L2_Simple<double>> _kdtree;

public:
  Tree(Node& root, Node& goal, double probGoal);
  //função para determinar a extensão da árvore
  Node extend(Environment& env, ObstacleGrid& obs, double step);
  //função para escolher target
  Node chooseTarget(Environment& env);
  //calcular Neighrest Neighbor
  Node calcNN(Node& current);
  //adicionar pontos na arvore
  void addPoints(Node& current);
};

#pragma once

#include <vector>
#include <utility>

#include "Tree.hpp"

using namespace std;

class Environment{
public:
  //Função para calcular distância entre dois nós
  double distance(Node& current, Node& target);
};

class ObstacleGrid{
public:
  //Vetor de obstáculos (raio, Ponto)
  vector<pair<double, Node>> grid;
  ObstacleGrid(vector<pair<double, Node>> grid);
  //Verificar se ocorre colisão entre current e target
  bool checkObstacleColision(Node& current, Node& target);
};

#pragma once

#include <vector>
#include <utility>

#include "Tree.hpp"

using namespace std;

class Environment{
private:
  int _field_legth;
  int _field_width;
public:
  Environment(int field_legth, int field_width);
  //Função para calcular distância entre dois nós
  double distance(Node& current, Node& target);
  //Função para calcular um ponto aleatório no campo
  Node randomState();
  //Checar colisão com as extremidades
  bool checkWallColision(Node& current);
};

class ObstacleGrid{
public:
  //Vetor de obstáculos (raio, Ponto)
  vector<pair<double, Node>> _grid;
  ObstacleGrid(vector<pair<double, Node>> grid);
  //Verificar se ocorre colisão entre current e target
  bool checkObstacleColision(Node& current, Node& target);
};

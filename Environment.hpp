#pragma once

#include "util.hpp"
#include "Tree.hpp"

class Environment{
public:
  //Função para calcular distância entre dois nodos
  double distance(Node& current, Node& target);
};

#include <math.h>
#include <iostream>
#include <utility>

#include "Environment.hpp"

using namespace std;

double Environment::distance(Node& current, Node& target){
  return (target - current).modulus();
}

ObstacleGrid::ObstacleGrid(vector<pair<double, Node>> grid) : grid(grid){}

bool ObstacleGrid::checkObstacleColision(Node& current, Node& target){
  Node aux_center(0, 0, nullptr);
  double aux_radius;
  double aux;
  //Verificar se ocorre colisão com cada obstáculo
  for(auto i = grid.begin(); i!= grid.end(); ++i){
    //atribuir valores nas variáveis auxiliares
    aux_center = (*i).second;
    aux_radius = (*i).first;
    //Verificar se algum ponto está dentro da circunferência
    if((aux_center - current).modulus() <= aux_radius || (aux_center - target).modulus() <= aux_radius) return true;
    //Verificar se há solução para a intercessão
    //Verificar se o discriminante é maior ou igual a 0
    aux = pow(2*((target - current)*(current - aux_center)), 2) - 4*(pow((target - current).modulus(), 2))*(pow((current - aux_center).modulus(), 2) - pow(aux_radius, 2));
    if(aux >= 0){
      if((sqrt(aux) - 2*((target - current)*(current - aux_center)))/(2*pow((target - current).modulus(), 2)) <= 1 &&
      (sqrt(aux) - 2*((target - current)*(current - aux_center)))/(2*pow((target - current).modulus(), 2)) >= 0) return true;
      if((-sqrt(aux) - 2*((target - current)*(current - aux_center)))/(2*pow((target - current).modulus(), 2)) <= 1 &&
      (-sqrt(aux) - 2*((target - current)*(current - aux_center)))/(2*pow((target - current).modulus(), 2)) >= 0) return true;
    }
  }
  aux = 0;
  return false;
}

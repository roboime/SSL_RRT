#include <math.h>
#include <iostream>
#include <utility>
#include<ctime>

#include "Environment.hpp"

using namespace std;

Environment::Environment(int field_legth, int field_width) : field_legth(field_legth), field_width(field_width) {}

double Environment::distance(Node& current, Node& target){
  return (target - current).modulus();
}

Node Environment::randomState(){
  Node newstate(0, 0, nullptr);
  //gera uma seed diferente
  srand(time(NULL));
  //atribui valores aleatórios para o novo Node
  newstate.x = field_legth * (((double) (rand() % 1000 ) / (1000)) - 0.5 );
  newstate.y = field_width * (((double) (rand() % 1000 ) / (1000)) - 0.5 );
  return newstate;
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
      //Verificar se as solução está entre 0 e 1
      if((sqrt(aux) - 2*((target - current)*(current - aux_center)))/(2*pow((target - current).modulus(), 2)) <= 1 &&
      (sqrt(aux) - 2*((target - current)*(current - aux_center)))/(2*pow((target - current).modulus(), 2)) >= 0) return true;
      if((-sqrt(aux) - 2*((target - current)*(current - aux_center)))/(2*pow((target - current).modulus(), 2)) <= 1 &&
      (-sqrt(aux) - 2*((target - current)*(current - aux_center)))/(2*pow((target - current).modulus(), 2)) >= 0) return true;
    }
  }
  return false;
}

#include <math.h>
#include <iostream>
#include <utility>
#include<ctime>

#include "Environment.hpp"

using namespace std;

Environment::Environment(int field_legth, int field_width) : _field_legth(field_legth), _field_width(field_width) {}

double Environment::distance(Node& current, Node& target){
  return (target - current).modulus();
}

Node Environment::randomState(){
  Node newstate(0, 0, nullptr);
  //gera uma seed diferente
  srand(time(NULL));
  //atribui valores aleatórios para o novo Node
  newstate._x = _field_legth * (((double) (rand() % 1000 ) / (1000)) - 0.5 );
  newstate._y = _field_width * (((double) (rand() % 1000 ) / (1000)) - 0.5 );
  return newstate;
}

bool Environment::checkWallColision(Node& current){
  //Verificar se os módulos das coordenadas são maiores que as dimensões do campo
  if(abs(current._x) >= (_field_legth/2)) return true;
  else if(abs(current._y) >= (_field_width/2)) return true;
  else return false;
}

ObstacleGrid::ObstacleGrid(vector<pair<double, Node>> grid) : _grid(grid){}

bool ObstacleGrid::checkObstacleColision(Node& current, Node& target){
  Node aux_center(0, 0, nullptr);
  double aux_radius;
  double aux;
  //Verificar se ocorre colisão com cada obstáculo
  for(auto i = _grid.begin(); i!= _grid.end(); ++i){
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

/*
int main(){

  Environment env(200, 200);
  Node obs1 (3,4,nullptr);
  Node current(10,13,nullptr);
  Node target(9,12,nullptr);
  ObstacleGrid obs(vector<pair<double, Node>>{make_pair(5, obs1)});
  Tree tr(0);
  Node temp = tr.extend(env, obs, current, target, 2);
  cout << temp.x << "\n" <<temp.y;
}
*/

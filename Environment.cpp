#include <math.h>
#include <iostream>
#include <utility>
#include<ctime>
#include <fstream>

#include "Environment.hpp"

using namespace std;

Environment::Environment(int field_legth, int field_width) : _field_legth(field_legth), _field_width(field_width) {
  //INICIAR SEED
  srand(time(NULL));
}

double Environment::distance(Node& current, Node& target){
  return (target - current).modulus();
}

Node Environment::randomState(){
  Node newstate(0, 0, nullptr);
  //atribui valores aleatórios para o novo Node
  newstate._x = _field_legth * (((double) (rand() % 100 ) / (100)) - 0.5 );
  newstate._y = _field_width * (((double) (rand() % 100 ) / (100)) - 0.5 );
  return Node(newstate._x, newstate._y, nullptr);
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

GERAR PIXMAP

int main(){

  Environment env(200, 200);
  Node obs1 (50,50,nullptr);
  Node start(0,0,nullptr);
  Node goal(100,100,nullptr);
  Tree tr(start, goal, 0.5);
  ObstacleGrid obs(vector<pair<double, Node>>{make_pair(40, obs1)});
  for(int i = 0; i <= 1000;i++) tr.extend(env, obs, 8);
  for (auto x : tr._nodemap)
    cout << x.first[0] << " " << x.first[1] << "\n";
  //printar o ppm
  ofstream myfile;
  myfile.open("ex.ppm");
  int nx = 200;
  int ny = 200;
  int ir = 0; int ig = 0; int ib = 0;
  bool found = false;
  myfile  << "P3\n" << nx << " " << ny << "\n255\n";
  for(int j = ny - 1; j >= 0; j--){
    for(int i = 0; i < nx; i++){
      for (auto x : tr._nodemap){
        if(int(x.first[0]) == i-100 && int(x.first[1]) == j-100){
          ir = 255;
          break;
        }
      }
      if(i == 100 && j == 100) ig = 255;
      if(i == 199 && j == 199) ib = 255;
      myfile << ir << " " << ig << " " << ib << "\n";
      ir = 0;
      ig = 0;
      ib = 0;
    }
  }
}
*/

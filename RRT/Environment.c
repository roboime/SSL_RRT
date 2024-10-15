#include <math.h>
#include <iostream>
#include <utility>
#include<ctime>
#include <fstream>

#include "pch.h"
#include "Environment.h"

using namespace std;

Environment::Environment(int field_legth, int field_width) : _field_legth(field_legth), _field_width(field_width) {}

float Environment::distance(Node& current, Node& target) {
    return (target - current).modulus();
}

Node Environment::randomState() {
    Node newstate(0, 0, nullptr);
    //atribui valores aleatórios para o novo Node
    newstate._x = _field_legth * (((float)(rand() % 100) / (100)) - 0.5);
    newstate._y = _field_width * (((float)(rand() % 100) / (100)) - 0.5);
    return Node(newstate._x, newstate._y, nullptr);
}

bool Environment::checkWallColision(Node& current) {
    //Verificar se os módulos das coordenadas são maiores que as dimensões do campo
    if (abs(current._x) >= (_field_legth / 2)) return true;
    else if (abs(current._y) >= (_field_width / 2)) return true;
    else return false;
}

ObstacleGrid::ObstacleGrid(vector<pair<string, vector<float>>> grid) : _grid(grid) {}

bool ObstacleGrid::pointInRectangle(Node& point, Node& rectCenter, float rectWidth, float rectHeight) {
    float leftX = rectCenter._x - rectWidth / 2;
    float rightX = rectCenter._x + rectWidth / 2;
    float topY = rectCenter._y + rectHeight / 2;
    float bottomY = rectCenter._y - rectHeight / 2;

    return (point._x >= leftX && point._x <= rightX && point._y >= bottomY && point._y <= topY);
}

bool ObstacleGrid::lineIntersectsRectangle(Node& current, Node& target, Node& rectCenter, float rectWidth, float rectHeight) {
    float leftX = rectCenter._x - rectWidth / 2;
    float rightX = rectCenter._x + rectWidth / 2;
    float topY = rectCenter._y + rectHeight / 2;
    float bottomY = rectCenter._y - rectHeight / 2;

    // Verificar se a linha formada pelos pontos current e target intersecta alguma das arestas do retângulo
    if (current._x < leftX && target._x < leftX) return false;
    if (current._x > rightX && target._x > rightX) return false;
    if (current._y > topY && target._y > topY) return false;
    if (current._y < bottomY && target._y < bottomY) return false;

    return true;
}

bool ObstacleGrid::checkObstacleCollision(Node& current, Node& target) {
  // Verificar colisão com cada obstáculo

  //variaveis auxiliare para o circulo
  Node aux_center(0, 0, nullptr);
  float aux_radius;
  float aux;

  //variaveis auxiliare para o retangulo
  Node rect_center(0, 0, nullptr);
  float rect_width, rect_height;

  for (auto i = _grid.begin(); i != _grid.end(); ++i) {
      if ((*i).first == "circle") {
        // Lógica para colisão com obstáculo circular
        aux_radius = (*i).second[0];
        
        aux_center._x = (*i).second[1];
        aux_center._y = (*i).second[2];

        //Verificar se algum ponto está dentro da circunferência
        if ((aux_center - current).modulus() <= aux_radius || (aux_center - target).modulus() <= aux_radius) return true;
        //Verificar se há solução para a intercessão
        //Verificar se o discriminante é maior ou igual a 0
        aux = pow(2 * ((target - current) * (current - aux_center)), 2) - 4 * (pow((target - current).modulus(), 2)) * (pow((current - aux_center).modulus(), 2) - pow(aux_radius, 2));
        if (aux >= 0) {
            //Verificar se as solução está entre 0 e 1
            if ((sqrt(aux) - 2 * ((target - current) * (current - aux_center))) / (2 * pow((target - current).modulus(), 2)) <= 1 &&
                (sqrt(aux) - 2 * ((target - current) * (current - aux_center))) / (2 * pow((target - current).modulus(), 2)) >= 0) return true;
            if ((-sqrt(aux) - 2 * ((target - current) * (current - aux_center))) / (2 * pow((target - current).modulus(), 2)) <= 1 &&
                (-sqrt(aux) - 2 * ((target - current) * (current - aux_center))) / (2 * pow((target - current).modulus(), 2)) >= 0) return true;
        }

      } else if ((*i).first == "rectangle") {
          
        // Atribuir valores nas variáveis auxiliares
        rect_width = (*i).second[0];  
        rect_height = (*i).second[1];  
        rect_center._x = (*i).second[2];
        rect_center._y = (*i).second[3];  
        
        // Passo 1: Verificar se current ou target estão dentro do retângulo
        if (pointInRectangle(current, rect_center, rect_width, rect_height) || 
            pointInRectangle(target, rect_center, rect_width, rect_height)) {
            return true;
        }
        
        // Passo 2: Verificar interseção entre o segmento current-target e as arestas do retângulo
        if (lineIntersectsRectangle(current, target, rect_center, rect_width, rect_height)) {
            return true;
        }  

      }
  }
  return false; // Sem colisão detectada
}


// int main(){
//   int itr = 0;
//   while(itr< 100){
//     auto st = std::chrono::high_resolution_clock::now();
//     itr++;

//     Environment env(200, 200);


//     Node obs1 (50,50,nullptr);

//     Node start(0,0,nullptr);
//     Node goal(100,100,nullptr);

//     vector<Node> iPath = {goal};


//     ObstacleGrid obsCircle(vector<pair<string, vector<float>>>{make_pair("circle", vector<float>{20.0f,50.0f,50.0f})});
//     //ObstacleGrid obsRectangle(vector<pair<string, vector<float>>>{make_pair("rectangle", vector<float>{40.0f,40.0f,50.0f,50.0f})});

//     Tree tr0(start, goal, 0.1, 0.6, env,obs, iPath);
//     tr0.grow();
//     vector<Node> path0 = tr0.backtrack();

//     Tree tr(start, goal, 0.1, 0.6, env,obs, path0);
//     tr.grow();
//     vector<Node> path = tr.backtrack();

//     auto finish = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<float> elapsed = finish - st;
//     std::cout << "Elapsed time: " << elapsed.count() << " s\n";
//   }


//   for (auto x : tr._nodemap){
//     cout << x.first[0] << " " << x.first[1] << " ";
//     if(x.first[0] != 0 &&  x.first[1] !=0)
//       cout << x.second._parent->_x << " " << x.second._parent->_y << "\n";
//   }

//   cout << "\n \n \n \n";
//   for (auto x = path.begin(); x!= path.end(); ++x){
//     cout << (*x)._x << " " << (*x)._y <<"\n";
//   }

// }


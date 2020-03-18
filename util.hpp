#pragma once

#include <math.h>

/**
  *Arquivo contendo funções e classes úteis ao longo do código
  *class Point define simplesmente um ponto e suas operações
*/
class Point{
public:
  double x;
  double y;

  Point(double x = 0, double y = 0) : x(x), y(y){}

  //Definir operação de soma
  Point operator + (const Point& obj){
    Point operation;
    operation.x = this->x + obj.x;
    operation.y = this->y + obj.y;
    return operation;
  }

  //Definir operação de subtração
  Point operator - (const Point& obj){
    Point operation;
    operation.x = this->x - obj.x;
    operation.y = this->y - obj.y;
    return operation;
  }

  //Definir operação de produto escalar
  double operator * (const Point& obj){
    return ((this->x * obj.x) + (this->y * obj.y));
  }

  //Módulo do vetor;
  double modulus(){
    return(sqrt(pow(this->x, 2) + pow(this->y, 2)));
  }

};

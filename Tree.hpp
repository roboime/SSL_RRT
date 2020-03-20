#pragma once

class Node{
private:
  Node* _parent;

public:
  double x;
  double y;
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
  //Calcular projeção
  Node calculateProjection(const Node& base);
};

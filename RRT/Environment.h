#pragma once

#include <vector>
#include <utility>

#include "Tree.h"

using namespace std;

class Environment {
private:
    int _field_legth;
    int _field_width;
public:
    Environment(int field_legth, int field_width);
    //Função para calcular distância entre dois nós
    float distance(Node& current, Node& target);
    //Função para calcular um ponto aleatório no campo
    Node randomState();
    //Checar colisão com as extremidades
    bool checkWallColision(Node& current);
};


//ANTIGO     Obstaclegrid
// class ObstacleGrid {
// public:
//     //Vetor de obstáculos (raio, Ponto)
//     vector<pair<float, Node>> _grid;
//     ObstacleGrid(vector<pair<float, Node>> grid);
//     //Verificar se ocorre colisão entre current e target
//     bool checkObstacleColision(Node& current, Node& target);
// };

class ObstacleGrid {
public:
    // Vector of obstacles (shape, parameters)
    //parametros circle: raio, centro_x, centro_y
    //parametros rect: width, height, centro_x, centro_y

    vector<pair<string, vector<float>>> _grid;

    ObstacleGrid(vector<pair<string, vector<float>>> grid);

    //funcoes auxilares para a colisao de obstaculos retangulares
    bool pointInRectangle(Node& point, Node& rect_center, float rect_width, float rect_height);
    bool lineIntersectsRectangle(Node& current, Node& target, Node& rectCenter, float rectWidth, float rectHeight);
    
    // Check for collision between current and target nodes
    bool checkObstacleCollision(Node& current, Node& target);

};

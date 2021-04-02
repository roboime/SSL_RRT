#include <iostream>
#include <math.h>
#include <tuple>
#include <vector>
#include <chrono>

#include "UVF.cpp"

//definindo a função sign, que retorna +1 se o sinal for positivo, -1 caso negativo e 0 caso 0
int sign(double x){
    return (x > 0) - (x < 0);
}

//controle do veículo dado sua orientação e posição no campo. As constantes de,kr,d_min e delta devem ser ajustadas empíricamente e depois fixadas no código
tuple<double, double> control(tuple<double,double,double> point, tuple<double,double,double> target, vector<tuple<double, double>> obstacle_vector, 
            double de, double kr, double d_min, double delta){
    //distância que a desacelação radial começa
    double rho_min = 5;
    //velocidade máxima linear permitida pelo robo
    double v_max = 2;
    double v;
    double w;
    //constante de ajuste de angulação
    double k_error = 0.25;
    //calculo da distância até o objetivo
    double rho = sqrt(pow((get<0>(point)-get<0>(target)),2)+ pow(get<1>(point)-get<1>(target),2));

    //definição de pontos auxiliares para cálculo do gradiente linearizado em torno do ponto
    tuple<double,double,double> p_lower_x = make_tuple(get<0>(point)-1, get<1>(point), get<2>(point)); 
    tuple<double,double,double> p_upper_x = make_tuple(get<0>(point)+1, get<1>(point), get<2>(point));
    tuple<double,double,double> p_lower_y = make_tuple(get<0>(point), get<1>(point)-1, get<2>(point));
    tuple<double,double,double> p_upper_y = make_tuple(get<0>(point), get<1>(point)+1, get<2>(point));

    //cálculo dos 4 gradientes
    double grad_x;
    double grad_y;
    double grad_x_minus = 0.5*((UVF(p_upper_x, target, obstacle_vector, de, kr, d_min, delta)) - (UVF(p_lower_x, target, obstacle_vector, de, kr, d_min, delta)));
    double grad_x_plus  = 0.5*((UVF(p_upper_x, target, obstacle_vector, de, kr, d_min, delta)) + (UVF(p_lower_x, target, obstacle_vector, de, kr, d_min, delta)));
    double grad_y_minus = 0.5*((UVF(p_upper_y, target, obstacle_vector, de, kr, d_min, delta)) - (UVF(p_lower_y, target, obstacle_vector, de, kr, d_min, delta)));
    double grad_y_plus  = 0.5*((UVF(p_upper_y, target, obstacle_vector, de, kr, d_min, delta)) + (UVF(p_lower_y, target, obstacle_vector, de, kr, d_min, delta)));

    //testagem de qual gradiente não apresenta descontinuidades
    if(abs(grad_x_minus) >= abs(grad_x_plus)){
        grad_x = grad_x_plus;
    }else{
        grad_x = grad_x_minus;
    }
    if(abs(grad_y_minus) >= abs(grad_y_plus)){
        grad_y = grad_y_plus;
    }else{
        grad_y = grad_y_minus;
    }

    //determinação da velocidade radial e angular
    if (rho >= rho_min){
        v = v_max;

        double uvf_angle = UVF(point, target, obstacle_vector, de, kr, d_min, delta);
        double error_angle = remainder(get<2>(point) - uvf_angle, 2 * M_PI);

        w = (grad_x * cos(get<2>(point))  + grad_y * sin(get<2>(point)))* v - k_error * sign(error_angle) * sqrt(abs(error_angle));

    }else{
        v = v_max * (rho / rho_min);

        double uvf_angle = UVF(point, target, obstacle_vector, de, kr, d_min, delta);
        double error_angle = remainder(get<2>(point) - uvf_angle, 2 * M_PI);

        w = (grad_x * cos(get<2>(point))  + grad_y * sin(get<2>(point)))* v - k_error * sign(error_angle) * sqrt(abs(error_angle));
    }
    return make_tuple(v, w);
};
#include <iostream>
#include <math.h>
#include <tuple>
#include <vector>


using namespace std;

//Univector Field code test
//Developed by RoboIME Team 2021

//função gaussiana que será usada na construção do UVF
double gauss(double r, double delta) {
    double g = exp(- pow(r, 2) / (2 * pow(delta, 2)));
    return g;
};


//Espiral hiperbólica (HS), onde tx e ty são parâmetros de ajuste, de é o raio mínimo e kr é a constante de suavização da curva 
double HS(tuple<double,double,double> point , double tx, double ty, double de, double kr, double ccw) {
    double PhiH;
    int signal = 1;
    //ajusta o sinal dependendo da orientação desejada
    if (ccw == 1) {
        signal = -1;
    };

    //definição de variáveis auxiliares
    double theta = atan2(get<1>(point) - ty, get<0>(point) - tx);
    double ro = sqrt(pow(get<0>(point) - tx, 2) + pow(get<1>(point) - ty, 2));

    //casos dependendo do raio mínimo
    if (ro > de) {
        PhiH = theta + signal * M_PI_2 * (2 - (de + kr)/(ro + kr)) ;
    };
    if (ro <= de) {
        PhiH = theta + signal * M_PI_2 * sqrt(ro/de) ;
    };
    //ajusta o ângulo para o intervalo [-pi, pi]
    return remainder(PhiH, 2 * M_PI);    
};


//Campo para desviar de obstáculos (AUF), onde obstacle vector são os obstáculos no campo passados na forma de tuplas <x, y>.
tuple<double, tuple<double,double> ,double> AUF(tuple<double,double,double> point, vector<tuple<double, double>> obstacle_vector) {
    //parâmetros para controlar qual o obstáculo mais próximo ao ponto
    double min_dist = 99999;
    tuple<double, double> min_obst;
    //vetor para inserir todos os cossenos, senos e distâncias calculadas
    vector<tuple<double, double, double>> obstacle_p;
    int i; 
    for(i=0; i< obstacle_vector.size(); i++) {
        //determinação da distância para cada obstáculo no campo
        double dist = sqrt(pow((get<0>(point) - get<0>(obstacle_vector[i])), 2) + pow((get<1>(point) - get<1>(obstacle_vector[i])), 2));
        //guarda as menores distâncias e o obstáculo mais próximo
        if(dist <= min_dist) {
            min_dist = dist;
            min_obst = obstacle_vector[i];
        };
        if(dist == 0){
            continue;
        }
        else{
            //cálculo do cosseno e seno relativo a cada obstáculo no campo
            double cos = (get<0>(point) - get<0>(obstacle_vector[i]))/dist;
            double sin = (get<1>(point) - get<1>(obstacle_vector[i]))/dist;
            tuple<double, double, double> obs_p = make_tuple(cos, sin, dist);
            obstacle_p.push_back(obs_p);
        }
    };
    //cosseno e seno resultantes do AUF
    double cos_r = 0;
    double sin_r = 0;
    for(i=0; i< obstacle_p.size(); i++) {
        cos_r += get<0>(obstacle_p[i]) / get<2>(obstacle_p[i]);
        sin_r += get<1>(obstacle_p[i]) / get<2>(obstacle_p[i]);
    };
    //normalização dos resultados 
    double rho = sqrt(pow(cos_r, 2) + pow(sin_r, 2));
    cos_r = cos_r/rho;
    sin_r = sin_r/rho;
    
    //retorna um ângulo no intervalo [-pi, pi], além das coordenadas do obstáculo mais próximo e a distância classificada como mínima
    return make_tuple(remainder(atan2(cos_r, sin_r), 2 * M_PI), min_obst, min_dist);  
};

int main()
{
    double x;
    double y;
    double o;
    tuple<double,double,double> current;
    vector<tuple<double,double>> obstacle_vector;
    obstacle_vector.push_back(make_tuple(0,0));
    cin >> x >> y >> o;
    current = make_tuple(x, y, o); 
    cout << get<0>(AUF(current, obstacle_vector)) << endl;
    cout << get<0>(get<1>(AUF(current, obstacle_vector))) << get<1>(get<1>(AUF(current, obstacle_vector))) << endl;
    cout << get<2>(AUF(current, obstacle_vector)) << endl;
    return 0;
};
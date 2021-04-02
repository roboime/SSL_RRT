#include <iostream>
#include <math.h>
#include <tuple>
#include <vector>
#include <chrono>

using namespace std;

//Univector Field code test
//Developed by RoboIME Team 2021

//função gaussiana que será usada na construção do UVF pela composição do TUF e do AUF
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


//Campo que gera um caminho simples até o objetivo (TUF) na orientação desejada. Os parâmetros de e kr serão utilizados
//para gerar as HS. Essa função retorna um ângulo entre [-pi, pi]
double TUF(tuple<double,double,double> point, tuple<double,double,double> target, double de, double kr){
    //obtém a orientação do target, que será utilizada para fazer a rotação do sistema coordenado
    double rot = -remainder(get<2>(target), 2 * M_PI);
    double tuf_angle;
    //cria um ponto auxiliar para realizar a rotação e translação
    tuple<double,double,double> tmppoint;

    get<0>(tmppoint) = (get<0>(point) * cos(rot) - get<1>(point) * sin(rot)) - (get<0>(target) * cos(rot) - get<1>(target) * sin(rot));
    get<1>(tmppoint) = (get<0>(point) * sin(rot) + get<1>(point) * cos(rot)) - (get<0>(target) * sin(rot) + get<1>(target) * cos(rot));

    //cria os pontos auxiliares para a determinação das espirais hiperbólicas
    tuple<double,double, double> p_l = make_tuple(get<0>(tmppoint), get<1>(tmppoint) + de, 0);
    tuple<double,double, double> p_r = make_tuple(get<0>(tmppoint), get<1>(tmppoint) - de, 0);

    if(get<1>(tmppoint) <= - de){
        tuf_angle = HS(p_l, 0, 0, de, kr, 1);
    }
    else if(get<1>(tmppoint) >=  de){
        tuf_angle = HS(p_r, 0, 0, de, kr, 0);
    }
    else {
        //definição das duas espirais hiperbólicas
        double phiCCW = HS(p_r, 0, 0, de, kr, 0);
        double phiCW  = HS(p_l, 0, 0, de, kr, 1);

        tuple<double,double> NhCW   = make_tuple(cos(phiCW), sin(phiCW));
        tuple<double,double> NhCCW  = make_tuple(cos(phiCCW), sin(phiCCW));

        //definição das componentes do campo resultante
        double tuf_cos = (abs(get<1>(p_l))*get<0>(NhCCW) + abs(get<1>(p_r))*get<0>(NhCW))/(2.0*de);
        double tuf_sin = (abs(get<1>(p_l))*get<1>(NhCCW) + abs(get<1>(p_r))*get<1>(NhCW))/(2.0*de);
        
        tuf_angle = atan2(tuf_sin, tuf_cos);
    };
    //retorna a orientação do campo original
    return remainder(tuf_angle - rot, 2 * M_PI);
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
    return make_tuple(remainder(atan2(sin_r, cos_r), 2 * M_PI), min_obst, min_dist);  
};


//Campo resultante. retorna o ângulo final para cálculo das velocidades
double UVF(tuple<double,double,double> point, tuple<double,double,double> target, vector<tuple<double, double>> obstacle_vector, 
            double de, double kr, double d_min, double delta){
    //vetor com as informações do AUF
    tuple<double, tuple<double,double> ,double> auf_vector = AUF(point, obstacle_vector);
    double tuf_angle = TUF(point, target, 5, 5);
    double uvf_angle;

    cout << get<0>(auf_vector) << endl;
    cout << tuf_angle << endl;

    //caso a distância seja menor que um mínimo, ele deverá considerar apenas a angulação do AUF. Caso contrário, deverá ser feita a composição
    if(get<2>(auf_vector) < d_min){
        uvf_angle = get<0>(auf_vector);
    }else{
        if(abs(tuf_angle - get<0>(auf_vector)) >= (2 * M_PI - abs(tuf_angle - get<0>(auf_vector)))){
            if(get<0>(auf_vector) < 0){    
                uvf_angle = remainder(((2 * M_PI + get<0>(auf_vector)) * gauss(get<2>(auf_vector) - d_min, delta) + (tuf_angle) * (1 - gauss(get<2>(auf_vector) - d_min, delta))), 2 * M_PI);
            }
            if(tuf_angle < 0){    
                uvf_angle = remainder(((get<0>(auf_vector)) * gauss(get<2>(auf_vector) - d_min, delta) + (2 * M_PI + tuf_angle) * (1 - gauss(get<2>(auf_vector) - d_min, delta))), 2 * M_PI);
            }         
        }else{
            uvf_angle = remainder((get<0>(auf_vector)) * gauss(get<2>(auf_vector) - d_min, delta) + (tuf_angle) * (1 - gauss(get<2>(auf_vector) - d_min, delta)), 2 * M_PI);
        }

    }
    return uvf_angle;
};

int main()
{
    double x;
    double y;
    double o;
    tuple<double,double,double> current;
    tuple<double,double,double> target = make_tuple(10, 0, M_PI_2);
    vector<tuple<double,double>> obstacle_vector;
    obstacle_vector.push_back(make_tuple(0,0));
    cin >> x >> y >> o;
    current = make_tuple(x, y, o); 
    // Record start time
    auto start = chrono::high_resolution_clock::now();
    cout << UVF(current,target,obstacle_vector, 5,5,5,3) << endl;
    // Record end time
    auto finish = std::chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = finish - start;
    cout << "Elapsed time: " << elapsed.count();
    return 0;
};
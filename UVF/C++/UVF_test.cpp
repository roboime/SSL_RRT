#include <iostream>
#include <math.h>
#include <tuple>


using namespace std;

//Univector Field code test
//Developed by RoboIME Team 2021

//função gaussiana que será usada na construção do UVF
double gauss(double r, double delta) {
    double g = exp(- pow(r, 2) / (2 * pow(delta, 2)));
    return g;
};


//Espiral hiperbólica (HS), onde tx e ty são parâmetros de ajuste, de é o raio mínimo e kr é a constante de suavização da curva 
double HS(std::tuple<double,double,double> point , double tx, double ty, double de, double kr, double ccw) {
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


int main()
{
    double x;
    double y;
    double o;
    double g;
    std::tuple<double,double,double> current;
    cin >> g;
    cout <<  gauss(g, 1) << endl;
    cin >> x >> y >> o;
    current = make_tuple(x, y, o); 
    cout << HS(current, 0, 0, 5, 5, 0) << endl;
    return 0;
};
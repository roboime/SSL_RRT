#include <math.h>
#include <iostream>

#include "Environment.hpp"

using namespace std;

double Environment::distance(Node& current, Node& target){
  return (target.getCoord() - current.getCoord()).modulus();
}

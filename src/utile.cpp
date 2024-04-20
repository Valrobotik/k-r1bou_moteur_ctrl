#include "utile.hpp"

double mod2pi(double a){
    return a - 2*PI*floor(a/(2*PI));
}
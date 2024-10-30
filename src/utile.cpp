#include "utile.hpp"

double mod2pi(double a){
    return a - 2*PI*floor(a/(2*PI));
}

int indexOf(String str, char c)
{
    for (unsigned int i = 0; i < str.length(); i++)
    {
        if (str[i] == c)
            return i;
    }
    return -1;
}

#include <cmath>


int signum(double input) {
    int output = (input > 0) - (input < 0);

    return output;
}
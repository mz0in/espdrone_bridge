#ifndef _AC_CONTROL_UTILS_H_
#define _AC_CONTROL_UTILS_H_
#include <cmath>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <assert.h>

float sigmoid(float x)
{
    return (1 / (1 + exp(-x)));
}

double sigmoid(double x)
{
    return (1 / (1 + exp(-x)));
}

double sigmoid(double x, double x0, double xt, double scale = 10.)
{
    assert(xt > x0);
    assert(x >= x0);
    assert(xt >= x);
    x = 2 * scale * (x - x0) / (xt - x0) - scale;
    return (1 / (1 + exp(-x)));
}

double sigmoid_round(double x, double x0, double xt, double scale = 10.)
{
    assert(xt > x0);
    x = 2 * scale * (x - x0) / (xt - x0) - scale;
    return ceil(1000 / (1 + exp(-x)))/1000.;
}
#endif
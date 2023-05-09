#ifndef __AERIAL_CATCHER_UAM_PARAM__
#define __AERIAL_CATCHER_UAM_PARAM__
#include <iostream>
#include <aerial_catcher/colors.h>

class uam_param
{
private:
    double d = 0.0;         // arm base joint to COM/rotors center
    double l = 0.;          // arm length
    double r = 0.;          // largest dist from propeller tip to COM/rotors center
    double r_thick = 0.;    // propeller thick 
    double r_n = 0.;        // net diameter
    double r_ring = 0.;     // I DON'T REMEMBER
public:
    uam_param(/* args */);
    ~uam_param();
    void print();
    void set_param(double d_, double l_, double r_, double r_thick_, double r_n_, double r_ring_);
    double get_d();
    double get_l();
    double get_r();
    double get_r_thick();
    double get_r_n();
    double get_r_ring();
};


#endif
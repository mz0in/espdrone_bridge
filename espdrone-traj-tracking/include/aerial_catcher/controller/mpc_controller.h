/*
 * Copyright 2022 Wang Siqiang, NROS, HITSZ, China
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __MPC_CONTROLLER_H__
#define __MPC_CONTROLLER_H__

// Acado

#include <Eigen/Eigen>
#include <stdio.h>
#include <chrono>
#include <ctime>
#include <time.h>
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado/function/function.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <aerial_catcher/predictor/uam_param.h>
#include <aerial_catcher/controller/basic_controller.h>
#include <yaml-cpp/yaml.h>
USING_NAMESPACE_ACADO

class mpc_controller : public basic_controller
{
private:
    uam_param param;
    double lr;
    double l1;
    double l2;
    double qoffx;
    double qoffz;
    double l11;
    double l12;
    double l13;
    double l14;
    double l21;
    double l22;
    double l23;
    double l24;
    double l25;
    double ts, te;
    double qr, hr;
    double qxx, qyy, qzz, qgg, qq1, qq2;
    double start_x, start_y, start_z, start_vx, start_vy, start_vz, start_yaw, start_q1, start_q2;
    double end_x, end_y, end_z, end_vx, end_vy, end_vz;
    std::unique_ptr<OptimizationAlgorithm> algorithm;
    GnuplotWindow window;
        DifferentialState x0, y0, z0,
        x1, y1, z1,
        x2, y2, z2,
        x3, y3, z3,
        ga0, ga1,
        q1;

    Control x4, y4, z4, ga2;
    Control qd1;
    IntermediateState eex, eey, eez, T, r, p;
    IntermediateState pq1x, pq1y, pq1z;
    IntermediateState pq2x, pq2y, pq2z;
    IntermediateState pq3x, pq3y, pq3z;
    IntermediateState pq4x, pq4y, pq4z;
    IntermediateState p11x, p11y, p11z;
    IntermediateState p12x, p12y, p12z;
    IntermediateState p13x, p13y, p13z;
    IntermediateState p14x, p14y, p14z;
    IntermediateState p21x, p21y, p21z;
    IntermediateState p22x, p22y, p22z;
    IntermediateState p23x, p23y, p23z;
    IntermediateState p24x, p24y, p24z;
    IntermediateState p25x, p25y, p25z;


public:
    bool solved;
    int numSteps;
    mpc_controller(/* args */);
    ~mpc_controller();
    bool run_once(double* mpc_time);
    void solve();
    void set_target(double goal_x, double goal_y, double goal_z, double goal_vx, double goal_vy, double goal_vz, double t_end);
    void set_start(double start_x, double start_y, double start_z, double start_vx, double start_vy, double start_vz);
    void step(std::vector<double> x_i, std::vector<double> u_i, std::vector<double>* x_next, double dt);
    void load_param(std::string path_to_cfg);
    std::vector<std::vector<double> > mpc_u;
    std::vector<std::vector<double> > mpc_x;
};

#endif //__MPC_CONTROLLER_H__
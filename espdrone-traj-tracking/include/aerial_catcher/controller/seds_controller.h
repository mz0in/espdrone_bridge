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

#ifndef __SEDS_CONTROLLER_H__
#define __SEDS_CONTROLLER_H__
#include <iostream>
#include <sstream>
#include <fstream>
#include "seds/GMR.h"
#include <aerial_catcher/colors.h>
#include <aerial_catcher/controller/basic_controller.h>
using namespace MathLib;
class seds_controller : public basic_controller
{
private:
    GaussianMixture mySEDS;
    std::string seds_cfg = "/home/leonidas/arm_ws/src/aerial_catcher/resource/seds/mySEDSModel4.txt";
    std::string csv_file = "/home/leonidas/arm_ws/src/aerial_catcher/data/seds/trajs.csv";
    std::ofstream outFile;
    int d = 3;
    int count = 0;
    double hz;
    bool target_initialized;
public:
    bool solved;
    MathLib::Vector x, xd, xT;
    MathLib::Vector kai;
    seds_controller();
    ~seds_controller();
    void set_target(double goal_x, double goal_y, double goal_z, double goal_vx, double goal_vy, double goal_vz, double t_end);
    void set_start(double s_x, double s_y, double s_z, double s_vx, double s_vy, double s_vz);
    void set_state(double s_x, double s_y, double s_z);
    void run_once();
    Eigen::Vector3d get_pos_cmd();
    Eigen::Vector3d get_vel_cmd();
};




#endif
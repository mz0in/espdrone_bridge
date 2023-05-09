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

#ifndef __BASIC_CONTROLLER_H__
#define __BASIC_CONTROLLER_H__

#include <iostream>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <aerial_catcher/colors.h>
#include <aerial_catcher/common.h>

class basic_controller
{
private:


protected:
    int hz;
    bool target_initialized;
    clock_t beginTime;
    clock_t endTime;
    int counter_t;
    Eigen::Vector3d vel_cmd = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos_cmd = Eigen::Vector3d::Zero();

public:
    bool solved;
    basic_controller(/* args */);
    ~basic_controller();
    void set_target(double goal_x, double goal_y, double goal_z, double goal_vx, double goal_vy, double goal_vz, double t_end);
    void set_start(double s_x, double s_y, double s_z, double s_vx, double s_vy, double s_vz);
    void set_state(double s_x, double s_y, double s_z);
    Eigen::Vector3d get_vel_cmd();
    Eigen::Vector3d get_pos_cmd();

    void solve();
    void run_once();
};

#endif
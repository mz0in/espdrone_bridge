#ifndef __AERIAL_CATCHER_PREDICTOR__
#define __AERIAL_CATCHER_PREDICTOR__

#include <iostream>
#include <cmath>
#include <vector>
#include <sys/timeb.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <aerial_catcher/colors.h>
#include <aerial_catcher/common.h>
#include <aerial_catcher/predictor/uam_param.h>
#include <Eigen/Eigen>

class trajPredictor
{
private:
    double time;
    bool test;
    std::vector<Eigen::Vector3d> predict_position;
    std::vector<Eigen::Vector3d> predict_velocity;
    bool traj_set;
    Eigen::Vector3d position_s;
    Eigen::Vector3d velocity_s;
public:
    bool intercept_set;
    double dt;
    int dim_px;
    int dim_py;
    int dim_pz;
    int dim_vx;
    int dim_vy;
    int dim_vz;
    std::vector<double> param_px;
    std::vector<double> param_py;
    std::vector<double> param_pz;
    std::vector<double> param_vx;
    std::vector<double> param_vy;
    std::vector<double> param_vz;
    uam_param param;
    
    trajPredictor(bool iftest);
    ~trajPredictor();

    void Print();
    void GetDerivative();
    void SetDT(double t);
    void SetTrajectory(const std_msgs::Float64MultiArrayConstPtr traj);
    void StepTrajectory(Eigen::Vector3d *traj, double time);
    void SetDrone(std::vector<double> p);
    void GetpositionAtTime(double time, Eigen::Vector3d *position);
    void GetVelocityAtTime(double time, Eigen::Vector3d *velocity);
    void GetEE(Eigen::Vector3d *pose,  Eigen::Vector3d *ee, double angle);
    void GetEE(Eigen::Vector3d *pose,  Eigen::Vector3d *ee, geometry_msgs::PoseStamped real_ee, geometry_msgs::PoseStamped real_base);
    void GetEEV(Eigen::Vector3d *v_ee, double w, double angle);
    void GetBase(Eigen::Vector3d *ee,  Eigen::Vector3d *base, double angle);
    void GetBase(Eigen::Vector3d *ee,  Eigen::Vector3d *base, geometry_msgs::PoseStamped real_ee, geometry_msgs::PoseStamped real_base);
    void Decomposition(Eigen::Vector3d *v_ori, Eigen::Vector3d *velocity, Eigen::Vector3d *v_tgt, Eigen::Vector3d *v_norm);
    bool CheckCollision(Eigen::Vector3d *pose);
    void SetPosition(Eigen::Vector3d *position);
    void SetVelocity(Eigen::Vector3d *velocity);
    void GetVelocity(Eigen::Vector3d *velocity);
    void PredictSteps(double time_now);
    void CalculateIntercept(double *time, Eigen::Vector3d *position, Eigen::Vector3d *velocity);
    double GetTime();

};

#endif
#include <iostream>
#include <Eigen/Eigen>
#include <aerial_catcher/colors.h>
#include <aerial_catcher/common.h>
#include <aerial_catcher/iLQR_uav_pathplanner.h>
#include <mav_msgs/eigen_mav_msgs.h>
// #include <ros/ros.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <yaml-cpp/yaml.h>
#include <aerial_catcher/controller/basic_controller.h>

class ilqr_controller : public basic_controller
{
private:
    Matrix_ac<X_DIM> x, x_last;
    Matrix_ac<X_DIM> xdot;
    Matrix_ac<4> u = zero<4>(); //just used for calculating control
    clock_t beginTime;
    clock_t endTime;
    int counter_t;
    

public:
    size_t numIter;
    std::vector<Matrix_ac<U_DIM, X_DIM>> L;
    std::vector<Matrix_ac<U_DIM>> l;
    double solve_time;
    bool solved;
    ilqr_controller();
    ~ilqr_controller();
    void set_environment_param();
    // void set_target(double t, Eigen::Vector3d sp, Eigen::Vector3d sv, Eigen::Vector3d tp, Eigen::Vector3d tv);
    void msgMavTrajectoryFromState(const Matrix_ac<X_DIM> state, trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);
    void load_param(std::string path_to_cfg);
    bool solve();

    void set_target(double goal_x, double goal_y, double goal_z, double goal_vx, double goal_vy, double goal_vz, double t_end);
    void set_start(double s_x, double s_y, double s_z, double s_vx, double s_vy, double s_vz);
    void set_state(double s_x, double s_y, double s_z);
    Eigen::Vector3d get_vel_cmd();
    Eigen::Vector3d get_pos_cmd();
    void run_once();

};

/*
 * Copyright 2022 Siqiang Wang, NROS, HITSZ, China
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
#ifndef FIELD_PX4_STATIC_H
#define FIELD_PX4_STATIC_H

// std libs
#include <iostream>
#include <sstream>
#include <fstream>
#include <thread>
#include <chrono>
#include <cmath>
// #include <boost/bind.hpp>
#include <Eigen/Eigen>

// ros libs
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
// #include <yaml-cpp/yaml.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// #include <mav_msgs/RollPitchYawrateThrust.h>
// #include <mav_msgs/Actuators.h>
// #include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
// #include <mav_msgs/eigen_mav_msgs.h>
// #include <gazebo_msgs/GetJointProperties.h>
// #include <gazebo_msgs/GetLinkState.h>
// #include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// custom libs
#include <aerial_catcher/matrix.h>
#include <aerial_catcher/colors.h>
#include <aerial_catcher/common.h>
#include <aerial_catcher/control_utils.h>
#include <aerial_catcher/parameters_ros.h>
// #include <aerial_catcher/predictor/predictor.h>
#include <aerial_catcher/lee_position_controller.h>
#include <seds/GMR.h>

class field_esp_base
{
protected:
    // geometry_msgs::Vector3 sim_vec;
    // geometry_msgs::Vector3 err_vec;
    nav_msgs::Odometry current_odom;
    // mav_msgs::RollPitchYawrateThrust rpyt_msg;
    // mav_msgs::Actuators motors;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    Eigen::Vector3d cmd_acc;
    // mav_msgs::EigenTrajectoryPoint eigen_reference;
    // gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
    // gazebo_msgs::GetLinkState get_ball_state_srv_msg;
    double current_roll, current_pitch, current_yaw;
    double joint_angle;
    double joint_angle_last;
    double angle_last_time;
    double angle_vel;
    int joint_seq;
    double uam_d;
    double uam_l;
    double uam_r;
    double uam_r_thick;
    double uam_r_n;
    double uam_r_ring;
    double total_dist;
    double current_dist;
    bool total_dist_initialized = false;
    int counter = 0;
    bool get_intercept = false;
    bool not_solved = true;
    bool get_traj = false;
    bool catched = false;
    bool solved = false;
    double time_inter;
    double joint_sub = 0.0;
    int sum;

    // trajPredictor predictor = new trajPredictor(false);
    Eigen::Vector3d position_inter, velocity_inter, ball_position, ball_velocity;
    Eigen::Vector3d start_postion;
    Eigen::Vector3d ball_vel_th;
    mav_msgs::EigenTrajectoryPoint point;
    mavros_msgs::AttitudeTarget attitude;
    std::ofstream out_file;
    std_msgs::Float64MultiArray intercept_msg;
    std_msgs::Float64 joint_cmd;
    nav_msgs::Path path;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped current_pose, net_pose, target_pose;
    geometry_msgs::TwistStamped current_velocity;
    tf::TransformBroadcaster br;
    tf::Matrix3x3 m0, m1, mee;
    tf::Quaternion q, q_now;
    // visualization_msgs::Marker ee;
    // visualization_msgs::Marker quad_body_marker1;
    // visualization_msgs::Marker quad_body_marker2;
    // visualization_msgs::Marker quad_body_marker3;
    // visualization_msgs::Marker quad_body_marker4;
    // visualization_msgs::Marker link1marker;
    ros::ServiceClient get_jnt_state_client, get_ball_state_client;
    ros::Publisher plan_pub;
    ros::Publisher joint_pub;
    ros::Publisher land_pub;
    ros::Publisher local_pos_pub;
    ros::Publisher sim_vec_pub;
    ros::Publisher err_vec_pub;
    ros::Publisher waypoint_pub_;
    ros::Publisher acc_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher thrust_pub_;
    ros::Publisher attitude_pub_;
    ros::Publisher rpy_pub;
    ros::Publisher path_pub;
    ros::Publisher marker_pub;
    ros::Publisher quad_pub;
    ros::Subscriber current_pose_sub;
    ros::Subscriber intercept_sub;
    ros::Subscriber trj_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber catch_sub;
    ros::Subscriber state_sub;
    ros::Subscriber jnt_sub;
    ros::Subscriber motor_sub;
    ros::Subscriber rb2_sub;
    ros::Subscriber rb3_sub;
    ros::Subscriber rb4_sub;
    ros::Subscriber net_sub;
    ros::Subscriber target_sub;
    ros::NodeHandle nh;
    // aerial_catcher::LeePositionController lee_position_controller_;

public:
    field_esp_base(int argc, char **argv);
    ~field_esp_base();
    // void process_rotors(const mav_msgs::EigenTrajectoryPoint &c_t, mav_msgs::Actuators &actuator_msg);
    bool use_position = false;
    bool enable_log = true;
    bool use_gazebo = false;
    int px4_ctrl_mode = 4;  // 0 for waypoint; 1 for acc; 2 for pose; 3 for thrust; 4 for attitude
    int counter_th = 20;
    int rate = 400;
    double X_MIN = -20.;
    double X_MAX = 20.;
    double Y_MIN = -20.;
    double Y_MAX = 20.;
    double Z_MIN = -0.5;
    double Z_MAX = 20.;
    double INIT_X = -1.;
    double INIT_Y = 0.4;
    double INIT_Z = 1.5;
    double ppx;
    double ppy;
    double ppz;
    double pvx;
    double pvy;
    double pvz;
    double mass;
    double pth;
    double angx;
    double angy;
    double angz;
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void intercept_cb(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void trj_cb(const std_msgs::Float64MultiArrayConstPtr msg);
    void catch_cb(const std_msgs::Bool::ConstPtr &msg);
    void msg_mav_trajectory_from_state(const Matrix_ac<12> state, trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);
    void msg_mav_trajectory_from_state(MathLib::Vector state, trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);
    void main_loop();
    virtual void plan_loop();
    void start_process();
    void load_param();
    void initialize_params(const ros::NodeHandle &private_nh_);
    void refresh_once();
    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void joint_cb(const std_msgs::Float64::ConstPtr &msg);
    void net_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void run();
    virtual void controller_run_once();
    virtual Eigen::Vector3d get_vel_cmd();
    virtual Eigen::Vector3d get_pos_cmd();
    virtual void set_target(double goal_x, double goal_y, double goal_z, double goal_vx, double goal_vy, double goal_vz, double t_end);
    virtual void set_start(double s_x, double s_y, double s_z, double s_vx, double s_vy, double s_vz);
    void print_basic_params();
    void land();
    void check_safty(Eigen::Vector3d position);
    void compute_desired_acceleration(Eigen::Vector3d *acceleration);
    void compute_desired_attitude(Eigen::Vector4d *attitude);
    void publish_cmd();
    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des);
    void target_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

#endif /*FIELD_PX4_STATIC_H*/
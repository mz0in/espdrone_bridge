#include "ac_lee_position_controller_esp.h"
#include <ostream>

static Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d &m)
{
    Eigen::Vector3d v;
    // TODO: Sanity checks if m is skew symmetric
    v << m(7), m(2), m(3);
    return v;
}

inline void vectorFromSkewMatrix(Eigen::Matrix3d &skew_matrix, Eigen::Vector3d *vector)
{
    *vector << skew_matrix(2, 1), skew_matrix(0, 2), skew_matrix(1, 0);
}
field_esp_base::field_esp_base(int argc, char **argv)
{
    ros::init(argc, argv, "field_esp_base");
    std::string log_path("/data/trajectory/px4/");
    load_param();
    current_pose_sub = 
        nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &field_esp_base::local_pose_cb, this);

    odom_sub = 
        nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &field_esp_base::odom_cb, this);

    target_sub = 
        nh.subscribe<geometry_msgs::PoseStamped>("/aerial_catcher/esp/target_pose", 10, &field_esp_base::target_cb, this);

    // local_pos_pub = 
    //     nh.advertise<geometry_msgs::PoseStamped>("/aerial_catcher/esp/pose", 10);

    // acc_pub_ = 
    //     nh.advertise<geometry_msgs::Vector3Stamped>("/aerial_catcher/esp/accel", 10);

    attitude_pub_ = 
        nh.advertise<mavros_msgs::AttitudeTarget>("/aerial_catcher/esp/attitude", 10);

    path_pub = 
        nh.advertise<nav_msgs::Path>("/aerial_catcher/esp/path", 10);

    path.header.frame_id = "world";
    // nh.param("/firefly/use_gazebo", use_gazebo, use_gazebo);
    // nh.param("/path", log_path, log_path);
    // nh.param("/x_max", X_MAX, X_MAX);
    // nh.param("/x_min", X_MIN, X_MIN);
    // nh.param("/y_max", Y_MAX, Y_MAX);
    // nh.param("/y_min", Y_MIN, Y_MIN);
    // nh.param("/z_max", Z_MAX, Z_MAX);
    // nh.param("/z_min", Z_MIN, Z_MIN);
    // nh.param("/x_init", INIT_X, INIT_X);
    // nh.param("/y_init", INIT_Y, INIT_Y);
    // nh.param("/z_init", INIT_Z, INIT_Z);
    // nh.param("/ppx", ppx, 6.);
    // nh.param("/ppy", ppy, 6.);
    // nh.param("/ppz", ppz, 12.);
    // nh.param("/pvx", pvx, 4.7);
    // nh.param("/pvy", pvy, 4.7);
    // nh.param("/pvz", pvz, 8.0);
    // nh.param("/mass", mass, 2.31);
    // nh.param("/pth", pth, 0.8);
    // nh.param("/angx", angx, -3.);
    // nh.param("/angy", angy, -3.);
    // nh.param("/angz", angz, -3.);

    initialize_params(nh);
    ball_position = Eigen::Vector3d::Zero();
    // controller setup
    ros::Rate r(rate);

    pose.pose.position.x = current_pose.pose.position.x;
    pose.pose.position.y = current_pose.pose.position.y;
    pose.pose.position.z = current_pose.pose.position.z;

    pose.pose.orientation.x = 0.;
    pose.pose.orientation.y = 0.;
    pose.pose.orientation.z = 0.;
    pose.pose.orientation.w = 1.;
    trajectory_msg.points.clear();
}

field_esp_base::~field_esp_base()
{
}

void field_esp_base::target_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    target_pose = *msg;
    point.position_W = Eigen::Vector3d(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    point.velocity_W = Eigen::Vector3d(0.,0.,0.);
    point.acceleration_W = Eigen::Vector3d(0.,0.,0.);
    point.setFromYaw(0.);
}

void field_esp_base::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_velocity = *msg;
}

void field_esp_base::odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_odom = *msg;
    compute_desired_acceleration(&cmd_acc);
}

void field_esp_base::compute_desired_acceleration(Eigen::Vector3d *acceleration)
{
    assert(acceleration);
    aerial_catcher::EigenOdometry odometry;
    aerial_catcher::eigenOdometryFromMsgOdom(current_odom, &odometry);
    Eigen::Vector3d position_error;
    position_error = odometry.position - point.position_W;
    const Eigen::Matrix3d R_W_I = odometry.orientation.toRotationMatrix();
    Eigen::Vector3d velocity_W = odometry.velocity;
    Eigen::Vector3d velocity_error;
    velocity_error = velocity_W - point.velocity_W;
    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
    *acceleration = (position_error.cwiseProduct(Eigen::Vector3d(ppx, ppy, ppz)) + velocity_error.cwiseProduct(Eigen::Vector3d(pvx, pvy, pvz))) / mass - 9.80 * e_3 - point.acceleration_W;
}

void field_esp_base::local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    current_odom.pose.pose.position.x = current_pose.pose.position.x;
    current_odom.pose.pose.position.y = current_pose.pose.position.y;
    current_odom.pose.pose.position.z = current_pose.pose.position.z;
    current_odom.pose.pose.orientation.x = current_pose.pose.orientation.x;
    current_odom.pose.pose.orientation.y = current_pose.pose.orientation.y;
    current_odom.pose.pose.orientation.z = current_pose.pose.orientation.z;
    current_odom.pose.pose.orientation.w = current_pose.pose.orientation.w;
}

void field_esp_base::msg_mav_trajectory_from_state(const Matrix_ac<12> state, trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg)
{
    trajectory_msg.header.stamp = ros::Time::now();
    Eigen::Matrix3d desired_ori;
    Eigen::Vector3d desired_position(0., 0., 1.5);
    Eigen::Vector3d desired_velocity(0., 0., 0.);

    // mav trajectory
    double desired_yaw = state[8];
    desired_ori = Eigen::AngleAxisd(state[6], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(state[7], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(state[8], Eigen::Vector3d::UnitZ());
    desired_position << state[0], state[1], state[2];
    desired_velocity << state[3], state[4], state[5];
    // point.velocity_W = desired_velocity;
    // point.position_W = desired_position;
    // point.setFromYaw(desired_yaw);
    // Eigen::Quaterniond doo(desired_ori.transpose());
    // point.orientation_W_B = doo;
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(point, &trajectory_msg);
}

void field_esp_base::publish_cmd()
{
    // cout << "ppz is " << ppz << endl;
    aerial_catcher::EigenOdometry odometry;
    aerial_catcher::eigenOdometryFromMsgOdom(current_odom, &odometry);
    attitude.header.stamp = ros::Time::now();
    attitude.header.frame_id = "map";
    Eigen::Matrix3d R = odometry.orientation.toRotationMatrix();
    Eigen::Vector3d b1_des;
    double yaw = point.getYaw();
    b1_des << cos(yaw), sin(yaw), 0;

    Eigen::Vector3d b3_des;
    b3_des = -cmd_acc / cmd_acc.norm();

    Eigen::Vector3d b2_des;
    b2_des = b3_des.cross(b1_des);
    b2_des.normalize();

    Eigen::Matrix3d R_des;
    R_des.col(0) = b2_des.cross(b3_des);
    R_des.col(1) = b2_des;
    R_des.col(2) = b3_des;
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
    Eigen::Vector3d angle_error;
    Eigen::Quaterniond q_des(R_des);
    angle_error = -0.5* matrix_hat_inv(R_des.transpose() * R - R.transpose() * R_des);
    attitude.type_mask = 7; // ignore orientation
    double total_th = (mass * 9.8) / pth;
    attitude.thrust = (-mass * cmd_acc).dot(odometry.orientation.toRotationMatrix().col(2)) / total_th;
    attitude.thrust = (attitude.thrust * 49999) + 10001;
    attitude.body_rate.x = angx * angle_error.x(); //TODO: 调整比例系数，改为由launch文件传参
    attitude.body_rate.y = angy * angle_error.y();
    attitude.body_rate.z = angz * angle_error.z();
    attitude.orientation.w = q_des.w();
    attitude.orientation.x = q_des.x();
    attitude.orientation.y = q_des.y();
    attitude.orientation.z = q_des.z();
    attitude_pub_.publish(attitude);
}

void field_esp_base::msg_mav_trajectory_from_state(MathLib::Vector state, trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg)
{
    trajectory_msg.header.stamp = ros::Time::now();
    Eigen::Matrix3d desired_ori;
    Eigen::Vector3d desired_position(0., 0., 1.5);
    Eigen::Vector3d desired_velocity(0., 0., 0.);
    Eigen::Vector3d desired_acc(0., 0., 0.);
    desired_position << state[0], state[1], state[2];
    desired_velocity << state[3], state[4], state[5];
    // point.velocity_W = desired_velocity;
    Eigen::Quaterniond doo(desired_ori.transpose());
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(point, &trajectory_msg);
}

void field_esp_base::main_loop()
{
    int rate = 200;
    ros::Rate r(rate);
    // MathLib::Vector x_0;
    // x_0.Resize(15);
    // x_0[0] = 0.;
    // x_0[1] = 0.;
    // x_0[2] = 1.5;
    // x_0[3] = 0.;
    // x_0[4] = 0.;
    // x_0[5] = 0.;
    // msg_mav_trajectory_from_state(x_0, trajectory_msg);
    //     // std::cout << BOLDCYAN << "-3" << std::endl;

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0.;
    // pose.pose.position.y = 0.;
    // pose.pose.position.z = 1.5;
    // pose.pose.orientation.x = 0.;
    // pose.pose.orientation.y = 0.;
    // pose.pose.orientation.z = 0.;
    // pose.pose.orientation.w = 1.;
    // point.position_W.x() = INIT_X;
    // point.position_W.y() = INIT_Y;
    // point.position_W.z() = INIT_Z;
    while (ros::ok())
    {
        // if (solved)
        // {
        //     std::cout << BOLDCYAN << "solved" << RESET << std::endl;
        // }
        // // std::cout << BOLDCYAN << "-2" << std::endl;
        publish_cmd();
        // refresh_once();
        ros::spinOnce();
        r.sleep();
    }
    return;
    // MathLib::Vector x;
    // x.Resize(15);
    // x[0] = x_0[0];
    // x[1] = x_0[1];
    // x[2] = x_0[2];
    // x[3] = 0.;
    // x[4] = 0.;
    // x[5] = 0.;

    // pose.pose.position.x = 0.;
    // pose.pose.position.y = 0.;
    // pose.pose.position.z = 2.0;
    // while (ros::ok())
    // {
    //     counter++;
    //     controller_run_once();
    //     set_start(start_postion[0], start_postion[1], start_postion[2], 0., 0., 0.);
    //     double factor = 1.0;
    //     double dis, vel;
    //     double vel_ball = sqrt(pow(ball_velocity[0], 2) + pow(ball_velocity[1], 2) + pow(ball_velocity[2], 2));
    //     ball_vel_th[0] = ball_velocity[0] / vel_ball * 5.0;
    //     ball_vel_th[1] = ball_velocity[1] / vel_ball * 5.0;
    //     ball_vel_th[2] = ball_velocity[2] / vel_ball * 5.0;
    //     current_dist = sqrt(pow(current_pose.pose.position.x - start_postion[0], 2) + pow(current_pose.pose.position.y - start_postion[1], 2) + pow(current_pose.pose.position.z - start_postion[2], 2));
    //     std::cout << BOLDCYAN << "round: " << counter << "\t";
    //     Eigen::Vector3d vel_cmd = get_vel_cmd();
    //     Eigen::Vector3d pos_cmd = get_pos_cmd();
    //     vel = abs(vel_cmd[0]) + abs(vel_cmd[1]) + abs(vel_cmd[2]);
    //     if (counter <= counter_th)
    //     {
    //         double vx_th = 1.5;
    //         double vy_th = 1.5;
    //         double vz_th = 1.5;
    //         double vx = max(min(vel_cmd[0], vx_th), -vx_th);
    //         double vy = max(min(vel_cmd[1], vy_th), -vy_th);
    //         double vz = max(min(vel_cmd[2], vz_th), -vz_th);
    //         if (use_position)
    //         {
    //             x[0] = pos_cmd[0];
    //             x[1] = pos_cmd[1];
    //             x[2] = pos_cmd[2];
    //         }
    //         else
    //         {
    //             x[0] += vx * factor / rate;
    //             x[1] += vy * factor / rate;
    //             x[2] += vz * factor / rate;
    //         }
    //         std::cout << BOLDRED << pos_cmd[0] << "\t" << pos_cmd[1] << "\t" << pos_cmd[2] << "\t";
    //         std::cout << BOLDBLUE << x[0] << "\t" << x[1] << "\t" << x[2] << "\t" << std::endl;
    //         x[3] = factor * vx;
    //         x[4] = factor * vy;
    //         x[5] = factor * vz;
    //         pose.pose.position.x = x[0];
    //         pose.pose.position.y = x[1];
    //         pose.pose.position.z = x[2];
    //         trajectory_msg.points.clear();
    //         msg_mav_trajectory_from_state(x, trajectory_msg);
    //         publish_cmd();
    //         if (catched)
    //         {
    //             break;
    //         }
    //     }
    //     else
    //     {
    //         break;
    //     }
    //     publish_cmd();
    //     refresh_once();
    //     r.sleep();
    //     ros::spinOnce();
    // }
    // std::cout << "final: " << x[0] << "\t" << x[1] << "\t" << x[2] << "\t";
    // std::cout << current_pose.pose.position.x << "\t" << current_pose.pose.position.y << "\t" << current_pose.pose.position.z << "\n";
    // pose.pose.position.x = current_pose.pose.position.x;
    // pose.pose.position.y = current_pose.pose.position.y;
    // pose.pose.position.z = current_pose.pose.position.z;
    // Eigen::Vector3d position(pose.pose.position.x,
    //                         pose.pose.position.y,
    //                         pose.pose.position.z);
    // while (ros::ok())
    // {
    //     trajectory_msg.points.clear();
    //     x[0] = position[0];
    //     x[1] = position[1];
    //     x[2] = position[2];
    //     x[3] = 0.;
    //     x[4] = 0.;
    //     x[5] = 0.;
    //     x[6] = 0.;
    //     x[7] = 0.;
    //     x[8] = 0.;
    //     x[9] = 0.;
    //     x[10] = 0.;
    //     x[11] = 0.;
    //     msg_mav_trajectory_from_state(x, trajectory_msg);
    //     publish_cmd();
    //     ROS_INFO_ONCE("steadying");
    //     refresh_once();
    //     r.sleep();
    //     ros::spinOnce();
    // }
    // std::cout << BOLDRED << "current log num: " << sum << RESET << std::endl;
}

void field_esp_base::plan_loop()
{
    std::cout << "l 499 of ac_lee_position_controller_esp.cpp " << std::endl;
}

void field_esp_base::start_process()
{
    std::thread main_thread(&field_esp_base::main_loop, this);
    std::thread process_thread(&field_esp_base::plan_loop, this);
    main_thread.join();
    process_thread.join();
}

void field_esp_base::run()
{
    start_process();
    ros::spin();
}

void field_esp_base::load_param()
{
    // load params from yaml
    uam_d = 0.08;
    uam_l = 0.30;
    uam_r = 0.35;
    uam_r_thick = 0.05;
    
    uam_r_n = 0.15;
    uam_r_ring = 0.03;
    // predictor.param.set_param(uam_d, uam_l, uam_r, uam_r_thick, uam_r_n, uam_r_ring);
    // predictor.param.print();
}
void field_esp_base::initialize_params(const ros::NodeHandle &private_nh_)
{
}
void field_esp_base::refresh_once()
{
    // std::cout << CYAN << "-1" << std::endl;

    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose = current_odom.pose.pose;
    path.poses.push_back(this_pose_stamped);
    // std::cout << CYAN << "0" << std::endl;
    path_pub.publish(path);
    Eigen::Vector3d ee_pose;
    Eigen::Vector3d position;
    position[0] = current_odom.pose.pose.position.x;
    position[1] = current_odom.pose.pose.position.y;
    position[2] = current_odom.pose.pose.position.z;
    check_safty(position);
    q_now.setW(current_odom.pose.pose.orientation.w);
    q_now.setX(current_odom.pose.pose.orientation.x);
    q_now.setY(current_odom.pose.pose.orientation.y);
    q_now.setZ(current_odom.pose.pose.orientation.z);
    m0.setRotation(q_now);
}

void field_esp_base::controller_run_once()
{
    std::cout << "l 15 of base.cpp " << std::endl;
}

Eigen::Vector3d field_esp_base::get_vel_cmd()
{
    std::cout << "l 590 of field_esp_base.cpp " << std::endl;
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d field_esp_base::get_pos_cmd()
{
    std::cout << "l 596 of field_esp_base.cpp " << std::endl;
    return Eigen::Vector3d::Zero();
}

void field_esp_base::set_target(double goal_x, double goal_y, double goal_z, double goal_vx, double goal_vy, double goal_vz, double t_end = 1.0)
{
}

void field_esp_base::set_start(double s_x, double s_y, double s_z, double s_vx, double s_vy, double s_vz)
{
}
void field_esp_base::print_basic_params()
{
    std::cout << BOLDGREEN << std::boolalpha << "\nprinting basic params:" << std::endl;
    std::cout << "\t"
              << "use_position:\t" << use_position << std::endl;
    std::cout << "\t"
              << "enable_log:\t" << enable_log << std::endl;
    std::cout << "\t"
              << "counter_th:\t" << counter_th << std::endl;
    std::cout << "\t"
              << "rate:\t\t" << rate << std::endl;
    std::cout << "\t"
              << "use_gazebo:\t" << use_gazebo << std::endl;
    std::cout << std::noboolalpha << RESET;
    std::cout << "ppx:\t" << ppx << std::endl;
    std::cout << "ppy:\t" << ppy << std::endl;
    std::cout << "ppz:\t" << ppz << std::endl;
    std::cout << "pvx:\t" << pvx << std::endl;
    std::cout << "pvy:\t" << pvy << std::endl;
    std::cout << "pvz:\t" << pvz << std::endl;
}

void field_esp_base::check_safty(Eigen::Vector3d position)
{
    bool inbox = true;
    inbox = inbox && (X_MIN <= position[0] && position[0] <= X_MAX);
    inbox = inbox && (Y_MIN <= position[1] && position[1] <= Y_MAX);
    inbox = inbox && (Z_MIN <= position[2] && position[2] <= Z_MAX);
    if (inbox == false)
    {
        land();
        std::cout << BOLDCYAN << "DANGEROUS BEHAVIOR, LANDING" << RESET << std::endl;
        std::cout << CYAN << "X_RANGE: [" << X_MIN << ", " << X_MAX << "], current: " << position[0] << RESET << std::endl;
        std::cout << CYAN << "Y_RANGE: [" << Y_MIN << ", " << Y_MAX << "], current: " << position[1] << RESET << std::endl;
        std::cout << CYAN << "Z_RANGE: [" << Z_MIN << ", " << Z_MAX << "], current: " << position[2] << RESET << std::endl;
    }
}

void field_esp_base::land()
{
    std_msgs::Bool land;
}
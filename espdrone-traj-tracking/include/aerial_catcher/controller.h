#ifndef IMP_CONTROLLER_H
#define IMP_CONTROLLER_H

// Ros
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.hpp>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h> //robotname/groudtruth
#include <aerial_catcher/BaseState.h> //robotname/pose
#include <aerial_catcher/pid.h>
#include <aerial_catcher/set_pids.h>
#include <gazebo_msgs/ContactsState.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

//TODO contact sensor
//#include <gazebo/sensors/SensorManager.hh>
//#include <gazebo/sensors/ContactSensor.hh>

#include <realtime_tools/realtime_publisher.h>

namespace ac_impedance_controller
{

class ImpController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
{
public:
    ImpController();

    ~ImpController();

    /**
         * @brief Initializes sample ImpController
         * @param hardware_interface::RobotHW* robot hardware interface
         * @param ros::NodeHandle& Root node handle
         * @param ros::NodeHandle& Supervisor node handle
         */
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    /**
         * @brief Starts the sample ImpController when ImpController manager request it
         * @param const ros::Time& time Time
         */
    void starting(const ros::Time& time);

    /**
         * @brief Updates the sample ImpController according to the control
         * frequency (task frequency)
         * @param const ros::Time& time Time
         * @param const ros::Duration& Period
         */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
         * @brief Stops the sample ImpController when ImpController manager request it
         * @param const ros::time& Time
         */
    void stopping(const ros::Time& time);

private:

    void commandCallback(const sensor_msgs::JointState &msg);

    bool setPidsCallback(aerial_catcher::set_pids::Request& req,
                         aerial_catcher::set_pids::Response& res);
    void baseGroundTruthCB(const nav_msgs::OdometryConstPtr &msg);
    void fextCallback(const std_msgs::Float64ConstPtr &msg);

    ros::Subscriber sub_;
    ros::Subscriber fext_sub;
    ros::Subscriber gt_sub_;
    ros::ServiceServer set_pids_srv_;
    ros::ServiceServer get_map_srv_;

    std::shared_ptr<realtime_tools::RealtimePublisher<aerial_catcher::BaseState>> pose_pub_rt_;
    std::shared_ptr<realtime_tools::RealtimePublisher<gazebo_msgs::ContactsState>> contact_state_pub_rt_;

    /** @brief Number of joints */
    unsigned int num_joints_;
    /** @brief Joint names */
    std::vector<std::string> joint_names_;
    /** @brief Joint states for reading positions, velocities and efforts and writing effort commands */
    std::vector<hardware_interface::JointHandle> joint_states_;
    /** @brief Desired joint efforts */
    Eigen::VectorXd des_joint_efforts_;
    /** @brief Desired joint positions */
    Eigen::VectorXd des_joint_positions_;
    /** @brief Desired joint velocities */
    Eigen::VectorXd des_joint_velocities_;
    /** @brief Actual P value for the joints PID ImpController */
    std::vector<double> joint_p_gain_;
    /** @brief Actual I value for the joints PID ImpController */
    std::vector<double> joint_i_gain_;
    /** @brief Actual D value for the joints PID ImpController */
    std::vector<double> joint_d_gain_;
    /** @brief Desired joint efforts computed by the PIDs */
    Eigen::VectorXd des_joint_efforts_pids_;
    tf::Quaternion q_base;
    tf::Vector3 base_pos_w;
    geometry_msgs::Twist base_twist_w;
    //TODO contact state
    //std::vector<std::shared_ptr<gazebo::sensors::ContactSensor> > foot_sensors_;
    std::vector<std::vector<double> > force_;
    std::vector<std::vector<double> > torque_;
    std::vector<std::vector<double> > normal_;
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::EffortJointInterface effort_joint_interface;
    std::vector<double> joint_position_state;
    std::vector<double> joint_velocity_state;
    std::vector<double> joint_effort_state;
    std::vector<double> joint_effort_command;
    double B_q[1][1]={{0.00552}};
    double C_q[1][1]={{0.0}};
    double F_ext=0;

    ros::NodeHandle * root_nh_;
    bool verbose = false;
    //dls::perception::GridMapTerrainROS grid_map_terrain_;

};


PLUGINLIB_EXPORT_CLASS(ac_impedance_controller::ImpController, controller_interface::ControllerBase);

} //@namespace ac_impedance_controller

#endif //IMP_CONTROLLER_H

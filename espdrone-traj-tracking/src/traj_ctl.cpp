#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ros/callback_queue.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "ros/time.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include <nav_msgs/Odometry.h>
#include "ros/subscriber.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"

using namespace std;

#define SIM 0

enum MISSION_STATE
{
    INIT,
    POSITION,  // TAKEOFF
    LAND
};

MISSION_STATE mission_state;

mavros_msgs::State px4_state, px4_state_prev;
nav_msgs::Odometry esp_odom;

#define DEAD_ZONE 0.25
#define MAX_MANUAL_VEL 1.0
#define RC_REVERSE_PITCH 0
#define RC_REVERSE_ROLL 0
#define RC_REVERSE_THROTTLE 0

double last_set_hover_pose_time;

ros::Publisher     target_pose_pub, odom_pub, attitude_and_pose_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

//ros::Publisher traj_repub;

//  W: World;V: View; B: Body;
Eigen::Affine3f     T_W_Bt, T_V_Bt_set, T_W_V, T_B_C;
geometry_msgs::Pose T_W_B_set, pose_prev;
geometry_msgs::PoseStamped prev_header, attitude_and_pose;

bool take_off = false;
bool send_tra = false;
int set_init = 0;
double tra_start_x;  //0.894
double tra_start_y; //-0.393
double tra_start_z;  //1.300
double traj_start_qx;  
double traj_start_qy;  
double traj_start_qz;  
double traj_start_qw;

void pose_set_callback(const geometry_msgs::PoseStampedConstPtr &pose_set_msg) 
{
    double z_take_off = 0.800;
    if (set_init == 1 && fabs(z_take_off - pose_set_msg->pose.position.z) < 0.100) {
        // T_W_B_set.position.x = tra_start_x;
        // T_W_B_set.position.y = tra_start_y;
        // T_W_B_set.position.z = tra_start_z;
        // T_W_B_set.orientation.x = traj_start_qx;
        // T_W_B_set.orientation.y = traj_start_qy;
        // T_W_B_set.orientation.z = traj_start_qz;
        // T_W_B_set.orientation.w = traj_start_qw;

        // take_off = true;
        // set_init++;
    }

    if(set_init == 2 
    && fabs(tra_start_x - pose_set_msg->pose.position.x) < 0.150
    && fabs(tra_start_y - pose_set_msg->pose.position.y) < 0.150
    && fabs(tra_start_z - pose_set_msg->pose.position.z) < 0.150){
        ROS_INFO("You can now send the trajectory message!");

        send_tra = true;
        set_init++;
        }

    if (set_init == 0 && mission_state != LAND && take_off == false) {
        T_W_B_set.position.x = pose_set_msg->pose.position.x;
        T_W_B_set.position.y = pose_set_msg->pose.position.y;
        T_W_B_set.position.z = z_take_off;
        T_W_B_set.orientation = pose_set_msg->pose.orientation;

        set_init++;
    }

    if (mission_state != INIT)
    {
        double now               = ros::Time::now().toSec();
        double delta_t           = now - last_set_hover_pose_time;
        last_set_hover_pose_time = now;
        if (mission_state == LAND)
        {
            T_W_B_set.position.z -= 0.3 * delta_t;
        }

        if (T_W_B_set.position.z < -0.3)
            T_W_B_set.position.z = -0.3;
        else if (T_W_B_set.position.z > 1.8)
            T_W_B_set.position.z = 1.8;
    }

    esp_odom.pose.pose.position = pose_set_msg->pose.position;
    esp_odom.pose.pose.orientation = pose_set_msg->pose.orientation;
    // esp_odom.twist.twist.linear.x = (pose_set_msg->pose.position.x - pose_prev.position.x) / (pose_set_msg->header.stamp.toSec() - prev_header.header.stamp.toSec());
    // esp_odom.twist.twist.linear.y = (pose_set_msg->pose.position.y - pose_prev.position.y) / (pose_set_msg->header.stamp.toSec() - prev_header.header.stamp.toSec());
    // esp_odom.twist.twist.linear.z = (pose_set_msg->pose.position.z - pose_prev.position.z) / (pose_set_msg->header.stamp.toSec() - prev_header.header.stamp.toSec());
    esp_odom.twist.twist.linear.x = pose_set_msg->pose.position.x - pose_prev.position.x;
    esp_odom.twist.twist.linear.y = pose_set_msg->pose.position.y - pose_prev.position.y;
    esp_odom.twist.twist.linear.z = pose_set_msg->pose.position.z - pose_prev.position.z;

    pose_prev = pose_set_msg->pose;
    prev_header.header.stamp = pose_set_msg->header.stamp;

    nav_msgs::Odometry odom;
    odom.header.stamp    = ros::Time::now();
    odom.header.frame_id = "map";
    odom.pose            = esp_odom.pose;
    odom.twist           = esp_odom.twist;
    odom_pub.publish(odom);
}

std::deque<ros::Duration> command_waiting_times_;
ros::Timer command_timer_;
double tra_x[10000];
double tra_y[10000];
double tra_z[10000];
double tra_ox[10000];
double tra_oy[10000];
double tra_oz[10000];
double tra_ow[10000];
int j = -1;

void trajectory_set_callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg) 
{
  //traj_repub.publish(msg);
  //ROS_INFO("got traj msg!!!");
  ROS_INFO_ONCE("received trajectory message");
  if (send_tra && mission_state == POSITION) {
    const size_t n_commands = msg->points.size();
    if (n_commands < 1) {
      ROS_WARN_STREAM("Got trajectory message, but message has no points.");
      return;
    }

    command_waiting_times_.clear();
    command_timer_.stop();

    for (size_t i = 0; i < n_commands - 1; i++) {
      const trajectory_msgs::MultiDOFJointTrajectoryPoint &current_reference =
          msg->points[i];
      const trajectory_msgs::MultiDOFJointTrajectoryPoint &reference_after =
          msg->points[i + 1];
      tra_x[i] = current_reference.transforms[0].translation.x;
      tra_y[i] = current_reference.transforms[0].translation.y;
      tra_z[i] = current_reference.transforms[0].translation.z;
      tra_ox[i] = current_reference.transforms[0].rotation.x;
      tra_oy[i] = current_reference.transforms[0].rotation.y;
      tra_oz[i] = current_reference.transforms[0].rotation.z;
      tra_ow[i] = current_reference.transforms[0].rotation.w;
      command_waiting_times_.push_back(reference_after.time_from_start -
                                       current_reference.time_from_start);
    }

    if (n_commands > 1) {
      command_timer_.setPeriod(command_waiting_times_.front());
      command_waiting_times_.pop_front();
      command_timer_.start();
    }
    }
}

void attitude_set_callback(const mavros_msgs::AttitudeTargetConstPtr &msg)
{
    attitude_and_pose.pose.position.x = pose_prev.position.x;
    attitude_and_pose.pose.position.y = pose_prev.position.y;
    attitude_and_pose.pose.position.z = pose_prev.position.z;
    attitude_and_pose.pose.orientation.w = msg->thrust;
    
}

void time_command_callback(const ros::TimerEvent &e) 
{
    if(send_tra && mission_state == POSITION)
    {
      j++;
      cout << "heading to point " << j << endl;
      T_W_B_set.position.x = tra_x[j];
      T_W_B_set.position.y = tra_y[j];
      T_W_B_set.position.z = tra_z[j];
      T_W_B_set.orientation.x = tra_ox[j];
      T_W_B_set.orientation.y = tra_oy[j];
      T_W_B_set.orientation.z = tra_oz[j];
      T_W_B_set.orientation.w = tra_ow[j];
      command_timer_.stop();

      if(command_waiting_times_.empty()){
        ROS_INFO("The trajectory tracking is over and all points have been passed");
        mission_state = LAND;
      }
      if (!command_waiting_times_.empty()) {
        command_timer_.setPeriod(command_waiting_times_.front());
        command_waiting_times_.pop_front();
        command_timer_.start();
      }
    }
}

#define BAUDRATE 57600
#define ID 1
#define INIT_RAD -1.57233

int  goal_position;
void pitch_set_callback(const std_msgs::Float32ConstPtr &pitch_set_msg)
{
    float pitch_set = pitch_set_msg->data;
    float radian    = INIT_RAD + pitch_set;
}

void rc_callback(const mavros_msgs::RCInConstPtr &rc_msg)
{
    double rc_ch[4];
    for (int i = 0; i < 4; i++)
    {
        // 归一化遥控器输入
        rc_ch[i] = ((double)rc_msg->channels[i] - 1500.0) / 500.0;
        if (rc_ch[i] > DEAD_ZONE)
            rc_ch[i] = (rc_ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (rc_ch[i] < -DEAD_ZONE)
            rc_ch[i] = (rc_ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            rc_ch[i] = 0.0;
    }

    if (rc_msg->channels[4] < 1250)
    {
        if (px4_state.mode != "STABILIZED")
        {
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "STABILIZED";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Switch to STABILIZED!");
                px4_state_prev      = px4_state;
                px4_state_prev.mode = "STABILIZED";
            }
            else
            {
                ROS_WARN("Failed to enter STABILIZED!");
                return;
            }
        }
        mission_state = INIT;
        cout << "px4 state mode is " << px4_state.mode << endl;
    }
    else if (rc_msg->channels[4] > 1250 && rc_msg->channels[4] < 1750)  // heading to POSITION
    {
        if (mission_state == INIT)
        {
            if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[3] == 0.0)
            {
                mission_state            = POSITION;
                ROS_INFO("Switch to POSITION succeed!");
            }
            else
            {
                ROS_WARN("Switch to POSITION failed! Rockers are not in reset middle!");
                return;
            }
        }
    }

    if (!SIM)
    {
        if (rc_msg->channels[5] > 1750)
        {
            if (mission_state == POSITION)
            {
                if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[3] == 0.0 && !px4_state.armed)
                {
                    if (px4_state.mode != "OFFBOARD")
                    {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                        {
                            ROS_INFO("Offboard enabled");
                            px4_state_prev      = px4_state;
                            px4_state_prev.mode = "OFFBOARD";
                        }
                        else
                        {
                            ROS_WARN("Failed to enter OFFBOARD!");
                            return;
                        }
                    }
                    else if (px4_state.mode == "OFFBOARD")
                    {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;

                        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle armed");
                        }
                        else
                        {
                            ROS_ERROR("Failed to armed");
                            return;
                       }
                    }
                }
                else if (!px4_state.armed)
                {
                    ROS_WARN("Arm denied! Rockers are not in reset middle!");
                    return;
                }
            }
        } else if (rc_msg->channels[5] > 1250 && rc_msg->channels[5] < 1750) {
          if (px4_state_prev.mode == "OFFBOARD") {
            mission_state = LAND;
          }
        } else if (rc_msg->channels[5] < 1250) {
          if (px4_state.armed) {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = false;

            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
              ROS_INFO("Vehicle disarmed");
            } else {
              ROS_ERROR("Failed to disarmed");
              return;
            }
            mission_state = INIT;
            ROS_INFO("Swith to INIT state!");
          }
        }
    }

    if (mission_state != INIT)
    {
        double now               = ros::Time::now().toSec();
        double delta_t           = now - last_set_hover_pose_time;
        last_set_hover_pose_time = now;
        if (mission_state == LAND)
        {
            T_W_B_set.position.z -= 0.3 * delta_t;
        }

        if (T_W_B_set.position.z < -0.3)
            T_W_B_set.position.z = -0.3;
        else if (T_W_B_set.position.z > 1.8)
            T_W_B_set.position.z = 1.8;
    }
}
void px4_state_callback(const mavros_msgs::StateConstPtr &state_msg)
{
    px4_state = *state_msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nbv_ctl");
    ros::NodeHandle nh("~");
    ros::Rate       rate(200);

    nh.param("init_x", tra_start_x, 0.894);
    nh.param("init_y", tra_start_y, -0.393);
    nh.param("init_z", tra_start_z, 1.300);
    nh.param("init_qx", traj_start_qx, 0.00);
    nh.param("init_qy", traj_start_qy, 0.00);
    nh.param("init_qz", traj_start_qz, 0.00);
    nh.param("init_qw", traj_start_qw, 1.00);

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, px4_state_callback,
                                         ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>(
        "/mavros/rc/in", 10, rc_callback, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber pose_set_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pose_set_callback);

    ros::Subscriber trajectory_sub = 
        nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/hummingbird/whole_command_trajectory", 1, trajectory_set_callback);
    
    ros::Subscriber attitude_sub = 
        nh.subscribe<mavros_msgs::AttitudeTarget>("/aerial_catcher/esp/attitude", 1, attitude_set_callback);

    attitude_and_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/pid_param", 100);

    target_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/aerial_catcher/esp/target_pose", 100);

    odom_pub =
        nh.advertise<nav_msgs::Odometry>("/mavros/local_position/odom",100);

    arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    command_timer_ = nh.createTimer(ros::Duration(0), time_command_callback, true, false);

    mission_state = POSITION;

    const char *log;

    while (ros::ok())
    {
        if (mission_state != INIT)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose            = T_W_B_set;
            target_pose_pub.publish(pose);

            geometry_msgs::PoseStamped pose_2;
            pose_2.header.stamp    = ros::Time::now();
            pose_2.header.frame_id = "map";
            pose_2.pose            = attitude_and_pose.pose;
            attitude_and_pose_pub.publish(pose_2);

            // nav_msgs::Odometry odom;
            // odom.header.stamp    = ros::Time::now();
            // odom.header.frame_id = "map";
            // odom.pose            = esp_odom.pose;
            // odom.twist           = esp_odom.twist;
            // odom_pub.publish(odom);
        }
        // cout << "traj_start_x is " << tra_start_x << endl;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
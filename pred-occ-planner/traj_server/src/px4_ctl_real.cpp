#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ros/callback_queue.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/PositionTarget.h"
#include "quadrotor_msgs/PositionCommand.h"
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

#define DEAD_ZONE 0.25
#define MAX_MANUAL_VEL 1.0
#define RC_REVERSE_PITCH 0
#define RC_REVERSE_ROLL 0
#define RC_REVERSE_THROTTLE 0

double last_set_hover_pose_time;

ros::Publisher     target_pose_pub;
ros::Publisher     mav_trajectory_pub;
ros::Publisher     trigger_pub;
ros::Publisher     rawpose_pub;
ros::Publisher     click_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

//  W: World;V: View; B: Body;
geometry_msgs::Pose init_set;
mavros_msgs::PositionTarget traj_set;

bool take_off = false;
bool OCC_PLAN = false;
bool rcv_cmd = false;
int set_count = 0;
int click_count = 0;
double traj_start_x;  //0.894
double traj_start_y; //-0.393
double traj_start_z;  //1.300
double traj_start_qx;  
double traj_start_qy;  
double traj_start_qz;  
double traj_start_qw;  

void pose_set_callback(const geometry_msgs::PoseStampedConstPtr &pose_set_msg) {
  double z_take_off = 0.800;
  if (set_count == 1 && fabs(z_take_off - pose_set_msg->pose.position.z) < 0.100) {
    init_set.position.x = traj_start_x;
    init_set.position.y = traj_start_y;
    init_set.position.z = traj_start_z;
    init_set.orientation.w = traj_start_qw;
    init_set.orientation.x = traj_start_qx;
    init_set.orientation.y = traj_start_qy;
    init_set.orientation.z = traj_start_qz;

    take_off = true;
    set_count++;
  }

if(set_count == 2 
&& fabs(traj_start_x - pose_set_msg->pose.position.x) < 0.150
&& fabs(traj_start_y - pose_set_msg->pose.position.y) < 0.150
&& fabs(traj_start_z - pose_set_msg->pose.position.z) < 0.150){
    ROS_INFO("About to start planning the trajectory");

    OCC_PLAN = true;
    set_count++;
}

  if (set_count == 0 && mission_state != LAND && take_off == false) {
    init_set.position.x = pose_set_msg->pose.position.x;
    init_set.position.y = pose_set_msg->pose.position.y;
    init_set.position.z = z_take_off;
    init_set.orientation = pose_set_msg->pose.orientation;

    set_count++;
  }

  if(mission_state == POSITION && OCC_PLAN == true && rcv_cmd == true)
  {
    init_set = pose_set_msg->pose;
  }
}

void trajectory_set_callback(const quadrotor_msgs::PositionCommandConstPtr &cmd) 
{
    rcv_cmd = true;
    traj_set.position.x = cmd->position.x;
    traj_set.position.y = cmd->position.y;
    traj_set.position.z = cmd->position.z;
    traj_set.velocity.x = cmd->velocity.x;
    traj_set.velocity.y = cmd->velocity.y;
    traj_set.velocity.z = cmd->velocity.z;
    traj_set.acceleration_or_force.x = cmd->acceleration.x;
    traj_set.acceleration_or_force.y = cmd->acceleration.y;
    traj_set.acceleration_or_force.z = cmd->acceleration.z;
    traj_set.yaw = cmd->yaw;
    traj_set.yaw_rate = cmd->yaw_dot;
}


#define BAUDRATE 57600
#define ID 1
#define INIT_RAD -1.57233

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
            OCC_PLAN = false;
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
            init_set.position.z -= 0.3 * delta_t;
        }

        if (init_set.position.z < -0.3)
            init_set.position.z = -0.3;
        else if (init_set.position.z > 1.8)
            init_set.position.z = 1.8;
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
    ros::Rate       rate(30);

    nh.param("init_x", traj_start_x, 0.894);
    nh.param("init_y", traj_start_y, -0.393);
    nh.param("init_z", traj_start_z, 1.300);
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
        nh.subscribe<quadrotor_msgs::PositionCommand>("/uav0/controller/pos_cmd", 1, trajectory_set_callback); 

    rawpose_pub= 
        nh.advertise<geometry_msgs::PoseStamped>("/raw_pose_rviz", 100);

    target_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    
    mav_trajectory_pub =
        nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    
    trigger_pub = 
        nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

    arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mission_state = INIT;

    const char *log;

    while (ros::ok())
    {
        //OCC_PLAN = true;//debug
        if (OCC_PLAN == true)
        {
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id    = "map";
            ps.header.stamp       = ros::Time::now();
            ps.pose.orientation.x = 0;
            ps.pose.orientation.y = 0;
            ps.pose.orientation.z = 0;
            ps.pose.orientation.w = 1;
            ps.pose.position.x    = -2.22;
            ps.pose.position.y    = 0;
            ps.pose.position.z    = 1;
            trigger_pub.publish(ps);
        }
        if (mission_state != INIT)
        {
            if(mission_state == POSITION && OCC_PLAN == true && rcv_cmd == true)
            {
                mavros_msgs::PositionTarget command;
                command.header.stamp    = ros::Time::now();
                command.header.frame_id = "map";
                command.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                command.position = traj_set.position;
                command.velocity = traj_set.velocity;
                command.acceleration_or_force = traj_set.acceleration_or_force;
                command.yaw = 0.;
                command.yaw_rate = 0.;
                mav_trajectory_pub.publish(command);
                geometry_msgs::PoseStamped rawpose;
                rawpose.header.stamp    = ros::Time::now();
                rawpose.header.frame_id = "map";
                rawpose.pose.position = traj_set.position;
                rawpose.pose.orientation = init_set.orientation;
                rawpose_pub.publish(rawpose);
            }
            else
            {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp    = ros::Time::now();
                pose.header.frame_id = "map";
                pose.pose            = init_set;
                target_pose_pub.publish(pose);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
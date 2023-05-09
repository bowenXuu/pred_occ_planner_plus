/**
 * @file fake_joystick.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief copied from turtlebot_teleop_key.cpp
 * @version 1.0
 * @date 2022-07-11
 *
 * @copyright Copyright (c) 2022
 *
 */

/* !TODO(@siyuan): It's hard to work with poscmd2odom because it requires position commands at
                   frequency of 100Hz. This node only provides asynchronous velocity commands.
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>

#include "quadrotor_msgs/PositionCommand.h"
#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#else
#include <windows.h>
#endif

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT  0x44
#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_B     0x62
#define KEYCODE_C     0x63
#define KEYCODE_D     0x64
#define KEYCODE_E     0x65
#define KEYCODE_F     0x66
#define KEYCODE_G     0x67
#define KEYCODE_Q     0x71
#define KEYCODE_R     0x72
#define KEYCODE_T     0x74
#define KEYCODE_V     0x76

const double MAX_LIN_VEL = 1.0;
const int    LOOP_RATE   = 10;
const double LOOP_TIME   = 1 / static_cast<double>(LOOP_RATE);

class KeyboardReader {
 public:
  KeyboardReader()
#ifndef _WIN32
      : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char *c) {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0) {
      throw std::runtime_error("read failed");
    }
#else
    for (;;) {
      HANDLE       handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD        events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if (events > 0) {
        ReadConsoleInput(handle, &buffer, 1, &events);
        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT) {
          *c = KEYCODE_LEFT;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP) {
          *c = KEYCODE_UP;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT) {
          *c = KEYCODE_RIGHT;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN) {
          *c = KEYCODE_DOWN;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42) {
          *c = KEYCODE_B;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43) {
          *c = KEYCODE_C;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44) {
          *c = KEYCODE_D;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45) {
          *c = KEYCODE_E;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46) {
          *c = KEYCODE_F;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47) {
          *c = KEYCODE_G;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51) {
          *c = KEYCODE_Q;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52) {
          *c = KEYCODE_R;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54) {
          *c = KEYCODE_T;
          return;
        } else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56) {
          *c = KEYCODE_V;
          return;
        }
      }
    }
#endif
  }
  void shutdown() {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }

 private:
#ifndef _WIN32
  int            kfd;
  struct termios cooked;
#endif
};

KeyboardReader input;

class TeleopDrone {
 public:
  TeleopDrone(ros::NodeHandle &nh);
  void keyLoop();
  void odomCallback(const nav_msgs::OdometryConstPtr &p_odom);

 private:
  ros::NodeHandle nh_;
  double          linear_, angular_, l_scale_, a_scale_;
  double          v_forward_, v_left_;
  ros::Publisher  cmd_pub_;
  ros::Subscriber odom_sub_;
  ros::Timer      timer_;

  double px_, py_, pz_;
  double qx_, qy_, qz_, qw_;
};

TeleopDrone::TeleopDrone(ros::NodeHandle &nh)
    : nh_(nh), linear_(0), angular_(0), l_scale_(2.0), a_scale_(2.0) {
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  cmd_pub_  = nh_.advertise<quadrotor_msgs::PositionCommand>("command", 1);
  odom_sub_ = nh_.subscribe("odom", 1, &TeleopDrone::odomCallback, this);

  timer_ = nh_.createTimer(ros::Duration(LOOP_TIME), [&](const ros::TimerEvent &) {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp    = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.position.x      = px_ + v_forward_ * LOOP_TIME;
    cmd.position.y      = py_ + v_left_ * LOOP_TIME;
    cmd.position.z      = pz_;
    cmd.velocity.x      = v_forward_;
    cmd.velocity.y      = v_left_;
    cmd.velocity.z      = 0;
    cmd.acceleration.x  = 0;
    cmd.acceleration.y  = 0;
    cmd.acceleration.z  = 0;
    cmd.yaw             = 0;
    cmd.yaw_dot         = 0;
    cmd_pub_.publish(cmd);
  });

  v_forward_ = v_left_ = 0.0;
}

void TeleopDrone::odomCallback(const nav_msgs::OdometryConstPtr &p_odom) {
  px_ = p_odom->pose.pose.position.x;
  py_ = p_odom->pose.pose.position.y;
  pz_ = p_odom->pose.pose.position.z;
  qx_ = p_odom->pose.pose.orientation.x;
  qy_ = p_odom->pose.pose.orientation.y;
  qz_ = p_odom->pose.pose.orientation.z;
  qw_ = p_odom->pose.pose.orientation.w;
}

void TeleopDrone::keyLoop() {
  char c;
  // bool dirty = false;

  ROS_INFO_ONCE("Reading from keyboard");
  ROS_INFO_ONCE("---------------------------");
  ROS_INFO_ONCE("Use arrow keys to move the drone. 'q' to quit.");
  ros::Time t0 = ros::Time::now();

  // for (;;) {
  // get the next event from the keyboard
  try {
    input.readOne(&c);
  } catch (const std::runtime_error &) {
    perror("read():");
    return;
  }

  linear_ = angular_ = 0;
  ROS_DEBUG("value: 0x%02X\n", c);

  switch (c) {
    case KEYCODE_LEFT:
      ROS_DEBUG("LEFT");
      v_left_ = 1.0;
      v_forward_ = 0.0;
      break;
    case KEYCODE_RIGHT:
      ROS_DEBUG("RIGHT");
      v_left_ = -1.0;
      v_forward_ = 0.0;
      break;
    case KEYCODE_UP:
      ROS_DEBUG("UP");
      v_forward_ = 1.0;
      v_left_ = 0.0;
      break;
    case KEYCODE_DOWN:
      ROS_DEBUG("DOWN");
      v_forward_ = -1.0;
      v_left_ = 0.0;
      break;
    case KEYCODE_Q:
      ROS_DEBUG("quit");
      return;
  }
  ros::Time t1 = ros::Time::now();
  ROS_INFO("Time: %f", (t1 - t0).toSec());
  // }
}

void quit(int sig) {
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "teleop_key");
  ros::NodeHandle nh("~");
  TeleopDrone     teleop_drone(nh);
  signal(SIGINT, quit);
  while (ros::ok()) {
    /* code */
    teleop_drone.keyLoop();
    ros::spinOnce();
  }

  quit(0);

  return (0);
}
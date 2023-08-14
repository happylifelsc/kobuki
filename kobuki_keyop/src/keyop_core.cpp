/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /kobuki_keyop/src/keyop_core.cpp
 *
 * @brief Creates a node for remote controlling parts of robot_core.
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <ecl/time.hpp>
#include <ecl/exceptions.hpp>
#include <std_srvs/Empty.h>
#include <kobuki_msgs/MotorPower.h>
#include "../include/keyop_core/keyop_core.hpp"
#include <iostream>
#include <math.h>

/*****************************************************************************
 ** global vars & constants for UGV control
 *****************************************************************************/
#define PI 3.1415926
#define MOVE_LINE_TRAJ 1 // 1 if moving a straight line, 0 for L traj

const float start_position[2] = {2.2, 6.5}; 
const float turn_point[2] = {2.5, 6.0}; 
const float start_orientation = -0.5*PI; // yaw, rad
// const float start_orientation = 0; // yaw, rad 
const float end_position[2] = {2.2, 1.0}; 
const float end_orientation = -0.5*PI; // yaw, rad
const float K_ang = 1.5;
const float K_lin = 0.5;
const float MT_VEL = 0.15; // move traj const linear speed, MUST BE LESS THAN: linear_vel_max(3.4)

float posi_cur[3] = {0, 0, 0}; //current position
float orien_cur = 0.0; //current orientation
double ang_err_first = 0;
double ang_err_second = 0;
double ang_err_mt = 0;

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace keyop_core
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

/**
 * @brief Default constructor, needs initialisation.
 */
KeyOpCore::KeyOpCore() : last_zero_vel_sent(true), // avoid zero-vel messages from the beginning
                         accept_incoming(true),
                         power_status(false),
                         wait_for_connection_(true),
                         cmd(new geometry_msgs::Twist()),
                         cmd_stamped(new geometry_msgs::TwistStamped()),
                         linear_vel_step(0.1),
                         linear_vel_max(3.4),   //note that wheel speed can saturate, |VL|<1.5 and |VR|<1.5, use this to determine the lin_vel and ang_vel
                         angular_vel_step(0.02),
                         angular_vel_max(1.2),
                         quit_requested(false),
                         key_file_descriptor(0)
{
  tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
}

KeyOpCore::~KeyOpCore()
{
  tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

/**
 * @brief Initialises the node.
 */
bool KeyOpCore::init()
{
  ros::NodeHandle nh("~");

  name = nh.getUnresolvedNamespace();

  /*********************
   ** Parameters
   **********************/
  nh.getParam("linear_vel_step", linear_vel_step);
  nh.getParam("linear_vel_max", linear_vel_max);
  nh.getParam("angular_vel_step", angular_vel_step);
  nh.getParam("angular_vel_max", angular_vel_max);
  nh.getParam("wait_for_connection", wait_for_connection_);

  ROS_INFO_STREAM("KeyOpCore : using linear  vel step [" << linear_vel_step << "].");
  ROS_INFO_STREAM("KeyOpCore : using linear  vel max  [" << linear_vel_max << "].");
  ROS_INFO_STREAM("KeyOpCore : using angular vel step [" << angular_vel_step << "].");
  ROS_INFO_STREAM("KeyOpCore : using angular vel max  [" << angular_vel_max << "].");

  /*********************
   ** Subscribers
   **********************/
  keyinput_subscriber = nh.subscribe("teleop", 1, &KeyOpCore::remoteKeyInputReceived, this);
  viconPosition = nh.subscribe("kobuki", 10, &KeyOpCore::vicon_transform_callback, this);

  /*********************
   ** Publishers
   **********************/
  velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  motor_power_publisher_ = nh.advertise<kobuki_msgs::MotorPower>("motor_power", 1);

  /*********************
   ** Velocities
   **********************/
  cmd->linear.x = 0.0;
  cmd->linear.y = 0.0;
  cmd->linear.z = 0.0;
  cmd->angular.x = 0.0;
  cmd->angular.y = 0.0;
  cmd->angular.z = 0.0;

  /*********************
   ** Wait for connection
   **********************/
  if (!wait_for_connection_)
  {
    return true;
  }
  ecl::MilliSleep millisleep;
  int count = 0;
  bool connected = false;
  while (!connected)
  {
    if (motor_power_publisher_.getNumSubscribers() > 0)
    {
      connected = true;
      break;
    }
    if (count == 6)
    {
      connected = false;
      break;
    }
    else
    {
      ROS_WARN_STREAM("KeyOp: could not connect, trying again after 500ms...");
      try
      {
        millisleep(500);
      }
      catch (ecl::StandardException e)
      {
        ROS_ERROR_STREAM("Waiting has been interrupted.");
        ROS_DEBUG_STREAM(e.what());
        return false;
      }
      ++count;
    }
  }
  if (!connected)
  {
    ROS_ERROR("KeyOp: could not connect.");
    ROS_ERROR("KeyOp: check remappings for enable/disable topics).");
  }
  else
  {
    kobuki_msgs::MotorPower power_cmd;
    power_cmd.state = kobuki_msgs::MotorPower::ON;
    motor_power_publisher_.publish(power_cmd);
    ROS_INFO("KeyOp: connected.");
    power_status = true;
  }

  // start keyboard input thread
  thread.start(&KeyOpCore::keyboardInputLoop, *this);
  return true;
}

/*****************************************************************************
 ** Implementation [Spin]
 *****************************************************************************/

/**
 * @brief Worker thread loop; sends current velocity command at a fixed rate.
 *
 * It also process ros functions as well as aborting when requested.
 */
void KeyOpCore::spin()
{
  ros::Rate loop_rate(10);  //was 10

  while (!quit_requested && ros::ok())
  {
    // Avoid spamming robot with continuous zero-velocity messages
    if ((cmd->linear.x  != 0.0) || (cmd->linear.y  != 0.0) || (cmd->linear.z  != 0.0) ||
        (cmd->angular.x != 0.0) || (cmd->angular.y != 0.0) || (cmd->angular.z != 0.0))
    {
      velocity_publisher_.publish(cmd);
      last_zero_vel_sent = false;
    }
    else if (last_zero_vel_sent == false)
    {
      velocity_publisher_.publish(cmd);
      last_zero_vel_sent = true;
    }

    accept_incoming = true;
    ros::spinOnce();
    loop_rate.sleep();
  }
  if (quit_requested)
  { // ros node is still ok, send a disable command
    disable();
  }
  else
  {
    // just in case we got here not via a keyboard quit request
    quit_requested = true;
    thread.cancel();
  }
  thread.join();
}

/*****************************************************************************
 ** Implementation [Keyboard]
 *****************************************************************************/

/**
 * @brief The worker thread function that accepts input keyboard commands.
 *
 * This is ok here - but later it might be a good idea to make a node which
 * posts keyboard events to a topic. Recycle common code if used by many!
 */
void KeyOpCore::keyboardInputLoop()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor, TCSANOW, &raw);

  puts("Reading from keyboard - LUO");
  puts("---------------------------");
  puts("Forward/back arrows : linear velocity incr/decr.");
  puts("Right/left arrows : angular velocity incr/decr.");
  puts("Spacebar : reset linear/angular velocities.");
  puts("d : disable motors.");
  puts("e : enable motors.");
  puts("r : return.");
  puts("m : moving.");
  puts("q : quit.");
  char c;
  while (!quit_requested)
  {
    if (read(key_file_descriptor, &c, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }
    processKeyboardInput(c);
  }
}

/**
 * @brief Callback function for remote keyboard inputs subscriber.
 */
void KeyOpCore::remoteKeyInputReceived(const kobuki_msgs::KeyboardInput& key)
{
  processKeyboardInput(key.pressedKey);
}

/**
 * @brief This function is called when local VICON position data is available.
 * In the example below, we make use of two arbitrary targets as
 * an example for local position control.
 */
void KeyOpCore::vicon_transform_callback(const geometry_msgs::TransformStamped::ConstPtr& msg) 
{
  posi_cur[0] = msg->transform.translation.x;
  posi_cur[1] = msg->transform.translation.y;
  posi_cur[2] = msg->transform.translation.z;

  double roll, pitch, yaw;
  tf::Quaternion q(
        msg->transform.rotation.x,
        msg->transform.rotation.y,
        msg->transform.rotation.z,
        msg->transform.rotation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw); 
  orien_cur = yaw; //IMPORTANT  
}

/**
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void KeyOpCore::processKeyboardInput(char c)
{
  /*
   * Arrow keys are a bit special, they are escape characters - meaning they
   * trigger a sequence of keycodes. In this case, 'esc-[-Keycode_xxx'. We
   * ignore the esc-[ and just parse the last one. So long as we avoid using
   * the last one for its actual purpose (e.g. left arrow corresponds to
   * esc-[-D) we can keep the parsing simple.
   */
  switch (c)
  {
    case kobuki_msgs::KeyboardInput::KeyCode_Left:
    {
      incrementAngularVelocity();
      break;
    }
    case kobuki_msgs::KeyboardInput::KeyCode_Right:
    {
      decrementAngularVelocity();
      break;
    }
    case kobuki_msgs::KeyboardInput::KeyCode_Up:
    {
      incrementLinearVelocity();
      break;
    }
    case kobuki_msgs::KeyboardInput::KeyCode_Down:
    {
      decrementLinearVelocity();
      break;
    }
    case kobuki_msgs::KeyboardInput::KeyCode_Space:
    {
      resetVelocity();
      break;
    }
    case 'q':
    {
      quit_requested = true;
      break;
    }
    case 'd':
    {
      disable();
      break;
    }
    case 'e':
    {
      enable();
      break;
    }
    case 'r': // reset: starts moving or returns to the designated point
    {
      resetPosition();
      break;
    }
    case 'm': // move: move the robot along the trajectory
    {
      moveTraj();
      break;
    }
    default:
    {
      ROS_INFO_ONCE("wrong key.");
      break;
    }
  }
}

/*****************************************************************************
 ** Implementation [Commands]
 *****************************************************************************/
/**
 * @brief Disables commands to the navigation system.
 *
 * This does the following things:
 *
 * - Disables power to the navigation motors (via device_manager).
 * @param msg
 */
void KeyOpCore::disable()
{
  cmd->linear.x = 0.0;
  cmd->angular.z = 0.0;
  velocity_publisher_.publish(cmd);
  accept_incoming = false;

  if (power_status)
  {
    ROS_INFO("KeyOp: die, die, die (disabling power to the device's motor system).");
    kobuki_msgs::MotorPower power_cmd;
    power_cmd.state = kobuki_msgs::MotorPower::OFF;
    motor_power_publisher_.publish(power_cmd);
    power_status = false;
  }
  else
  {
    ROS_WARN("KeyOp: Motor system has already been powered down.");
  }
}

/**
 * @brief Reset/re-enable the navigation system.
 *
 * - resets the command velocities.
 * - reenable power if not enabled.
 */
void KeyOpCore::enable()
{
  accept_incoming = false;

  cmd->linear.x = 0.0;
  cmd->angular.z = 0.0;
  velocity_publisher_.publish(cmd);

  if (!power_status)
  {
    ROS_INFO("KeyOp: Enabling power to the device subsystem.");
    kobuki_msgs::MotorPower power_cmd;
    power_cmd.state = kobuki_msgs::MotorPower::ON;
    motor_power_publisher_.publish(power_cmd);
    power_status = true;
  }
  else
  {
    ROS_WARN("KeyOp: Device has already been powered up.");
  }
}

/**
 * @brief If not already maxxed, increment the command velocities..
 */
void KeyOpCore::incrementLinearVelocity()
{
  if (power_status)
  {
    if (cmd->linear.x <= linear_vel_max)
    {
      cmd->linear.x += linear_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: linear  velocity incremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already minned, decrement the linear velocities..
 */
void KeyOpCore::decrementLinearVelocity()
{
  if (power_status)
  {
    if (cmd->linear.x >= -linear_vel_max)
    {
      cmd->linear.x -= linear_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: linear  velocity decremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already maxxed, increment the angular velocities..
 */
void KeyOpCore::incrementAngularVelocity()
{
  if (power_status)
  {
    if (cmd->angular.z <= angular_vel_max)
    {
      cmd->angular.z += angular_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: angular velocity incremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already mined, decrement the angular velocities..
 */
void KeyOpCore::decrementAngularVelocity()
{
  if (power_status)
  {
    if (cmd->angular.z >= -angular_vel_max)
    {
      cmd->angular.z -= angular_vel_step;
    }
    ROS_INFO_STREAM("KeyOp: angular velocity decremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

void KeyOpCore::resetVelocity()
{
  if (power_status)
  {
    cmd->angular.z = 0.0;
    cmd->linear.x = 0.0;
    ROS_INFO_STREAM("KeyOp: reset linear/angular velocities.");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

void KeyOpCore::resetPosition2()
{
  if (power_status)
  {
    if (((std::abs(start_position[0] - posi_cur[0])) < 0.1) && ((std::abs(start_position[1] - posi_cur[1])) < 0.1))
    {        
      double ang_err_second = start_orientation - orien_cur;
      //anti-windup
      if (ang_err_second < -1.0*PI) 
      {
        ang_err_second += 2.0*PI;
      }
      else if (ang_err_second > PI)
      {
        ang_err_second -= 2.0*PI;
      }     
      
      if (std::abs(ang_err_second) < 3.0/180 * PI)
      {
        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        ROS_INFO_ONCE("=========Position Reset Complete========");
        return;   //stop the program here and avoid the following  
      } 
      else 
      {
        cmd->angular.z =  ang_err_second * K_ang;
        cmd->linear.x = 0.0;
        ROS_INFO_STREAM("KeyOp: align final orientation.");
        ROS_INFO_STREAM("target_ang(deg) = "<<start_orientation * 180/PI<<", orien_cur(deg) = "
                        <<orien_cur * 180/PI<<", ang_err(deg) = "<<ang_err_second * 180/PI);  
      }         
    }

    double norm = sqrt((start_position[0] - posi_cur[0])*(start_position[0] - posi_cur[0])
              + (start_position[1] - posi_cur[1])*(start_position[1] - posi_cur[1]));
    double target_ang = atan2((start_position[1] - posi_cur[1]), (start_position[0] - posi_cur[0]));
    double ang_err_first = target_ang - orien_cur;

    //anti-windup
    if (ang_err_first < -1.0*PI) 
    {
      ang_err_first += 2.0*PI;
    }
    else if (ang_err_first > PI)
    {
      ang_err_first -= 2.0*PI;
    }

    cmd->angular.z = ang_err_first * K_ang;
    cmd->linear.x = norm * K_lin;
    ROS_INFO_STREAM("KeyOp: reset position.");
    // ROS_INFO_STREAM("target_ang(deg) = "<<target_ang * 180/PI<<", orien_cur(deg) = "
                    // <<orien_cur * 180/PI<<", ang_err(deg) = "<<ang_err_first * 180/PI);
    // ROS_INFO_STREAM("posi cur x = "<<posi_cur[0]<<", posi cur y = "<<posi_cur[1]<<", orien cur(deg) = "<<orien_cur * 180/PI);    
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

void KeyOpCore::resetPosition()
{
  if (power_status)
  {
    ROS_INFO_STREAM_ONCE("KeyOp: reset position.");
    while(!(((std::abs(start_position[0] - posi_cur[0])) < 0.1) && ((std::abs(start_position[1] - posi_cur[1])) < 0.1))) 
    {
      double norm = sqrt((start_position[0] - posi_cur[0])*(start_position[0] - posi_cur[0])
              + (start_position[1] - posi_cur[1])*(start_position[1] - posi_cur[1]));
      double target_ang = atan2((start_position[1] - posi_cur[1]), (start_position[0] - posi_cur[0]));
      ang_err_first = target_ang - orien_cur;

      //anti-windup
      if (ang_err_first < -1.0*PI) 
      {
        ang_err_first += 2.0*PI;
      }
      else if (ang_err_first > PI)
      {
        ang_err_first -= 2.0*PI;
      }

      cmd->angular.z = ang_err_first * K_ang;
      cmd->linear.x = norm * K_lin;  
      //limit speeds
      if (cmd->angular.z > angular_vel_max)   {
        cmd->angular.z = angular_vel_max;  
      }               
      else if(cmd->angular.z < -1.0*angular_vel_max) {
        cmd->angular.z = -1.0*angular_vel_max; 
      }

      if (cmd->linear.x > linear_vel_max) {
        cmd->linear.x = linear_vel_max;
      }
      // ROS_INFO_STREAM("target_ang(deg) = "<<target_ang * 180/PI<<", orien_cur(deg) = "
                      // <<orien_cur * 180/PI<<", ang_err(deg) = "<<ang_err_first * 180/PI);
      // ROS_INFO_STREAM("posi cur x = "<<posi_cur[0]<<", posi cur y = "<<posi_cur[1]<<", orien cur(deg) = "<<orien_cur * 180/PI);    
      // ROS_INFO_STREAM("after: cmd->angular.z = "<<cmd->angular.z<<", cmd->linear.x = "<<cmd->linear.x);
    }
    
    ROS_INFO_STREAM_ONCE("KeyOp: align final orientation.");
    do {
      ang_err_second = start_orientation - orien_cur;
      //anti-windup
      if (ang_err_second < -1.0*PI) 
      {
        ang_err_second += 2.0*PI;
      }
      else if (ang_err_second > PI)
      {
        ang_err_second -= 2.0*PI;
      }     
      
      cmd->angular.z =  ang_err_second * K_ang;
      cmd->linear.x = 0.0; 
      //limit speeds
      if (cmd->angular.z > angular_vel_max)      
        cmd->angular.z = angular_vel_max;      
      else if(cmd->angular.z < -1.0*angular_vel_max)
        cmd->angular.z = -angular_vel_max; 

      // ROS_INFO_STREAM("target_ang(deg) = "<<start_orientation * 180/PI<<", orien_cur(deg) = "
                      // <<orien_cur * 180/PI<<", ang_err(deg) = "<<ang_err_second * 180/PI);  
              
    } while(!(std::abs(ang_err_second) < 3.0/180 * PI));

    cmd->angular.z = 0.0;
    cmd->linear.x = 0.0;
    ROS_INFO_STREAM_ONCE("=========Position Reset Complete========");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}

void KeyOpCore::moveTraj()
{
  if (power_status)
  {
    if (MOVE_LINE_TRAJ)
    { 
      ROS_INFO_STREAM_ONCE("KeyOp: moving the given trajectory.");
      while(!(((std::abs(end_position[0] - posi_cur[0])) < 0.1) && ((std::abs(end_position[1] - posi_cur[1])) < 0.1))) 
      {
        double norm = sqrt((end_position[0] - posi_cur[0])*(end_position[0] - posi_cur[0])
                + (end_position[1] - posi_cur[1])*(end_position[1] - posi_cur[1]));
        double target_ang = atan2((end_position[1] - posi_cur[1]), (end_position[0] - posi_cur[0]));
        ang_err_mt = target_ang - orien_cur;

        //anti-windup
        if (ang_err_mt < -1.0*PI) 
        {
          ang_err_mt += 2.0*PI;
        }
        else if (ang_err_mt > PI)
        {
          ang_err_mt -= 2.0*PI;
        }

        cmd->angular.z = ang_err_mt * K_ang;
        cmd->linear.x = MT_VEL;
        //limit speeds
        if (cmd->angular.z > angular_vel_max)      
          cmd->angular.z = angular_vel_max;      
        else if(cmd->angular.z < -1.0*angular_vel_max)
          cmd->angular.z = -angular_vel_max; 
      
        // ROS_INFO_STREAM("target_ang(deg) = "<<target_ang * 180/PI<<", orien_cur(deg) = "
                        // <<orien_cur * 180/PI<<", ang_err(deg) = "<<ang_err_mt * 180/PI);
        // ROS_INFO_STREAM("posi cur x = "<<posi_cur[0]<<", posi cur y = "<<posi_cur[1]<<", orien cur(deg) = "<<orien_cur * 180/PI);    
      }
    }
    
    if (!MOVE_LINE_TRAJ) 
    {
      ROS_INFO_STREAM_ONCE("KeyOp: moving the given trajectory -- PART 1.");
      while(!(((std::abs(turn_point[0] - posi_cur[0])) < 0.1) && ((std::abs(turn_point[1] - posi_cur[1])) < 0.1))) 
      {
        double norm = sqrt((turn_point[0] - posi_cur[0])*(turn_point[0] - posi_cur[0])
                + (turn_point[1] - posi_cur[1])*(turn_point[1] - posi_cur[1]));
        double target_ang = atan2((turn_point[1] - posi_cur[1]), (turn_point[0] - posi_cur[0]));
        ang_err_mt = target_ang - orien_cur;

        //anti-windup
        if (ang_err_mt < -1.0*PI) 
        {
          ang_err_mt += 2.0*PI;
        }
        else if (ang_err_mt > PI)
        {
          ang_err_mt -= 2.0*PI;
        }

        cmd->angular.z = ang_err_mt * K_ang;
        cmd->linear.x = MT_VEL;
        //limit speeds
        if (cmd->angular.z > angular_vel_max)      
          cmd->angular.z = angular_vel_max;      
        else if(cmd->angular.z < -1.0*angular_vel_max)
          cmd->angular.z = -angular_vel_max; 
      
        // ROS_INFO_STREAM("target_ang(deg) = "<<target_ang * 180/PI<<", orien_cur(deg) = "
                        // <<orien_cur * 180/PI<<", ang_err(deg) = "<<ang_err_mt * 180/PI);
        // ROS_INFO_STREAM("posi cur x = "<<posi_cur[0]<<", posi cur y = "<<posi_cur[1]<<", orien cur(deg) = "<<orien_cur * 180/PI);    
      }
      
      ROS_INFO_STREAM_ONCE("KeyOp: moving the given trajectory -- PART 2: rotation.");
      do {
        ang_err_second = end_orientation - orien_cur;
        //anti-windup
        if (ang_err_second < -1.0*PI) 
        {
          ang_err_second += 2.0*PI;
        }
        else if (ang_err_second > PI)
        {
          ang_err_second -= 2.0*PI;
        }     
        
        cmd->angular.z =  ang_err_second * K_ang;
        cmd->linear.x = 0.0; 
        //limit speeds
        if (cmd->angular.z > angular_vel_max)      
          cmd->angular.z = angular_vel_max;      
        else if(cmd->angular.z < -1.0*angular_vel_max)
          cmd->angular.z = -angular_vel_max; 

        // ROS_INFO_STREAM("target_ang(deg) = "<<start_orientation * 180/PI<<", orien_cur(deg) = "
                        // <<orien_cur * 180/PI<<", ang_err(deg) = "<<ang_err_second * 180/PI);  
                
      } while(!(std::abs(ang_err_second) < 3.0/180 * PI));

      cmd->angular.z = 0.0;
      cmd->linear.x = 0.0;
    
      ROS_INFO_STREAM_ONCE("KeyOp: moving the given trajectory -- PART 3.");
      while(!(((std::abs(end_position[0] - posi_cur[0])) < 0.1) && ((std::abs(end_position[1] - posi_cur[1])) < 0.1))) 
      {
        double norm = sqrt((end_position[0] - posi_cur[0])*(end_position[0] - posi_cur[0])
                + (end_position[1] - posi_cur[1])*(end_position[1] - posi_cur[1]));
        double target_ang = atan2((end_position[1] - posi_cur[1]), (end_position[0] - posi_cur[0]));
        ang_err_mt = target_ang - orien_cur;

        //anti-windup
        if (ang_err_mt < -1.0*PI) 
        {
          ang_err_mt += 2.0*PI;
        }
        else if (ang_err_mt > PI)
        {
          ang_err_mt -= 2.0*PI;
        }

        cmd->angular.z = ang_err_mt * K_ang;
        cmd->linear.x = MT_VEL;
        //limit speeds
        if (cmd->angular.z > angular_vel_max)      
          cmd->angular.z = angular_vel_max;      
        else if(cmd->angular.z < -1.0*angular_vel_max)
          cmd->angular.z = -angular_vel_max; 
      
        // ROS_INFO_STREAM("target_ang(deg) = "<<target_ang * 180/PI<<", orien_cur(deg) = "
                        // <<orien_cur * 180/PI<<", ang_err(deg) = "<<ang_err_mt * 180/PI);
        // ROS_INFO_STREAM("posi cur x = "<<posi_cur[0]<<", posi cur y = "<<posi_cur[1]<<", orien cur(deg) = "<<orien_cur * 180/PI);    
      }

    }
    
    cmd->angular.z = 0.0;
    cmd->linear.x = 0.0;
    ROS_INFO_STREAM_ONCE("=========Trajectory Complete========");
  }
  else
  {
    ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
  }
}


} // namespace keyop_core

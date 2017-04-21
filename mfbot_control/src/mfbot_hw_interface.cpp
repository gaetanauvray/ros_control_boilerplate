/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the MFBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <mfbot_control/mfbot_hw_interface.h>
#include "std_msgs/String.h"
#include <sstream>

//new test

namespace mfbot_control
{

MFBotHWInterface::MFBotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("mfbot_hw_interface", "MFBotHWInterface Ready.");

  joint_states_sub_ = nh.subscribe("/mfbot_arduino_states",   1, &MFBotHWInterface::callback,  this);
  joint_cmd_pub_ = nh.advertise<ros_control_boilerplate::mfbot_joint_cmd>("/mfbot_arduino_cmd", 1, true);

  ROS_INFO_NAMED("mfbot_hw_interface", "MFBotHWInterface pub and sub created.");
}

void MFBotHWInterface::callback(const ros_control_boilerplate::mfbot_joint_states::ConstPtr &msg)
{
  if(msg){
    new_data_ready_ = true;
    mfbot_input_msg_ = *msg;
  }
}


void MFBotHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  // Resize vectors
  //joint_position_prev_.resize(num_joints_, 0.0);

  ROS_INFO_NAMED(name_, "MFbotHWInterface Ready.");
}

void MFBotHWInterface::read(ros::Duration& elapsed_time)
{

  joint_position_[0] = mfbot_input_msg_.joint0 / 180.0 * 3.14159265359;
  joint_position_[1] = (mfbot_input_msg_.joint1 - 90)/ 180.0 * 3.14159265359;

}

void MFBotHWInterface::write(ros::Duration &elapsed_time)
  {
    // Safety
    enforceLimits(elapsed_time);


    mfbot_output_msg_.cmd0 = static_cast<int>(joint_position_command_[0]* 180.0 / 3.14159265359);
    mfbot_output_msg_.cmd1 = static_cast<int>(joint_position_command_[1]* 180.0 / 3.14159265359 + (90.0));

    joint_cmd_pub_.publish(mfbot_output_msg_);

    // ----------------------------------------------------
    //
    // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
    //
    // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
    // VELOCITY FROM POSITION WITH SMOOTHING, SEE
    // sim_hw_interface.cpp IN THIS PACKAGE
    //
    // DUMMY PASS-THROUGH CODE
//    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
//     joint_position_[joint_id] += joint_position_command_[joint_id];
    //END DUMMY CODE
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
//    // ----------------------------------------------------
//    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
//    {
//        const double max_delta_pos = joint_velocity_limits_[joint_id] * elapsed_time.toSec();

//        // Move all the states to the commanded set points at max velocity
//        p_error_ = joint_position_command_[joint_id] - joint_position_[joint_id];

//        const double delta_pos = std::max(std::min(p_error_, max_delta_pos), -max_delta_pos);
//        joint_position_[joint_id] += delta_pos;

//        // Bypass max velocity p controller:
//        //joint_position_[joint_id] = joint_position_command_[joint_id];

//        // Calculate velocity based on change in positions
//        if (elapsed_time.toSec() > 0)
//        {
//          joint_velocity_[joint_id] = (joint_position_[joint_id] - joint_position_prev_[joint_id]) / elapsed_time.toSec();
//        }
//        else
//          joint_velocity_[joint_id] = 0;

//        // Save last position
//        joint_position_prev_[joint_id] = joint_position_[joint_id];
    }


void MFBotHWInterface::enforceLimits(ros::Duration & period)
  {
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    //
    // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
    // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
    // DEPENDING ON YOUR CONTROL METHOD
    //
    // EXAMPLES:
    //
    // Saturation Limits ---------------------------
    //
    // Enforces position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces velocity and acceleration limits
    // vel_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces position, velocity, and effort
    // eff_jnt_sat_interface_.enforceLimits(period);

    // Soft limits ---------------------------------
    //
    // pos_jnt_soft_limits_.enforceLimits(period);
    // vel_jnt_soft_limits_.enforceLimits(period);
    // eff_jnt_soft_limits_.enforceLimits(period);
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
  }

} // namespace

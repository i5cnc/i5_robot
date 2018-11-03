/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, i5cnc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the i5cnc nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Author: Guo Wei, Liu Fang - i5cnc
 */

#include "i5_driver/joint_feedback_handler.h"
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>

namespace industrial_robot_client
{
namespace joint_feedback_handler
{
bool JointFeedbackHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::vector<std::string> &joint_names)
{
  this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states", 1);
  this->pub_joint_control_state_ =
          this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
  // save "complete" joint-name list, preserving any blank entries for later use
  this->all_joint_names_ = joint_names;

  return init((int)StandardMsgTypes::JOINT_FEEDBACK, connection);
}

bool JointFeedbackHandler::internalCB(SimpleMessage& in)
{
//        if(in.getMessageType() == StandardMsgTypes::JOINT_FEEDBACK)
//        {
  JointFeedbackMessage joint_msg;

  if (!joint_msg.init(in))
  {
    LOG_ERROR("Failed to initialize joint feedback message");
    return false;
  }

  return internalCB(joint_msg);
//        }
}

bool JointFeedbackHandler::internalCB(JointFeedbackMessage & msg_in)
{

  bool rtn = true;

  // read joint positions from JointMessage
  std::vector<double> all_joint_pos;
  std::vector<double> all_joint_vel;
  std::vector<double> all_joint_effort;
  for (int i = 0; i < all_joint_names_.size(); ++i)
  {
    industrial::joint_data::JointData tmp;
    industrial::shared_types::shared_real value;
    if (msg_in.getPositions(tmp))
    {
      if (tmp.getJoint(i, value))
        all_joint_pos.push_back(value);
      else
        rtn = false;
    }
    else
    {
      LOG_ERROR("Failed to parse #%d pos value from JointFeedback", i);
      rtn = false;
    }

    if (msg_in.getVelocities(tmp))
    {
      if (tmp.getJoint(i, value))
        all_joint_vel.push_back(value);
      else
        rtn = false;
    }
    else
    {
      LOG_ERROR("Failed to parse #%d vel value from JointFeedback", i);
      rtn = false;
    }
    //this is really the effort.
    if (msg_in.getAccelerations(tmp))
    {
      if (tmp.getJoint(i, value))
        all_joint_effort.push_back(value);
      else
        rtn = false;
    }
    else
    {
      LOG_ERROR("Failed to parse #%d effort value from JointFeedback", i);
      rtn = false;
    }
  }

  sensor_msgs::JointState sensor_state;
  sensor_state.header.stamp = ros::Time::now();
  sensor_state.name = all_joint_names_;
  sensor_state.position = all_joint_pos;
  sensor_state.velocity = all_joint_vel;
  sensor_state.effort = all_joint_effort;

//        if(rtn)
  this->pub_joint_sensor_state_.publish(sensor_state);


  control_msgs::FollowJointTrajectoryFeedback control_state;
  control_state.header.stamp = ros::Time::now();
  control_state.joint_names = all_joint_names_;
  control_state.actual.positions = all_joint_pos;
  this->pub_joint_control_state_.publish(control_state);
//        else
//              LOG_ERROR("joint_feedback_message error, failed to publish the topic.");

// Reply back to the controller if the sender requested it.
  if (CommTypes::SERVICE_REQUEST == msg_in.getCommType())
  {
    SimpleMessage reply;
    msg_in.toReply(reply, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  return rtn;
}


}

}





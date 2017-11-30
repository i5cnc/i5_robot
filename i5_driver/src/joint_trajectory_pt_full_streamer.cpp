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


#include "i5_driver/joint_trajectory_pt_full_streamer.h"




namespace industrial_robot_client
{
namespace joint_trajectory_pt_full_streamer
{

bool JointTrajectoryPtFullStreamer::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                   const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("JointTrajectoryPtFullStreamer: init");

  sub_robot_status_ = node_.subscribe("robot_status", 1, &JointTrajectoryPtFullStreamer::robotStatusCB, this);

  rtn &= JointTrajectoryInterface::init(connection, joint_names, velocity_limits);
//  this->srv_stop_motion_ = this->node_.advertiseService("stop_motion", &JointTrajectoryPtFullStreamer::stopMotionCB, this);
//  this->mutex_.lock();

  boost::mutex::scoped_lock lock(this->mutex_);

  this->current_point_ = 0;
  this->state_ = TransferStates::IDLE;
  this->streaming_thread_ =
      new boost::thread(boost::bind(&JointTrajectoryPtFullStreamer::streamingThread, this));
//  ROS_INFO("Unlocking mutex");
//  this->mutex_.unlock();

  return rtn;
}

JointTrajectoryPtFullStreamer::~JointTrajectoryPtFullStreamer()
{
  delete this->streaming_thread_;
}

JointTrajPtFullMessage JointTrajectoryPtFullStreamer::create_message(int seq, Industrial_JointTrajPtPull& pt_full)
{

  JointTrajPtFullMessage msg;
  industrial::joint_data::JointData pos;
  industrial::joint_data::JointData vel;
  industrial::joint_data::JointData acc;

  float time =0.0;
  //  ROS_ASSERT(joint_pos.size() <= (unsigned int)pos.getMaxNumJoints());
  if(!pt_full.getTime(time))
  {
    ROS_ERROR("getTime error");
    return msg;
  }
  if(!pt_full.getPositions(pos))
  {
    ROS_ERROR("getPositions error");
    return msg;
  }

  if(!pt_full.getVelocities(vel))
  {
    ROS_ERROR("getVelocities error");
    return msg;
  }

  int valid_fields = ValidFieldTypes::TIME | ValidFieldTypes::POSITION | ValidFieldTypes::VELOCITY | ValidFieldTypes::ACCELERATION;
  //TODO
  Industrial_JointTrajPtPull pt_full_data;
  pt_full_data.init(0, seq, valid_fields, time ,pos, vel, acc);

  msg.init(pt_full_data);

  return msg;
}

bool JointTrajectoryPtFullStreamer::select(int seq, const std::vector<std::string>& ros_joint_names, const Industrial_JointTrajPoint& ros_pt,
                      const std::vector<std::string>& rbt_joint_names, Industrial_JointTrajPtPull& rbt_pt)
{
    ROS_ASSERT(ros_joint_names.size() == ros_pt.positions.size());

    rbt_pt.init();

  industrial::joint_data::JointData pos;
  industrial::joint_data::JointData vel;
  industrial::joint_data::JointData acc;

//    std::cerr << "Vel: ";
  for (size_t rbt_idx=0; rbt_idx < rbt_joint_names.size(); ++rbt_idx)
  {
    bool is_empty = rbt_joint_names[rbt_idx].empty();

  // find matching ROS element
    size_t ros_idx = std::find(ros_joint_names.begin(), ros_joint_names.end(), rbt_joint_names[rbt_idx]) - ros_joint_names.begin();
    bool is_found = ros_idx < ros_joint_names.size();

      // error-chk: required robot joint not found in ROS joint-list
      if (!is_empty && !is_found)
      {
          ROS_ERROR("Expected joint (%s) not found in JointTrajectory.  Aborting command.", rbt_joint_names[rbt_idx].c_str());
          return false;
      }

      if (is_empty)
      {
        if (!ros_pt.positions.empty())
        {
          pos.setJoint(rbt_idx, default_joint_pos_);
        }
        if (!ros_pt.velocities.empty())
        {
          vel.setJoint(rbt_idx, 0.0);
        }
        if (!ros_pt.accelerations.empty())
        {
          acc.setJoint(rbt_idx, 0.0);
        }
      }
      else
      {
        if (!ros_pt.positions.empty())
        {
          pos.setJoint(rbt_idx, ros_pt.positions[ros_idx]);
        }
        if (!ros_pt.velocities.empty())
        {
          vel.setJoint(rbt_idx, ros_pt.velocities[ros_idx]);
//                std::cerr << "index: " << rbt_idx << "--" << ros_pt.velocities[ros_idx] << "\t";
        }
        if (!ros_pt.accelerations.empty())
        {
          acc.setJoint(rbt_idx, ros_pt.accelerations[ros_idx]);
        }
      }

        // robot_id, sequence, fields, time, pos, vel, acc
      float time = ros_pt.time_from_start.toSec();
      int valid_fields = ValidFieldTypes::TIME | ValidFieldTypes::POSITION | ValidFieldTypes::VELOCITY | ValidFieldTypes::ACCELERATION;
        rbt_pt.init(0, seq, valid_fields, time ,pos, vel, acc);
    }
//  std::cerr << std::endl;
  return true;
}

bool JointTrajectoryPtFullStreamer::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector<JointTrajPtFullMessage>* msgs)
{
  msgs->clear();

  // check for valid trajectory
  if (!is_valid(*traj))
    return false;

//  ros_JointTrajPtFull rbt_pt_full, xform_pt_full;
//  JointTrajPtFullMessage msg = create_message(i, xform_pt.positions, vel, duration);
  for (size_t i=0; i<traj->points.size(); ++i)
  {
    Industrial_JointTrajPtPull xform_pt_full;
//    double vel, duration;

    // select / reorder joints for sending to robot
    if (!select(i, traj->joint_names, traj->points[i], this->all_joint_names_, xform_pt_full))
      return false;

    // transform point data (e.g. for joint-coupling)
//    if (!transform(rbt_pt, &xform_pt))
//      return false;

    // reduce velocity to a single scalar, for robot command
//    if (!calc_speed(xform_pt, &vel, &duration))
//      return false;

//    JointTrajPtFullMessage msg = create_message(i, xform_pt_full);

  JointTrajPtFullMessage msg;
  msg.init(xform_pt_full);
    msgs->push_back(msg);
  }

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))
  {
    ROS_DEBUG("Padding trajectory: current(%d) => minimum(%d)", (int)msgs->size(), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }


  return true;
}

void JointTrajectoryPtFullStreamer::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);
  if (TransferStates::IDLE != state)
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

//  this->mutex_.lock();
    boost::mutex::scoped_lock lock(this->mutex_);
    trajectoryStop();
//  this->mutex_.unlock();
    return;
  }

//  // check for STOP command
//  if (msg->points.empty())
//  {
//    ROS_INFO("Empty trajectory received, canceling current trajectory");
//    trajectoryStop();
//    return;
//  }

  if (msg->points.empty())
  {
    //empty points request controller to stop.
    if(robot_status_ && robot_status_->in_motion.val == industrial_msgs::TriState::TRUE)
    {
      boost::mutex::scoped_lock lock(this->mutex_);
      trajectoryStop();
    }
    ROS_INFO("Empty trajectory received while in IDLE state, nothing is done");
    return;
  }
  //check motion possible
  if(!robot_status_)
  {
      ROS_INFO("Can't send trajectory, no robot status");
      return;
  }
  else
  {
    if(robot_status_->motion_possible.val == industrial_msgs::TriState::FALSE)
    {
      ROS_INFO("Can't send trajectory, motion not possible");
      return;
    }
  }

  // calc new trajectory
  std::vector<JointTrajPtFullMessage> new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return;

  // send command messages to robot
  send_to_robot(new_traj_msgs);
}

bool JointTrajectoryPtFullStreamer::send_to_robot(const std::vector<JointTrajPtFullMessage>& messages)
{
  ROS_INFO("Loading trajectory, setting state to streaming");
//  this->mutex_.lock();

  boost::mutex::scoped_lock lock(this->mutex_);
  {
    ROS_INFO("Executing trajectory of size: %d", (int)messages.size());
    this->current_traj_ = messages;
    this->current_point_ = 0;
    this->state_ = TransferStates::STREAMING;
    this->streaming_start_ = ros::Time::now();
  }
//  this->mutex_.unlock();

  return true;
}

void JointTrajectoryPtFullStreamer::robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  robot_status_ = msg; //caching robot status for later use.
}

bool JointTrajectoryPtFullStreamer::stopMotionCB(industrial_msgs::StopMotion::Request &req,
                                  industrial_msgs::StopMotion::Response &res)
{
  ROS_INFO("Stop motion callbacks");
  std::cerr<<"shit\n";
  trajectoryStop();
  // no success/fail result from trajectoryStodp.  Assume success.
  res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}


void JointTrajectoryPtFullStreamer::streamingThread()
{
  JointTrajPtFullMessage jtpMsg;
  jtpMsg.setSequence(SpecialSeqValues::START_TRAJECTORY_STREAMING);
  int connectRetryCount = 1;

  ROS_INFO("Starting joint trajectory streamer thread");
  while (ros::ok())
  {
    ros::Duration(0.005).sleep();

    // automatically re-establish connection, if required
    if (connectRetryCount-- > 0)
    {
      ROS_INFO("Connecting to robot motion server");
      this->connection_->makeConnect();
      ros::Duration(0.250).sleep();  // wait for connection

      if (this->connection_->isConnected())
        connectRetryCount = 0;
      else if (connectRetryCount <= 0)
      {
        ROS_ERROR("Timeout connecting to robot controller.  Send new motion command to retry.");
        this->state_ = TransferStates::IDLE;
      }
      continue;
    }

//    this->mutex_.lock();
    boost::mutex::scoped_lock lock(this->mutex_);

    SimpleMessage msg, reply;

    switch (this->state_)
    {
      case TransferStates::IDLE:
        ros::Duration(0.250).sleep();  //  slower loop while waiting for new trajectory
        break;

      case TransferStates::STREAMING:
        if (this->current_point_ >= (int)this->current_traj_.size())
        {
          ROS_INFO("Trajectory streaming complete, setting state to IDLE");
          this->state_ = TransferStates::IDLE;
          break;
        }

        if (!this->connection_->isConnected())
        {
          ROS_DEBUG("Robot disconnected.  Attempting reconnect...");
          connectRetryCount = 5;
          break;
        }

        jtpMsg = this->current_traj_[this->current_point_];
        jtpMsg.toRequest(msg);

        ROS_DEBUG("Sending joint trajectory point");
        if (this->connection_->sendAndReceiveMsg(msg, reply, false))
        {
          ROS_INFO("Point[%d of %d] sent to controller",
                   this->current_point_, (int)this->current_traj_.size());
          this->current_point_++;
        }
        else
          ROS_WARN("Failed sent joint point, will try again");

        break;
      default:
        ROS_ERROR("Joint trajectory streamer: unknown state");
        this->state_ = TransferStates::IDLE;
        break;
    }

//    this->mutex_.unlock();
  }

  ROS_WARN("Exiting trajectory streamer thread");
}

void JointTrajectoryPtFullStreamer::trajectoryStop()
{
  JointTrajPtFullMessage jMsg;
  SimpleMessage msg, reply;

  ROS_INFO("Joint trajectory handler: entering stopping state");
  jMsg.setSequence(SpecialSeqValues::STOP_TRAJECTORY);
  jMsg.toRequest(msg);
  ROS_DEBUG("Sending stop command");
  this->connection_->sendAndReceiveMsg(msg, reply);

  ROS_DEBUG("Stop command sent, entering idle mode");
  this->state_ = TransferStates::IDLE;
}

} //joint_trajectory_pt_full_streamer
} //industrial_robot_client

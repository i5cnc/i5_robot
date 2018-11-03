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


#ifndef JOINT_TRAJECTORY_PT_FULL_STREAMER_H_
#define JOINT_TRAJECTORY_PT_FULL_STREAMER_H_

#include <boost/thread/thread.hpp>
#include <industrial_robot_client/joint_trajectory_interface.h>
#include <industrial_msgs/RobotStatus.h>
#include <simple_message/messages/joint_traj_pt_message.h>
#include <simple_message/messages/joint_traj_pt_full_message.h>
#include <simple_message/joint_traj_pt_full.h>

typedef industrial::joint_traj_pt_full::JointTrajPtFull Industrial_JointTrajPtPull;
typedef trajectory_msgs::JointTrajectoryPoint Industrial_JointTrajPoint;

namespace industrial_robot_client
{
namespace joint_trajectory_pt_full_streamer
{

using namespace industrial::joint_traj_pt_full;
using industrial::simple_message::SimpleMessage;
using industrial_robot_client::joint_trajectory_interface::JointTrajectoryInterface;
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::joint_traj_pt_message::JointTrajPtMessage;

namespace TransferStates
{
enum TransferState
{
  IDLE = 0, STREAMING = 1 //,STARTING, //, STOPPING
};
}
typedef TransferStates::TransferState TransferState;

/**
 * \brief Message handler that streams joint trajectories to the robot controller
 */

//* JointTrajectoryStreamer
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointTrajectoryPtFullStreamer : public JointTrajectoryInterface
{

public:

  // since this class defines a different init(), this helps find the base-class init()
  using JointTrajectoryInterface::init;

  /**
   * \brief Default constructor
   *
   * \param min_buffer_size minimum number of points as required by robot implementation
   */
  JointTrajectoryPtFullStreamer(int min_buffer_size = 1) :
      min_buffer_size_(min_buffer_size)
  {
  }
  ;

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_names list of expected joint-names.
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   * \param velocity_limits map of maximum velocities for each joint
   *   - leave empty to lookup from URDF
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                    const std::map<std::string, double> &velocity_limits = std::map<std::string, double>());

  ~JointTrajectoryPtFullStreamer();

  virtual void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg) override;

  JointTrajPtFullMessage create_message(int seq, Industrial_JointTrajPtPull& pt_full);

  bool select(int seq, const std::vector<std::string>& ros_joint_names, const Industrial_JointTrajPoint& ros_pt,
              const std::vector<std::string>& rbt_joint_names, Industrial_JointTrajPtPull& rbt_pt);

//  virtual bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector<JointTrajPtMessage>* msgs);
  bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj,
                          std::vector<JointTrajPtFullMessage>* msgs);

  void streamingThread();

  bool send_to_robot(const std::vector<JointTrajPtFullMessage>& messages);

  // pure virtual funciton
  virtual bool send_to_robot(const std::vector<JointTrajPtMessage>& messages)
  {
    return false;
  }

  void robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg);

  bool stopMotionCB(industrial_msgs::StopMotion::Request &req,
                                    industrial_msgs::StopMotion::Response &res) final;


protected:

  void trajectoryStop() final;

  boost::thread* streaming_thread_;
  boost::mutex mutex_;
  int current_point_;
  std::vector<JointTrajPtFullMessage> current_traj_;
  TransferState state_;
  ros::Time streaming_start_;
  int min_buffer_size_;

  ros::Subscriber sub_robot_status_;
  industrial_msgs::RobotStatusConstPtr robot_status_;

};

} //joint_trajectory_pt_full_streamer
}


#endif

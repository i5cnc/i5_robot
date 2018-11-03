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

#ifndef IO_SERVICE_H_
#define IO_SERVICE_H_

#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <simple_message/smpl_msg_connection.h>
#include <simple_message/socket/tcp_client.h>
#include <simple_message/simple_message.h>

// set/get IO
#include "i5_msgs/ReadDigitalInput.h"
#include "i5_msgs/WriteDigitalOutput.h"

#include "i5_simple_message/messages/i5_read_single_io_message.h"
#include "i5_simple_message/messages/i5_write_single_io_message.h"

using namespace i5::simple_message::io_message;

namespace industrial_robot_client
{

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::tcp_client::TcpClient;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;
namespace io_service
{
/**
 * \brief Message handler that relays joint trajectories to the robot controller
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class IOService
{

public:

  /**
   * \brief Default constructor.
   */
  IOService()
    :node_("~")
  {
  }
  ;

  /**
   * \brief Initialize robot connection using default method.
   *
   * \param default_ip default IP address to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "robot_ip_address" cannot be read
   * \param default_port default port to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "~port" cannot be read
   *
   * \return true on success, false otherwise
   */
  virtual bool init(std::string default_ip = "", int default_port = StandardSocketPorts::IO);

  virtual ~IOService()
  {
  }

  /**
   * \brief Begin processing messages and publishing topics.
   */
  virtual void run()
  {
    ros::spin();
  }

protected:

  TcpClient default_tcp_connection_;

  ros::NodeHandle node_;
  SmplMsgConnection* connection_;

  ros::ServiceServer srv_set_io_;  // set IO srv
  ros::ServiceServer srv_get_io_in_;  // get IO input srv
protected:

  bool writeDigitalOutputCB(i5_msgs::WriteDigitalOutput::Request &req, i5_msgs::WriteDigitalOutput::Response &res);
  bool readDigitalInputCB(i5_msgs::ReadDigitalInput::Request &req, i5_msgs::ReadDigitalInput::Response &res);
};
}

} //i5

#endif /* IO_SERVICE_H */

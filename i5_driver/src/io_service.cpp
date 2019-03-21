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

#include <industrial_utils/param_utils.h>
#include "i5_driver/io_service.h"

using namespace i5;

using namespace industrial_utils::param;
using industrial::simple_message::SimpleMessage;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;
using namespace industrial_robot_client::io_service;

#define ROS_ERROR_RETURN(rtn,...) do {ROS_ERROR(__VA_ARGS__); return(rtn);} while(0)

bool IOService::init(std::string default_ip, int default_port)
{

  this->srv_set_io_ = this->node_.advertiseService("write_digital_output", &IOService::writeDigitalOutputCB, this);

  this->srv_get_io_in_ = this->node_.advertiseService("read_digital_input", &IOService::readDigitalInputCB, this);

  std::string ip;
  int port;

  // override IP/port with ROS params, if available(in private namespace)
  ros::param::param<std::string>("~robot_ip_address", ip, default_ip);
  if (ip.empty())
    ros::param::param<std::string>("robot_ip_address", ip, default_ip);  // find in global namespace
  ros::param::param<int>("~port", port, default_port);

  // check for valid parameter values
  if (ip.empty())
  {
    ROS_ERROR("No valid robot IP address found.  Please set ROS '~robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str()); // connection.init() requires "char*", not "const char*"
  ROS_INFO("i5Robot io_service connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);

  connection_ = &default_tcp_connection_;

  return connection_->makeConnect();
}

bool IOService::writeDigitalOutputCB(i5_msgs::WriteDigitalOutput::Request &req,
                                     i5_msgs::WriteDigitalOutput::Response &res)
{

  WriteSingleIOMessage io_msg;

  industrial::simple_message::SimpleMessage msg, reply;

  io_msg.cmd_.setType(req.type);
  io_msg.cmd_.setAddress(req.index);
  io_msg.cmd_.setValue(req.value);

  // fill to msg
  industrial::byte_array::ByteArray data;
  data.load(io_msg);
  msg.init(io_msg.getMessageType(), industrial::simple_message::CommTypes::SERVICE_REQUEST,
           industrial::simple_message::ReplyTypes::INVALID, data);

  this->connection_->sendAndReceiveMsg(msg, reply);

  res.code.val =
      reply.getReplyCode() == industrial::simple_message::ReplyTypes::SUCCESS ?
          (int)industrial_msgs::ServiceReturnCode::SUCCESS : (int)industrial_msgs::ServiceReturnCode::FAILURE;

  return true; // always return true.  To distinguish between call-failed and service-unavailable.
}

bool IOService::readDigitalInputCB(i5_msgs::ReadDigitalInput::Request &req, i5_msgs::ReadDigitalInput::Response &res)
{

  ReadSingleIOMessage io_msg;

  industrial::simple_message::SimpleMessage msg, reply;

  io_msg.cmd_.setType(req.type);
  io_msg.cmd_.setAddress(req.index);
  io_msg.cmd_.setValue(-1);  // default invalid value

  // fill to msg
  industrial::byte_array::ByteArray data;
  data.load(io_msg);
  msg.init(io_msg.getMessageType(), industrial::simple_message::CommTypes::SERVICE_REQUEST,
           industrial::simple_message::ReplyTypes::INVALID, data);

  this->connection_->sendAndReceiveMsg(msg, reply);

//    if (!io_msg.init(reply))
//    {
//    	res.value = -1;
//    }
  //TODO
//    res.code.val = reply.getReplyCode() ==
//                        industrial::simple_message::ReplyTypes::SUCCESS ?
//                                        (int)industrial_msgs::ServiceReturnCode::SUCCESS : (int)industrial_msgs::ServiceReturnCode::FAILURE;

  if (reply.getReplyCode() == industrial::simple_message::ReplyTypes::SUCCESS)
  {
    io_msg.init(reply);
    res.code.val = (int)industrial_msgs::ServiceReturnCode::SUCCESS;
    res.value = io_msg.cmd_.getValue();
  }
  else
  {
    res.code.val = (int)industrial_msgs::ServiceReturnCode::FAILURE;
  }

  return true; // always return true.  To distinguish between call-failed and service-unavailable.
}

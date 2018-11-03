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

#include "i5_driver/system_service.h"

#include <industrial_utils/param_utils.h>


using namespace i5::simple_message::system_message;
using namespace industrial_utils::param;
using industrial::simple_message::SimpleMessage;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

#define ROS_ERROR_RETURN(rtn,...) do {ROS_ERROR(__VA_ARGS__); return(rtn);} while(0)

using namespace industrial_robot_client::system_service;

bool SystemService::init(std::string default_ip, int default_port)
{

  this->srv_execute_application_cmd_ = this->node_.advertiseService("execute_application_cmd",
                                                                    &SystemService::executeApplicationCmdCB, this);
  this->srv_get_application_cmd_help_info_ = this->node_.advertiseService("print_app_cmd_info",
                                                                          &SystemService::printHelpInfo, this);

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
  ROS_INFO("i5 system_service Interface connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);

  connection_ = &default_tcp_connection_;

  return connection_->makeConnect();
}

bool SystemService::executeApplicationCmdCB(i5_msgs::ExecuteApplicationCmd::Request &req,
                                            i5_msgs::ExecuteApplicationCmd::Response &res)
{

  ApplicationCmdMessage cmd_msg;

  industrial::simple_message::SimpleMessage msg, reply;

  cmd_msg.cmd_.setCommand(req.command);

  for (unsigned int i = 0; i < req.param_data_i.size(); ++i)
  {
    cmd_msg.cmd_.setIntData(i, req.param_data_i[i]);
  }

  for (unsigned int i = 0; i < req.param_data_r.size(); ++i)
  {
    cmd_msg.cmd_.setRealData(i, req.param_data_r[i]);
  }

//	cmd_msg.setIntData(req.)

  // fill to msg
  industrial::byte_array::ByteArray data;
  data.load(cmd_msg);
  msg.init(cmd_msg.getMessageType(), industrial::simple_message::CommTypes::SERVICE_REQUEST,
           industrial::simple_message::ReplyTypes::INVALID, data);

  this->connection_->sendAndReceiveMsg(msg, reply);

  if (reply.getReplyCode() == industrial::simple_message::ReplyTypes::SUCCESS)
  {
    cmd_msg.init(reply);
    res.code.val = (int)industrial_msgs::ServiceReturnCode::SUCCESS;
//    for (unsigned int i = 0; i < req.param_data_i.size(); ++i)
//    {
//      cmd_msg.setIntData(i, req.param_data_i[i]);
//    }
//
//    for (unsigned int i = 0; i < req.param_data_r.size(); ++i)
//    {
//      cmd_msg.setRealData(i, req.param_data_r[i]);
//    }
  }
  else
  {
    res.code.val = (int)industrial_msgs::ServiceReturnCode::FAILURE;
  }
  return true; // always return true.  To distinguish between call-failed and service-unavailable.
}

bool SystemService::printHelpInfo(i5_msgs::GetApplicationCmdHelpInfo::Request &req,
                                             i5_msgs::GetApplicationCmdHelpInfo::Response &res)
{
  ROS_INFO("i5Robot general_cmd_service help info: 0 servoOn, 1 enableRobot, 2 disableRobot, 3 switchMode(PLAYBACK)");
  res.help_info = std::string("set_general_cmd param: 0 servoOn, 1 enableRobot, 2 disableRobot, 3 switchMode(PLAYBACK)");
  return true;
}

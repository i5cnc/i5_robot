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


#include "i5_simple_message/messages/i5_application_cmd_message.h"

#include <simple_message/byte_array.h>
#include <simple_message/log_wrapper.h>


using industrial::byte_array::ByteArray;
using industrial::simple_message::SimpleMessage;
using i5::simple_message::system::ApplicationCmd;

namespace i5
{
namespace simple_message
{
namespace system_message
{

ApplicationCmdMessage::ApplicationCmdMessage(void)
{
  this->init();
}

ApplicationCmdMessage::~ApplicationCmdMessage(void)
{
}

bool ApplicationCmdMessage::init(SimpleMessage & msg)
{
  ByteArray data = msg.getData();
  this->init();

  if (!data.unload(this->cmd_))
  {
    LOG_ERROR("Failed to unload GeneralCMDMessage data");
    return false;
  }

  return true;
}

void ApplicationCmdMessage::init(ApplicationCmd & cmd)
{
  this->init();
  this->cmd_.copyFrom(cmd);
}

void ApplicationCmdMessage::init()
{
  this->setMessageType(I5MsgTypes::I5_APPLICATION_CMD);
  this->cmd_.init();
}

bool ApplicationCmdMessage::load(ByteArray *buffer)
{
  LOG_COMM("Executing GeneralCMDMessage message load");
  if (!buffer->load(this->cmd_))
  {
    LOG_ERROR("Failed to load GeneralCMDMessage message");
    return false;
  }

  return true;
}

bool ApplicationCmdMessage::unload(ByteArray *buffer)
{
  LOG_COMM("Executing GeneralCMDMessage message unload");

  if (!buffer->unload(this->cmd_))
  {
    LOG_ERROR("Failed to unload GeneralCMDMessage message");
    return false;
  }

  return true;
}

}  // namespace system_message
}  // namespace simple_message
}  // namespace i5


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

#include "i5_simple_message/i5_application_cmd.h"

using namespace i5;
using namespace simple_message;
using namespace i5::simple_message::system;

using namespace industrial::shared_types;

ApplicationCmd::ApplicationCmd(void)
{
  this->init();
}
ApplicationCmd::~ApplicationCmd(void)
{
}

void ApplicationCmd::init()
{
  // TODO: is '0' a good initial value?
  this->init(0);
}

void ApplicationCmd::init(shared_int command)
{
  this->setCommand(command);
}

void ApplicationCmd::copyFrom(ApplicationCmd &src)
{
  this->setCommand(src.getCommand());
  for(unsigned int i = 0; i < 10; ++i)
  {
    this->setRealData(i,src.getRealData(i));
    this->setIntData(i,src.getIntData(i));
  }
}

bool ApplicationCmd::operator==(ApplicationCmd &rhs)
{
  bool rslt = this->command_ == rhs.command_;

  return rslt;
}

bool ApplicationCmd::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing ApplicationCmd load");

  if (!buffer->load(this->command_))
  {
    LOG_ERROR("Failed to load GeneralCMD address");
    return false;
  }


  for (size_t i = 0; i < SYSTEM_CMD_MAX_DATA_CNT; ++i)
  {
    shared_real value = this->getRealData(i);
    if (!buffer->load(value))
    {
      LOG_ERROR("Failed to load ApplicationCmd real data element %d from data[%d]", static_cast<int>(i), buffer->getBufferSize());
      return false;
    }
  }

  for (size_t i = 0; i < SYSTEM_CMD_MAX_DATA_CNT; ++i)
  {
    shared_int value = this->getIntData(i);
    if (!buffer->load(value))
    {
      LOG_ERROR("Failed to load ApplicationCmd int data element %d from data[%d]", static_cast<int>(i), buffer->getBufferSize());
      return false;
    }
  }

  LOG_COMM("ApplicationCmd data successfully loaded");
  return true;
}

bool ApplicationCmd::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing ApplicationCmd unload");


  for (int i = SYSTEM_CMD_MAX_DATA_CNT - 1; i >= 0; --i)
  {
    shared_int value;
    if (!buffer->unload(value))
    {
      LOG_ERROR("Failed to unload message data element: %d from data[%d]", static_cast<int>(i), buffer->getBufferSize());
      return false;
    }
    this->setIntData(i, value);
  }

  for (int i = SYSTEM_CMD_MAX_DATA_CNT - 1; i >= 0; --i)
  {
    shared_real value;
    if (!buffer->unload(value))
    {
      LOG_ERROR("Failed to unload message data element: %d from data[%d]", static_cast<int>(i), buffer->getBufferSize());
      return false;
    }
    this->setRealData(i, value);
  }

  if (!buffer->unload(this->command_))
  {
    LOG_ERROR("Failed to unload GeneralCMD value");
    return false;
  }

  LOG_COMM("GeneralCMD data successfully unloaded");
  return true;
}



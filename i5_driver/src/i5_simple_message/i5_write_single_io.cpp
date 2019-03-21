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

#include "i5_simple_message/i5_write_single_io.h"

using namespace i5;
using namespace simple_message;
using namespace io;

using namespace industrial::shared_types;

WriteSingleIO::WriteSingleIO(void)
{
  this->init();
}
WriteSingleIO::~WriteSingleIO(void)
{
}

void WriteSingleIO::init()
{
  // TODO: is '0' a good initial value?
  this->init(-1, 0, 0);
}

void WriteSingleIO::init(shared_int type, shared_int address, shared_int value)
{
  this->setType(type);
  this->setAddress(address);
  this->setValue(value);
}

void WriteSingleIO::copyFrom(WriteSingleIO &src)
{
  this->setType(src.getType());
  this->setAddress(src.getAddress());
  this->setValue(src.getValue());
}

bool WriteSingleIO::operator==(WriteSingleIO &rhs)
{
  bool rslt = this->type_ == rhs.type_ &&
              this->address_ == rhs.address_ &&
              this->value_ == rhs.value_;

  return rslt;
}

bool WriteSingleIO::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing WriteSingleIO command load");

  if (!buffer->load(this->type_))
  {
    LOG_ERROR("Failed to load WriteSingleIO type");
    return false;
  }

  if (!buffer->load(this->address_))
  {
    LOG_ERROR("Failed to load WriteSingleIO address");
    return false;
  }

  if (!buffer->load(this->value_))
  {
    LOG_ERROR("Failed to load WriteSingleIO value");
    return false;
  }

  LOG_COMM("WriteSingleIO data successfully loaded");
  return true;
}

bool WriteSingleIO::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing WriteSingleIO command unload");

  if (!buffer->unload(this->value_))
  {
    LOG_ERROR("Failed to unload WriteSingleIO value");
    return false;
  }

  if (!buffer->unload(this->address_))
  {
    LOG_ERROR("Failed to unload WriteSingleIO address");
    return false;
  }

  if (!buffer->unload(this->type_))
  {
    LOG_ERROR("Failed to unload WriteSingleIO type");
    return false;
  }

  LOG_COMM("WriteSingleIO data successfully unloaded");
  return true;
}

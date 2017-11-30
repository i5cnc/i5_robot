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

#ifndef I5_SIMPLE_MESSAGE_H_
#define I5_SIMPLE_MESSAGE_H_

#include <simple_message/simple_message.h>

namespace i5
{
namespace simple_message
{

// i5 msg type define
namespace I5MsgTypes
{

enum I5MsgType
{
  // i5 uses 2900 - 2999
  I5_MSG_BEGIN = 2900,
  I5_APPLICATION_CMD = 2910,

  I5_READ_SINGLE_IO = 2930,
  I5_WRITE_SINGLE_IO = 2931,


};
}
typedef I5MsgTypes::I5MsgType I5MsgType;

}  // ns simple_message
}  // ns i5

#endif /* SIMPLE_MESSAGE_INCLUDE_SIMPLE_MESSAGE_I5ROBOT_SIMPLE_MESSAGE_H_ */

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

#ifndef I5_APPLICATION_CMD_MESSAGE_H_
#define I5_APPLICATION_CMD_MESSAGE_H_

#include <simple_message/typed_message.h>
#include <simple_message/shared_types.h>
#include "i5_simple_message/i5_application_cmd.h"
#include "i5_simple_message/i5_simple_message.h"



namespace i5
{
namespace simple_message
{
namespace system_message
{

/**
 * \brief Class encapsulated i5 application cmd message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 * This message simply wraps the following data type:
 *   i5::simple_message::system::ApplicationCmd
 * The data portion of this typed message matches ApplicationCmd exactly.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class ApplicationCmdMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  ApplicationCmdMessage(void);
  /**
   * \brief Destructor
   *
   */
  virtual ~ApplicationCmdMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a application cmd structure
   *
   * \param cmd read single io data structure
   *
   */
  void init(i5::simple_message::system::ApplicationCmd & cmd);

  /**
   * \brief Initializes a new message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return this->cmd_.byteLength();
  }

  i5::simple_message::system::ApplicationCmd cmd_;

private:
};
}  // namespace system_message
}  // namespace simple_message
}  // namespace i5



#endif /* I5_APPLICATION_CMD_MESSAGE_H_ */

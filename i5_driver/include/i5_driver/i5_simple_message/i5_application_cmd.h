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

#ifndef I5_APPLICATION_CMD_H_
#define I5_APPLICATION_CMD_H_

#include <simple_message/simple_serialize.h>
#include <simple_message/shared_types.h>
#include <simple_message/log_wrapper.h>

#ifdef SYSTEM_CMD_MAX_DATA_CNT
#undef SYSTEM_CMD_MAX_DATA_CNT
#endif

#define SYSTEM_CMD_MAX_DATA_CNT 10

namespace i5
{
namespace simple_message
{
namespace system
{

/**
 * \brief Class encapsulated all the application data. i5 specific interface
 * to send command to the controller.
 *
 * The byte representation of a application command is as follows
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   command             (industrial::shared_types::shared_int)    4  bytes
 *   data_r              (industrial::shared_types::shared_real)   40  bytes
 *   data_i              (industrial::shared_types::shared_real)   40  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class ApplicationCmd : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  ApplicationCmd(void);
  /**
   * \brief Destructor
   *
   */
  ~ApplicationCmd(void);

  /**
   * \brief Initializes a empty system command
   *
   */
  void init();

  /**
   * \brief Initializes a complete system command
   *
   */
  void init(industrial::shared_types::shared_int value);

  /**
   * \brief Sets command
   *
   * \param value command.
   */
  void setCommand(industrial::shared_types::shared_int value)
  {
    this->command_ = value;
  }

  /**
   * \brief Returns the value of the IO element
   *
   * \return value
   */
  industrial::shared_types::shared_int getCommand()
  {
    return this->command_;
  }

  /**
   * \brief Sets real data
   *
   * \param idx  index to set
   * \param val data value
   */
  void setRealData(size_t idx, industrial::shared_types::shared_real val)
  {
    if (idx < SYSTEM_CMD_MAX_DATA_CNT)
      this->data_r_[idx] = val;
    else
      LOG_ERROR("Real data index out-of-range (%d >= %d)",
          static_cast<int>(idx), static_cast<int>(MAX_DATA_CNT));
  }

  /**
   * \brief Returns real data
   *
   * \param idx data-index to get
   * \return data value
   */
  industrial::shared_types::shared_real getRealData(size_t idx)
  {
    if (idx < SYSTEM_CMD_MAX_DATA_CNT)
    {
      return this->data_r_[idx];
    }
    else
    {
      LOG_ERROR("Real data index out-of-range (%d >= %d)",
          static_cast<int>(idx), static_cast<int>(MAX_DATA_CNT));

      return 0;
    }
  }



  /**
   * \brief Sets int data
   *
   * \param idx  index to set
   * \param val data value
   */
  void setIntData(size_t idx, industrial::shared_types::shared_int val)
  {
    if (idx < SYSTEM_CMD_MAX_DATA_CNT)
      this->data_i_[idx] = val;
    else
      LOG_ERROR("Int data index out-of-range (%d >= %d)",
          static_cast<int>(idx), static_cast<int>(MAX_DATA_CNT));
  }

  /**
   * \brief Returns int data
   *
   * \param idx data-index to get
   * \return data value
   */
  industrial::shared_types::shared_int getIntData(size_t idx)
  {
    if (idx < SYSTEM_CMD_MAX_DATA_CNT)
    {
      return this->data_i_[idx];
    }
    else
    {
      LOG_ERROR("Int data index out-of-range (%d >= %d)",
          static_cast<int>(idx), static_cast<int>(MAX_DATA_CNT));

      return 0;
    }
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(ApplicationCmd &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(ApplicationCmd &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) + SYSTEM_CMD_MAX_DATA_CNT * sizeof(industrial::shared_types::shared_real)
        + SYSTEM_CMD_MAX_DATA_CNT * sizeof(industrial::shared_types::shared_int);
  }

private:

//  static size_t MAX_DATA_CNT = 10;
  /**
   * \brief command value.
   */
  industrial::shared_types::shared_int command_;
  /**
   * \brief reserved real data for parameter.
   */
  industrial::shared_types::shared_real data_r_[SYSTEM_CMD_MAX_DATA_CNT];
  /**
   * \brief reserved int data for parameter.
   */
  industrial::shared_types::shared_int data_i_[SYSTEM_CMD_MAX_DATA_CNT];

};

}  // namespace cmd
}  // namespace simple_message
}  // namespace i5

#endif /* ROS_CONNECTOR_INCLUDE_I5ROBOT_SIMPLE_MESSAGE_I5ROBOT_GENERAL_CMD_H_ */

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

#ifndef I5_READ_SINGLE_IO_H_
#define I5_READ_SINGLE_IO_H_


#include <simple_message/simple_serialize.h>
#include <simple_message/shared_types.h>
#include <simple_message/log_wrapper.h>

namespace i5
{
namespace simple_message
{
namespace io
{


    class ReadSingleIO : public industrial::simple_serialize::SimpleSerialize
    {
    public:
      /**
       * \brief Default constructor
       *
       * This method creates empty data.
       *
       */
        ReadSingleIO(void);
      /**
       * \brief Destructor
       *
       */
      ~ReadSingleIO(void);

      /**
       * \brief Initializes a empty read single io command
       *
       */
      void init();

      /**
       * \brief Initializes a complete read single io command
       *
       */
      void init(industrial::shared_types::shared_int address,
        industrial::shared_types::shared_int value);

      /**
       * \brief Sets address
       *
       * \param address Controller address of the targeted IO element.
       */
      void setAddress(industrial::shared_types::shared_int address)
      {
        this->address_ = address;
      }

      /**
       * \brief Sets value
       *
       * \param value Controller value of the targeted IO element.
       */
      void setValue(industrial::shared_types::shared_int value)
      {
        this->value_ = value;
      }

      /**
       * \brief Returns the address of the IO element
       *
       * \return address
       */
      industrial::shared_types::shared_int getAddress()
      {
        return this->address_;
      }

      /**
       * \brief Returns the value of the IO element
       *
       * \return value
       */
      industrial::shared_types::shared_int getValue()
      {
        return this->value_;
      }

      /**
       * \brief Copies the passed in value
       *
       * \param src (value to copy)
       */
      void copyFrom(ReadSingleIO &src);

      /**
       * \brief == operator implementation
       *
       * \return true if equal
       */
      bool operator==(ReadSingleIO &rhs);

      // Overrides - SimpleSerialize
      bool load(industrial::byte_array::ByteArray *buffer);
      bool unload(industrial::byte_array::ByteArray *buffer);
      unsigned int byteLength()
      {
        return 2 * sizeof(industrial::shared_types::shared_int);
      }

    private:
      /**
       * \brief Address of IO element.
       */
      industrial::shared_types::shared_int address_;

      /**
       * \brief Value of IO element.
       */
      industrial::shared_types::shared_int value_;

    };

}  // namespace io
}  // namespace simple_message
}  // namespace i5


#endif /* I5_READ_SINGLE_IO_H_ */

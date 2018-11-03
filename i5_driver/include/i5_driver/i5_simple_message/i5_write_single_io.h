/*
 * i5robot_write_single_io.h
 *
 *  Created on: Aug 4, 2017
 *      Author: i5
 */

#ifndef I5_WRITE_SINGLE_IO_H_
#define I5_WRITE_SINGLE_IO_H_

#include <simple_message/simple_serialize.h>
#include <simple_message/shared_types.h>
#include <simple_message/log_wrapper.h>

namespace i5
{
namespace simple_message
{
namespace io
{



    class WriteSingleIO : public industrial::simple_serialize::SimpleSerialize
    {
    public:
      /**
       * \brief Default constructor
       *
       * This method creates empty data.
       *
       */
      WriteSingleIO(void);
      /**
       * \brief Destructor
       *
       */
      ~WriteSingleIO(void);

      /**
       * \brief Initializes a empty write single io command
       *
       */
      void init();

      /**
       * \brief Initializes a complete write single io command
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
      void copyFrom(WriteSingleIO &src);

      /**
       * \brief == operator implementation
       *
       * \return true if equal
       */
      bool operator==(WriteSingleIO &rhs);

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
#endif /* I5_WRITE_SINGLE_IO_H_ */

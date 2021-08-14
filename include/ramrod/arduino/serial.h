#ifndef RAMROD_ARDUINO_SERIAL_H
#define RAMROD_ARDUINO_SERIAL_H

#include <boost/asio/io_service.hpp>                         // for io_service
#include <boost/asio/serial_port.hpp>                        // for serial_port
#include <boost/chrono/duration.hpp>                         // for millisec...
#include <boost/date_time/posix_time/posix_time_config.hpp>  // for time_dur...
#include <boost/system/error_code.hpp>                       // for error_code
#include <iosfwd>                                            // for size_t
#include <string>                                            // for string
#include "ramrod/arduino/constants.h"                        // for SERIAL_8N1


namespace ramrod {
  namespace arduino {
    class serial
    {
    public:
      serial();
      serial(const std::string &port_name, const unsigned int rate, const int config = SERIAL_8N1);

      ~serial();
      /**
       * @brief Get the number of bytes (characters) available for reading from
       *        the serial port.
       *
       * This is data that’s already arrived and stored in the serial receive
       * buffer (which holds 64 bytes).
       *
       * @return The number of bytes available to read.
       */
      // TODO: complete available
      int available();
      /**
       * @brief Get the number of bytes (characters) available for writing in the
       *        serial buffer without blocking the write operation.
       *
       * @return The number of bytes available to write.
       */
      // TODO: complete available for write
      int available_for_write();
      /**
       * @brief Opening serial communication
       *
       * Sets the port used for communcation with arduino and the data rate in bits per
       * second (baud) for serial data transmission. For communicating with Serial Monitor,
       * make sure to use one of the baud rates listed in the menu at the bottom right
       * corner of its screen. You can, however, specify other rates - for example, to
       * communicate over pins 0 and 1 with a component that requires a particular baud rate.
       *
       * An optional third argument configures the data, parity, and stop bits.
       * The default is 8 data bits, no parity, one stop bit.
       *
       * @param port_name Defines the port name where arduino is connected
       * @param rate      In bits per second (baud).
       * @param config    Sets data, parity, and stop bits. Valid values are:
       *                      SERIAL_5N1
       *                      SERIAL_6N1
       *                      SERIAL_7N1
       *                      SERIAL_8N1 (the default)
       *                      SERIAL_5N2
       *                      SERIAL_6N2
       *                      SERIAL_7N2
       *                      SERIAL_8N2
       *                      SERIAL_5E1: even parity
       *                      SERIAL_6E1
       *                      SERIAL_7E1
       *                      SERIAL_8E1
       *                      SERIAL_5E2
       *                      SERIAL_6E2
       *                      SERIAL_7E2
       *                      SERIAL_8E2
       *                      SERIAL_5O1: odd parity
       *                      SERIAL_6O1
       *                      SERIAL_7O1
       *                      SERIAL_8O1
       *                      SERIAL_5O2
       *                      SERIAL_6O2
       *                      SERIAL_7O2
       *                      SERIAL_8O2
       *
       * @return `false` if the connection failed or if was not possible to set the baut rate
       */
      bool begin(const std::string &port_name, const unsigned int rate,
                 const int config = SERIAL_8N1);
      /**
       * @brief Disables serial communication, allowing the RX and TX pins to be used for
       *        general input and output. To re-enable serial communication, call `begin()`.
       *
       * @return `false` if there was an error when disabling serial communication
       */
      bool end();
      /**
       * @brief Reads data from the serial buffer until the target is found.
       *
       * @param target The string to search for.
       *
       * @return `true` if target is found, `false` if it times out.
       */
      bool find(const char target);
      /**
       * @brief Reads data from the serial buffer until the target is found.
       *
       * @param target The string to search for.
       * @param length Length of the target.
       *
       * @return `true` if target is found, `false` if it times out.
       */
      bool find(const char target, const std::size_t length);
      /**
       * @brief Reads data from the serial buffer until a target string of given
       *        length or terminator string is found.
       *
       * @param target   The string to search for.
       * @param terminal The terminal string in the search.
       *
       * @return `true` if the target string is found, `false` if it times out.
       */
      bool find_until(const char target, const char terminal);
      /**
       * @brief Waits for the transmission of outgoing serial data to complete.
       */
      void flush();
      // TODO: complete parsing
      float parse_float();
      float parse_float(const lookahead_mode lookahead);
      float parse_float(const lookahead_mode lookahead, const char ignore);
      // TODO: complete parsing
      long parse_int();
      long parse_int(const lookahead_mode lookahead);
      long parse_int(const lookahead_mode lookahead, const char ignore);
      /**
       * @brief Peeking the next character in the reading buffer
       *
       * Returns the next byte (character) of incoming serial data without removing it
       * from the internal serial buffer. That is, successive calls to `peek()` will return
       * the same character, as will the next call to `read()`.
       *
       * @return The first byte of incoming serial data available (or -1 if no data is available).
       */
      int peek();
      // TODO: complete printing
      bool print();
      // TODO: complete printing line
      bool println();
      /**
       * @brief Reads incoming serial data.
       *
       * @return The first byte of incoming serial data available (or -1 if no data is available)
       */
      int read();
      /**
       * @brief Read a certain quantity of bytes
       *
       * Reads characters from the serial port into a buffer. The function terminates if
       * the determined length has been read, or it times out (see `set_timeout()`).
       *
       * @param buffer The buffer to store the bytes in.
       * @param length The number of bytes to read.
       *
       * @return The number of characters placed in the buffer. A 0 means no valid data was found.
       */
      std::size_t read_bytes(char *buffer, const std::size_t length);
      /**
       * @brief Read a certain quantity of bytes or until certain character is reached
       *
       * Reads characters from the serial buffer into an array. The function terminates
       * (checks being done in this order) if the determined length has been read, if it
       * times out (see `set_timeout()`), or if the terminator character is detected
       * (in which case the function returns the characters up to the last character before
       * the supplied terminator). The terminator itself is not returned in the buffer.
       *
       * The terminator character is discarded from the serial buffer, unless the number
       * of characters read and copied into the buffer equals length.
       *
       * @param character The character to search for
       * @param buffer    The buffer to store the bytes in.
       * @param length    The number of bytes to read.
       *
       * @return The number of characters read into the buffer. A 0 means that the length
       *         parameter <= 0, a time out occurred before any other input, or a termination
       *         character was found before any other input.
       */
      std::size_t read_bytes_until(const char character, char *buffer, const std::size_t length);
      /**
       * @brief Reading a string
       *
       * Reads characters from the serial buffer into a String. The function terminates if
       * it times out (see `set_timeout()`).
       *
       * @return A String read from the serial buffer
       */
      std::string read_string();
      /**
       * @brief Reading a string up to a termination character
       *
       * Reads characters from the serial buffer into a String. The function terminates if
       * it times out (see `set_timeout()`).
       *
       * The terminator character is discarded from the serial buffer.
       *
       * @param terminator The character to search for.
       *
       * @return The entire String read from the serial buffer, up to the terminator character
       */
      std::string read_string_until(const char terminator);
      /**
       * @brief Sets the maximum milliseconds to wait for serial data. It defaults to
       *        1000 milliseconds.
       *
       * Serial functions that use the timeout value set via `set_timeout()`:
       *    1. find()
       *    2. find_until()
       *    3. parse_int()
       *    4. parse_float()
       *    5. read_bytes()
       *    6. read_bytes_until()
       *    7. read_string()
       *    8. read_string_until()
       *
       * @param milliseconds Number of milliseconds to wait for serial data
       */
      void set_timeout(const unsigned int milliseconds = 1000);
      /**
       * @brief Writing a single byte
       *
       * Writes binary data to the serial port. This data is sent as a byte or series of bytes;
       * to send the characters representing the digits of a number use the `print()`
       * function instead.
       *
       * @param value A value to send as a single byte.
       *
       * @return The number of bytes written, though reading that number is optional.
       */
      std::size_t write(const char value);
      /**
       * @brief Writing a string
       *
       * Writes binary data to the serial port. This data is sent as a byte or series of bytes;
       * to send the characters representing the digits of a number use the `print()`
       * function instead.
       *
       * @param string A string to send as a series of bytes.
       *
       * @return The number of bytes written, though reading that number is optional.
       */
      std::size_t write(const std::string &string);
      /**
       * @brief Writing a buffer of bytes
       *
       * Writes binary data to the serial port. This data is sent as a byte or series of bytes;
       * to send the characters representing the digits of a number use the `print()`
       * function instead.
       *
       * @param buffer An array to send as a series of bytes.
       * @param length The number of bytes to be sent from the array.
       *
       * @return The number of bytes written, though reading that number is optional.
       */
      std::size_t write(const char *buffer, const std::size_t length);
      /**
       * @brief Indicates if the specified Serial port is ready.
       *
       * @return Returns `true` if the specified serial port is available.
       */
      operator bool();

      bool change_buffer_max_size(const std::size_t new_buffer_size = 64);
      // TODO: set action_wait_

    private:
      void concurrent_read();
      bool set_config(const unsigned int rate, const int config);

      boost::asio::io_service io_;
      boost::asio::serial_port port_;
      boost::posix_time::time_duration timeout_;
      boost::system::error_code error_;

      std::size_t buffer_size_;

      char *in_buffer_;
      std::size_t in_position_;
      std::size_t in_size_;
      bool reading_;
      bool exit_read_;

      char *out_buffer_;
      std::size_t out_position_;
      std::size_t out_size_;
      bool out_finished_;

      boost::chrono::milliseconds action_wait_;
      const boost::chrono::milliseconds one_;

    };
  } // namespace: arduino 
} // namespace: ramrod

#endif // RAMRDO_ARDUINO_SERIAL_H

#include "ramrod/arduino/serial.h"

#include <boost/asio/buffer.hpp>                               // for mutabl...
#include <boost/asio/impl/io_context.hpp>                      // for io_con...
#include <boost/asio/impl/io_context.ipp>                      // for io_con...
#include <boost/asio/io_context.hpp>                           // for io_con...
#include <boost/asio/placeholders.hpp>                         // for error
#include <boost/asio/read.hpp>                                 // for async_...
#include <boost/asio/write.hpp>                                // for async_...
#include <boost/bind.hpp>                                      // for bind_t
#include <boost/date_time/posix_time/posix_time_duration.hpp>  // for hours

namespace ramrod {
  namespace arduino {

    serial::serial() :
      io_(),
      port_(io_),
      timer_(io_),
      timeout_(boost::posix_time::milliseconds(1000)),
      error_(),
      baud_rate_(9600),
      config_{SERIAL_8N1},
      port_name_(),
      bytes_transfered_{0}
    {
    }

    serial::serial(const std::string &port_name, const unsigned int rate, const int config) :
      io_(),
      port_(io_, port_name),
      timer_(io_),
      timeout_(boost::posix_time::milliseconds(1000)),
      error_(),
      baud_rate_(rate),
      config_{config},
      port_name_(port_name),
      bytes_transfered_{0}
    {
      set_config(rate, config);
    }

    serial::~serial(){
      end();
    }

    bool serial::begin(const std::string &port_name, const unsigned int rate, const int config){
      if(port_.is_open()) port_.close(error_);

      port_.open(port_name_ = port_name);

      return port_.is_open() && set_config(rate, config);
    }

    bool serial::end(){
      if(port_.is_open())
        return !port_.close(error_).failed();
      return true;
    }

    int serial::read(){
      char buffer[2];
      if(receive(buffer, 1) > 0)
        return int(buffer[0]);
      return -1;
    }

    std::size_t serial::read_bytes(char *buffer, const std::size_t length){
      if(!port_.is_open() || length == 0 || buffer == nullptr) return 0;

      return receive(buffer, length);
    }

    std::size_t serial::read_bytes_until(char character, char *buffer, const std::size_t length){
      if(port_.is_open() || length == 0 || buffer == nullptr) return 0;

      buffer_ = buffer;
      terminator_ = character;

      // After a timeout & cancel it seems we need
      // to do a reset for subsequent reads to work.
      io_.reset();

      // Asynchronously read `buffer_size` characters.
      boost::asio::async_read(port_, boost::asio::buffer(buffer, length),
                              boost::bind(&serial::completion, this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred),
                              boost::bind(&serial::completed, this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));

      // Setup a deadline time to implement our timeout.
      timer_.expires_from_now(timeout_);
      timer_.async_wait(boost::bind(&serial::time_out, this, boost::asio::placeholders::error));

      // This will block until a character is read or until it is cancelled.
      io_.run();

      return bytes_transfered_;
    }

    std::string serial::read_string(){
      std::string result;

      if(port_.is_open()) return result;

      string_ = &result;
      terminator_ = '\0';

      // After a timeout & cancel it seems we need
      // to do a reset for subsequent reads to work.
      io_.reset();

      // Asynchronously read `buffer_size` characters.
      boost::asio::async_read(port_, boost::asio::dynamic_string_buffer(result),
                              boost::bind(&serial::completion_string, this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred),
                              boost::bind(&serial::completed, this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));

      // Setup a deadline time to implement our timeout.
      timer_.expires_from_now(timeout_);
      timer_.async_wait(boost::bind(&serial::time_out, this, boost::asio::placeholders::error));

      // This will block until a character is read or until it is cancelled.
      io_.run();

      return result;
    }

    void serial::set_timeout(const unsigned int milliseconds){
      if(milliseconds == 0) timeout_ = boost::posix_time::hours(100000);
      else timeout_ = boost::posix_time::milliseconds(milliseconds);
    }

    std::size_t serial::write(char value){
      if(port_.is_open()) return 0;

      return send(&value, 1);
    }

    std::size_t serial::write(const std::string &string){
      if(port_.is_open() || string.size() == 0) return 0;

      return send(string.c_str(), string.size());
    }

    std::size_t serial::write(const char *buffer, const std::size_t length){
      if(port_.is_open() || length == 0 || buffer == nullptr) return 0;

      return send(buffer, length);
    }

    ramrod::arduino::serial::operator bool(){
      return port_.is_open();
    }

    // ::::::::::::::::::::::::::::::::::: PRIVATE FUNCTIONS :::::::::::::::::::::::::::::::::::::

    std::size_t serial::completion(const boost::system::error_code &error,
                                   const std::size_t bytes_transferred){
      if(*(buffer_ + bytes_transferred - 1) == terminator_ || error.failed())
        return 0;
      return 1;
    }

    std::size_t serial::completion_string(const boost::system::error_code &error,
                                          const std::size_t bytes_transferred){
      if(*(string_->data() + bytes_transferred - 1) == terminator_ || error.failed())
        return 0;
      return 1;
    }

    void serial::completed(const boost::system::error_code &error,
                           const std::size_t bytes_transferred){
      bytes_transfered_ = error.failed() ? 0 : bytes_transferred;

      // Read has finished, so cancel the timer.
      timer_.cancel();
    }

    std::size_t serial::receive(char *buffer, const std::size_t length){
      // After a timeout & cancel it seems we need
      // to do a reset for subsequent reads to work.
      io_.reset();

      // Asynchronously read `buffer_size` characters.
      boost::asio::async_read(port_, boost::asio::buffer(buffer, length),
                              boost::bind(&serial::completed, this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));

      // Setup a deadline time to implement our timeout.
      timer_.expires_from_now(timeout_);
      timer_.async_wait(boost::bind(&serial::time_out, this, boost::asio::placeholders::error));

      // This will block until a character is read or until it is cancelled.
      io_.run();

      return bytes_transfered_;
    }

    std::size_t serial::send(const char *buffer, const std::size_t length){
      // After a timeout & cancel it seems we need
      // to do a reset for subsequent reads to work.
      io_.reset();

      // Asynchronously read `buffer_size` characters.
      boost::asio::async_write(port_, boost::asio::buffer(buffer, length),
                               boost::bind(&serial::completed, this,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));

      // Setup a deadline time to implement our timeout.
      timer_.expires_from_now(timeout_);
      timer_.async_wait(boost::bind(&serial::time_out, this, boost::asio::placeholders::error));

      // This will block until a character is writen or until it is cancelled.
      io_.run();

      return bytes_transfered_;
    }

    bool serial::set_config(const unsigned int rate, const int config){
      // Setting baud rate
      bool failed{port_.set_option(boost::asio::serial_port::baud_rate(baud_rate_ = rate),
                                   error_).failed()};

      if(failed) return false;

      // Setting the stop bits
      if((config & 0b00000001) > 0)
        failed = failed || port_.set_option(boost::asio::serial_port_base::stop_bits(
                                              boost::asio::serial_port_base::stop_bits::type::two),
                                            error_).failed();
      else
        failed = failed || port_.set_option(boost::asio::serial_port_base::stop_bits(
                                              boost::asio::serial_port_base::stop_bits::type::one),
                                            error_).failed();

      if(failed) return false;

      // Setting the parity
      if((config & 0b00000010) > 0)
        failed = failed || port_.set_option(boost::asio::serial_port_base::parity(
                                              boost::asio::serial_port_base::parity::type::none),
                                            error_).failed();
      else if((config & 0b00000100) > 0)
        failed = failed || port_.set_option(boost::asio::serial_port_base::parity(
                                              boost::asio::serial_port_base::parity::type::even),
                                            error_).failed();
      else if((config & 0b00001000) > 0)
        failed = failed || port_.set_option(boost::asio::serial_port_base::parity(
                                              boost::asio::serial_port_base::parity::type::odd),
                                            error_).failed();

      if(failed) return false;

      // Setting the character size
      if((config & 0b00010000) > 0)
        failed = failed || port_.set_option(boost::asio::serial_port_base::character_size(5),
                                            error_).failed();
      else if((config & 0b00100000) > 0)
        failed = failed || port_.set_option(boost::asio::serial_port_base::character_size(6),
                                            error_).failed();
      else if((config & 0b01000000) > 0)
        failed = failed || port_.set_option(boost::asio::serial_port_base::character_size(7),
                                            error_).failed();
      else if((config & 0b10000000) > 0)
        failed = failed || port_.set_option(boost::asio::serial_port_base::character_size(8),
                                            error_).failed();

      return !failed;
    }

    void serial::time_out(const boost::system::error_code &error){
      // Was the timeout was cancelled?
      if(error){
        // yes
        bytes_transfered_ = 0;
        return;
      }

      // no, we have timed out, so kill the read/write operation
      // The read/write callback will be called with an error
      port_.cancel();
    }
  } // namespace: arduino 
} // namespace: ramrod

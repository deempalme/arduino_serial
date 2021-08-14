#include "ramrod/arduino/serial.h"

#include <string.h>                                            // for memcpy
#include <boost/asio/buffer.hpp>                               // for mutabl...
#include <boost/asio/impl/io_context.ipp>                      // for io_con...
#include <boost/asio/io_context.hpp>                           // for io_con...
#include <boost/asio/read.hpp>                                 // for read
#include <boost/asio/serial_port_base.hpp>                     // for serial...
#include <boost/bind.hpp>                                      // for bind_t
#include <boost/date_time/posix_time/posix_time_duration.hpp>  // for millis...
#include <boost/date_time/posix_time/posix_time_types.hpp>     // for micros...
#include <boost/date_time/posix_time/ptime.hpp>                // for ptime
#include <boost/date_time/time.hpp>                            // for base_time
#include <boost/operators.hpp>                                 // for operator>
#include <boost/thread/pthread/thread_data.hpp>                // for sleep_for
#include <boost/thread/thread_only.hpp>                        // for thread
#include <cstdlib>                                             // for free


namespace ramrod {
  namespace arduino {

    serial::serial() :
      io_(),
      port_(io_),
      timeout_(boost::posix_time::milliseconds(1000)),
      error_(),
      buffer_size_{64},
      in_buffer_{static_cast<char*>(std::calloc(buffer_size_, sizeof(char)))},
      in_position_{0},
      in_size_{0},
      reading_{false},
      exit_read_{false},
      out_buffer_{static_cast<char*>(std::calloc(buffer_size_, sizeof(char)))},
      out_position_{0},
      out_size_{0},
      out_finished_{true},
      action_wait_(5),
      one_(1)
    {
    }

    serial::serial(const std::string &port_name, const unsigned int rate, const int config) :
      io_(),
      port_(io_),
      timeout_(boost::posix_time::milliseconds(1000)),
      error_(),
      buffer_size_{64},
      in_buffer_{static_cast<char*>(std::calloc(buffer_size_, sizeof(char)))},
      in_position_{0},
      in_size_{0},
      reading_{false},
      exit_read_{false},
      out_buffer_{static_cast<char*>(std::calloc(buffer_size_, sizeof(char)))},
      out_position_{0},
      out_size_{0},
      out_finished_{true},
      action_wait_(5),
      one_(1)
    {
      begin(port_name, rate, config);
    }

    serial::~serial(){
      end();
      std::free(in_buffer_);
      std::free(out_buffer_);
    }

    int serial::available(){
      return static_cast<int>(in_size_);
    }

    int serial::available_for_write(){
      return static_cast<int>(buffer_size_ - out_size_);
    }

    bool serial::begin(const std::string &port_name, const unsigned int rate, const int config){
      end();

      bool ok{true};

      ok = ok && !port_.open(port_name, error_).failed();

      // Sleeping one millisecond to ensure the concurrent thread as exited
      boost::this_thread::sleep_for(one_);

      ok = ok && set_config(rate, config);

      if(!ok) return false;

      exit_read_ = false;
      boost::thread(boost::bind(&serial::concurrent_read, this)).detach();

      return port_.is_open() && ok;
    }

    bool serial::end(){
      exit_read_ = true;

      if(port_.is_open()){
        port_.cancel();
        return !port_.close(error_).failed();
      }
      return true;
    }

    bool serial::find(const char target){
      if(port_.is_open()) return false;

      std::size_t last{in_position_};

      boost::posix_time::ptime begin = boost::posix_time::microsec_clock::local_time();

      // Trying to find target in the available bytes
      while(true){
        if((in_position_ + in_size_) > last){
          std::size_t e{last - in_position_};
          last = in_position_ + in_size_;
          std::size_t pos;

          // Finding the target character
          for(; e < in_size_; ++e){
            pos = e + in_position_;
            if(pos >= buffer_size_) pos -= buffer_size_;
            // Breaking if the target character was found
            if(in_buffer_[pos] == target)
              return true;
          }
        }

        // This will break the while loop when the max allowed duration is reached
        if((boost::posix_time::microsec_clock::local_time() - begin) > timeout_) break;

        // This will make sleep this thread to avoid CPU overheating
        boost::this_thread::sleep_for(action_wait_);
      }

      return false;
    }

    bool serial::find(const char target, const std::size_t length){
      if(port_.is_open() || length == 0) return false;

      std::size_t i{0};
      std::size_t last{in_position_};

      boost::posix_time::ptime begin = boost::posix_time::microsec_clock::local_time();

      // Trying to find target in the available bytes
      while(i < length){
        if((in_position_ + in_size_) > last){
          std::size_t e{last - in_position_};
          last = in_position_ + in_size_;
          std::size_t pos;

          // Finding the target character
          for(; e < in_size_; ++e, ++i){
            pos = e + in_position_;
            if(pos >= buffer_size_) pos -= buffer_size_;
            // Breaking if the target character was found or the length was reached
            if(in_buffer_[pos] == target)
              return true;
            if(i == length)
              return false;
          }
        }

        // This will break the while loop when the max allowed duration is reached
        if((boost::posix_time::microsec_clock::local_time() - begin) > timeout_) break;

        // This will make sleep this thread to avoid CPU overheating
        boost::this_thread::sleep_for(action_wait_);
      }

      return false;
    }

    bool serial::find_until(const char target, const char terminal){
      if(port_.is_open()) return false;

      std::size_t last{in_position_};

      boost::posix_time::ptime begin = boost::posix_time::microsec_clock::local_time();

      // Trying to find target in the available bytes
      while(true){
        if((in_position_ + in_size_) > last){
          std::size_t e{last - in_position_};
          last = in_position_ + in_size_;
          std::size_t pos;

          // Finding the target character
          for(; e < in_size_; ++e){
            pos = e + in_position_;
            if(pos >= buffer_size_) pos -= buffer_size_;
            // Breaking if the target character was found
            if(in_buffer_[pos] == target)
              return true;
            // Breaking if the terminal character was found
            if(in_buffer_[pos] == terminal)
              return false;
          }
        }

        // This will break the while loop when the max allowed duration is reached
        if((boost::posix_time::microsec_clock::local_time() - begin) > timeout_) break;

        // This will make sleep this thread to avoid CPU overheating
        boost::this_thread::sleep_for(action_wait_);
      }

      return false;
    }

    void serial::flush(){
      while(!out_finished_)
        boost::this_thread::sleep_for(action_wait_);
    }

    int serial::peek(){
      if(in_size_ == 0) return -1;
      return int(in_buffer_[in_position_]);
    }

    int serial::read(){
      if(in_size_ == 0) return -1;
      const int value{int(in_buffer_[in_position_])};
      --in_size_;
      if(++in_position_ >= buffer_size_) in_position_ -= buffer_size_;
      return value;
    }

    std::size_t serial::read_bytes(char *buffer, const std::size_t length){
      if(port_.is_open() || length == 0 || buffer == nullptr) return 0;

      std::size_t i{0};

      boost::posix_time::ptime begin = boost::posix_time::microsec_clock::local_time();

      // Trying to get all bytes
      while(i < length){
        if(in_size_ > 0){
          // Determining how many bytes would be read in this cycle
          const std::size_t to_read = (i + in_size_) > length ? length - i : in_size_;

          const bool bigger{(in_position_ + to_read) >= buffer_size_};

          // Copying the read buffer into your buffer
          if(bigger){
            const std::size_t last{buffer_size_ - in_position_};
            std::memcpy(buffer, in_buffer_ + in_position_, last);
            std::memcpy(buffer + last, in_buffer_, to_read - last);
          }else
            std::memcpy(buffer, in_buffer_ + in_position_, to_read);

          // Changing the pointer and remaining size of the read buffer
          if((in_position_ += to_read) >= buffer_size_) in_position_ -= buffer_size_;
          in_size_ -= to_read;
          i += to_read;
        }
        // Terminating if we already reached the desired length
        if(i >= length) break;

        // This will break the while loop when the max allowed duration is reached
        if((boost::posix_time::microsec_clock::local_time() - begin) > timeout_) break;

        // This will make sleep this thread to avoid CPU overheating
        boost::this_thread::sleep_for(action_wait_);
      }

      return i;
    }

    std::size_t serial::read_bytes_until(const char character, char *buffer,
                                         const std::size_t length){
      if(port_.is_open() || length == 0 || buffer == nullptr) return 0;

      std::size_t i{0};
      bool ready{false};

      boost::posix_time::ptime begin = boost::posix_time::microsec_clock::local_time();

      // Trying to get all bytes
      while(i < length){
        if(in_size_ > 0){
          std::size_t e{0};
          std::size_t pos;
          bool bigger;

          // Finding the termination character
          for(; e < in_size_; ++e, ++i){
            pos = e + in_position_;
            if((bigger = pos >= buffer_size_)) pos -= buffer_size_;
            if(in_buffer_[pos] == character || i == length){
              ready = true;
              break;
            }
          }

          // Copying the read buffer into your buffer
          if(bigger){
            const std::size_t last{buffer_size_ - in_position_};
            std::memcpy(buffer, in_buffer_ + in_position_, last);
            std::memcpy(buffer + last, in_buffer_, pos);
          }else if(e != 0)
            std::memcpy(buffer, in_buffer_ + in_position_, e);

          // Changing the pointer and remaining size of the read buffer
          if((in_position_ += e + 1) >= buffer_size_) in_position_ -= buffer_size_;
          in_size_ -= e + 1;
        }
        // Breaking if the termination character was found or the length was reached
        if(ready) break;

        // This will break the while loop when the max allowed duration is reached
        if((boost::posix_time::microsec_clock::local_time() - begin) > timeout_) break;

        // This will make sleep this thread to avoid CPU overheating
        boost::this_thread::sleep_for(action_wait_);
      }

      return i;
    }

    std::string serial::read_string(){
      std::string result;

      if(port_.is_open()) return result;

      return result;
    }

    void serial::set_timeout(const unsigned int milliseconds){
      if(milliseconds == 0) timeout_ = boost::posix_time::hours(100000);
      else timeout_ = boost::posix_time::milliseconds(milliseconds);
    }

    std::size_t serial::write(const char value){
      if(port_.is_open()) return 0;

    }

    std::size_t serial::write(const std::string &string){
      if(port_.is_open() || string.size() == 0) return 0;

    }

    std::size_t serial::write(const char *buffer, const std::size_t length){
      if(port_.is_open() || length == 0 || buffer == nullptr) return 0;

    }

    ramrod::arduino::serial::operator bool(){
      return port_.is_open();
    }

    bool serial::change_buffer_max_size(const std::size_t new_buffer_size){
      if(new_buffer_size == 0) return false;

      // TODO: cancel data transfer and wait until fully stops
      if(new_buffer_size == buffer_size_) return true;

      char *new_in_buffer{static_cast<char*>(std::calloc(new_buffer_size, sizeof(char)))};
      char *new_out_buffer{static_cast<char*>(std::calloc(new_buffer_size, sizeof(char)))};
      char *ordered_in_buffer{nullptr};
      char *ordered_out_buffer{nullptr};

      // Copying old data into the new buffers

      // From read buffer
      if(in_size_ > 0){
        ordered_in_buffer = static_cast<char*>(std::malloc(buffer_size_));
        // Ordering old buffer
        std::memcpy(ordered_in_buffer, in_buffer_ + in_position_, buffer_size_ - in_position_);
        const std::size_t in_remaining{in_size_ - (buffer_size_ - in_position_)};
        if(in_remaining > 0)
          std::memcpy(ordered_in_buffer + buffer_size_ - in_position_, in_buffer_, in_remaining);
        // Copying to new buffer
        std::memcpy(new_in_buffer, ordered_in_buffer,
                    new_buffer_size >= buffer_size_ ? buffer_size_ : new_buffer_size);
        std::free(ordered_in_buffer);
      }

      // From write buffer
      if(out_size_ > 0){
        ordered_out_buffer = static_cast<char*>(std::malloc(buffer_size_));
        // Ordering old buffer
        std::memcpy(ordered_out_buffer, out_buffer_ + out_position_, buffer_size_ - out_position_);
        const std::size_t out_remaining{in_size_ - (buffer_size_ - in_position_)};
        if(out_remaining > 0)
          std::memcpy(ordered_out_buffer + buffer_size_ - out_position_, out_buffer_, out_remaining);
        // Copying to new buffer
        std::memcpy(new_out_buffer, ordered_out_buffer,
                    new_buffer_size >= buffer_size_ ? buffer_size_ : new_buffer_size);
        std::free(ordered_out_buffer);
      }

      std::free(in_buffer_);
      std::free(out_buffer_);

      in_buffer_ = new_in_buffer;
      in_position_ = 0;
      if(in_size_ > new_buffer_size) in_size_ = new_buffer_size;
      out_buffer_ = new_out_buffer;
      out_position_ = 0;
      if(out_size_ > new_buffer_size) out_size_ = new_buffer_size;
      buffer_size_ = new_buffer_size;
      return true;
    }

    // ::::::::::::::::::::::::::::::::::: PRIVATE FUNCTIONS :::::::::::::::::::::::::::::::::::::

    void serial::concurrent_read(){
      std::size_t total_read{0};
      std::size_t next{0};
      char input;
      boost::asio::mutable_buffer buffer{boost::asio::buffer(&input, 1)};
      reading_ = true;

      // Waiting until serial port is opened
      while(!port_.is_open() && !exit_read_)
        boost::this_thread::sleep_for(one_);

      while(!exit_read_){
        // Waiting for data to income
        total_read = boost::asio::read(port_, buffer);
        // If the buffer is full then continue reading discarting all input
        if(in_size_ >= buffer_size_) continue;
        // Saving the data into the read buffer
        next = in_position_ + in_size_;
        if(next >= buffer_size_) next -= buffer_size_;
        *(in_buffer_ + next) = input;
        in_size_ += total_read;
      }

      reading_ = false;
    }

    bool serial::set_config(const unsigned int rate, const int config){
      // Setting baud rate
      bool failed{port_.set_option(boost::asio::serial_port::baud_rate(rate),
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
  } // namespace: arduino 
} // namespace: ramrod

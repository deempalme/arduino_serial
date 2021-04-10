<img src="https://github.com/deempalme/arduino_serial/wiki/images/logo.jpg" alt="Arduino serial">

## Introduction

**Arduino serial** is a multi-platform library to communicate to arduino devices throught the serial port using the same function's names and code structure.

You should read the <a href="https://github.com/deempalme/arduino_serial/wiki/Installation-guide">installation guide</a> to setup all the necessary libraries and get your serial communication ready to run.

This is the guide for the library:

| Functions | Description |
|---        | ---         |
| [(constructor)](https://github.com/deempalme/arduino_serial/wiki/Constructor) | Creates the serial port object. |
| [if(serial)](https://github.com/deempalme/arduino_serial/wiki/Serial) | Indicates if the specified Serial port is ready. |
| [available()](https://github.com/deempalme/arduino_serial/wiki/Available) | Get the number of bytes (characters) available for reading. |
| [available_for_write()](https://github.com/deempalme/arduino_serial/wiki/Availabel-for-write) | Get the number of bytes (characters) available for writing in the serial buffer without blocking the write operation. |
| [begin()](https://github.com/deempalme/arduino_serial/wiki/Begin) | Opening serial communication. |
| [end()](https://github.com/deempalme/arduino_serial/wiki/End) | Disables serial communication. |
| [find()](https://github.com/deempalme/arduino_serial/wiki/Find) | Reads data from the serial buffer until the target is found. |
| [find_until(Find-until)](https://github.com/deempalme/arduino_serial/wiki/Find-until) | Reads data from the serial buffer until the target is found. |
| [flush()](https://github.com/deempalme/arduino_serial/wiki/Flush) | Waits for the transmission of outgoing serial data to complete. |
| [parse_float()](https://github.com/deempalme/arduino_serial/wiki/Parse-float) |  |
| [parse_int()](https://github.com/deempalme/arduino_serial/wiki/Parse-int) |  |
| [peek()](https://github.com/deempalme/arduino_serial/wiki/Peek) | Peeking the next character in the reading buffer |
| [print()](https://github.com/deempalme/arduino_serial/wiki/Print) |  |
| [print_ln()](https://github.com/deempalme/arduino_serial/wiki/Print-line) |  |
| [read()](https://github.com/deempalme/arduino_serial/wiki/Read) | Reads incoming serial data. |
| [read_bytes()](https://github.com/deempalme/arduino_serial/wiki/Read-bytes) | Read a certain quantity of bytes |
| [read_bytes_until()](https://github.com/deempalme/arduino_serial/wiki/Read-bytes-until) | Read a certain quantity of bytes or until certain character is reached |
| [read_string()](https://github.com/deempalme/arduino_serial/wiki/Read-string) | Reading a string |
| [read_string_until()](https://github.com/deempalme/arduino_serial/wiki/Read-string-until) | Reading a string up to a termination character |
| [set_timeout()](https://github.com/deempalme/arduino_serial/wiki/Set-timeout) | Sets the maximum milliseconds to wait for serial data. It defaults to 1000 milliseconds. |
| [write()](https://github.com/deempalme/arduino_serial/wiki/Write) | Writing a single byte |
| [serial_event()](https://github.com/deempalme/arduino_serial/wiki/Serial-event) | Writing a string |

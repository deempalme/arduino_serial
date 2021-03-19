#ifndef RAMROD_ARDUINO_CONSTANTS_H
#define RAMROD_ARDUINO_CONSTANTS_H

namespace ramrod {
  namespace arduino {
    enum serial_config : int {
      SERIAL_5N1 = 0b00010010,
      SERIAL_6N1 = 0b00100010,
      SERIAL_7N1 = 0b01000010,
      SERIAL_8N1 = 0b10000010, // (the default)
      SERIAL_5N2 = 0b00010011,
      SERIAL_6N2 = 0b00100011,
      SERIAL_7N2 = 0b01000011,
      SERIAL_8N2 = 0b10000011,
      SERIAL_5E1 = 0b00010100, // even parity
      SERIAL_6E1 = 0b00100100,
      SERIAL_7E1 = 0b01000100,
      SERIAL_8E1 = 0b10000100,
      SERIAL_5E2 = 0b00010101,
      SERIAL_6E2 = 0b00100101,
      SERIAL_7E2 = 0b01000101,
      SERIAL_8E2 = 0b10000101,
      SERIAL_5O1 = 0b00011000, // odd parity
      SERIAL_6O1 = 0b00101000,
      SERIAL_7O1 = 0b01001000,
      SERIAL_8O1 = 0b10001000,
      SERIAL_5O2 = 0b00011001,
      SERIAL_6O2 = 0b00101001,
      SERIAL_7O2 = 0b01001001,
      SERIAL_8O2 = 0b10001001
    };

    enum print_format : int {
      DEC = -1,
      HEX = -2,
      OCT = -3,
      BIN = -4
    };
  } // namespace: arduino
} // namespace: ramrod

#endif // RAMROD_ARDUINO_CONSTANTS_H

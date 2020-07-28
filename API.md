# RoverWing Firmware API

RoverWing must be connected to the master controller by I2C bus;
RoverWing acts as a slave on the bus, using I2C address  0x11. It supports
regular of Fast Mode (400khz) speed I2C protocol.

Interactions between the master MCU and RoverWing follow usual register model:
master either writes to a register or requests data from a register. RoverWing
has two register banks, bank A and bank B; somewhat unusually,  bank  A is
**read-only**, and bank  B **write only**. Because of this, there is no need to
specify the bank: any read request is interpreted as request to read from bank
A registers, and any write request is interpreted as a request to write to bank B.
There are no  read and write registers.

All multibyte data registers use little-endian byte order: thus, to get say
reading of analog sensor 0, which is a 16-bit integer stored in registers 2 and
3 in bank A, one would request 2 bytes from register 2, saving them in array
`bytes[]`, and  construct result as
`uint16_t analog_reading=(bytes[1]<<8|bytes[0])`

## Register bank A - read-only

|register(s)|  Name              | data type      |           Value | comment |
|------------|-------------------| ---------------|-----------------| --------|
| 0          | REGA_FW_VERSION   |    uint8       | firmware version - minor|
| 1          |                   |     uint8      | fw version major|
| 2-3        | REGA_ANALOG_RAW   |  uint16        | Raw reading of analog 0 (voltage), 0-1023;
| 4-5        |                   |  uint16        | Raw reading of analog 1, 0-1023|
| 6-7        |                   |  uint16        | Raw reading of analog 2, 0-1023|
| 8-9        |                   |  uint16        | Raw reading of analog 3, 0-1023|
| 10-11      |                   |  uint16        | Raw reading of analog 4, 0-1023|
| 12-13      |                   |  uint16        | Raw reading of analog 5, 0-1023|
| 14-15      |                   |  uint16        | Raw reading of analog 6, 0-1023|
|------------|-------------------| ---------------|-----------------| --------|

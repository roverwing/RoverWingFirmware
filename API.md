# RoverWing Firmware API

RoverWing must be connected to the master controller by I2C bus;
RoverWing acts as a slave on the bus, using I2C address  0x11. It supports
regular of Fast Mode (400khz) speed I2C protocol.

Interactions between the master MCU and RoverWing follow usual register model:
master either writes to a register or requests data from a register. RoverWing
has two register banks, bank A and bank B; somewhat unusually,  bank  A is
**read-only**, and bank  B **write-only**. Because of this, there is no need to
specify the bank: any read request is interpreted as request to read from bank
A registers, and any write request is interpreted as a request to write to bank B.
There are no  read and write registers.

All multibyte data registers use little-endian byte order: thus, to get say
reading of analog sensor 0, which is a 16-bit integer stored in registers 2 and
3 in bank A, one would request 2 bytes from register 2, saving them in array
`bytes[]`, and  construct result as
`uint16_t analog_reading=(bytes[1]<<8|bytes[0])`

## Register bank A - read-only

|register(s)|  Name              | data type      |           Value             | 
|------------|-------------------| ---------------|-----------------------------|
| 0          | REGA_FW_VERSION   |  uint8         | firmware version - minor    |
| 1          |                   |  uint8         | fw version major            |
| 2-3        | REGA_ANALOG_RAW   |  uint16        | Raw reading of analog 0 (battery voltage), 0-1023;
| 4-5        |                   |  uint16        | Raw reading of analog 1, 0-1023. Value of 1023 corresponds to 3.3V|
| 6-7        |                   |  uint16        | Raw reading of analog 2, 0-1023|
| 8-9        |                   |  uint16        | Raw reading of analog 3, 0-1023|
| 10-11      |                   |  uint16        | Raw reading of analog 4, 0-1023|
| 12-13      |                   |  uint16        | Raw reading of analog 5, 0-1023|
| 14-15      |                   |  uint16        | Raw reading of analog 6, 0-1023|
| 16-17      | REGA_ANALOG       |  uint16        | Filtered reading of analog 0 (voltage), scale 0-10230 (note: it 10230 is not a typo!)|
| 18-19      |                   |  uint16        | Filtered reading of analog 1, scale 0-10230 |
| 20-21      |                   |  uint16        | Filtered reading of analog 2, scale 0-10230 |
| 22-23      |                   |  uint16        | Filtered reading of analog 3, scale 0-10230 |
| 24-25      |                   |  uint16        | Filtered reading of analog 4, scale 0-10230 |
| 26-27      |                   |  uint16        | Filtered reading of analog 5, scale 0-10230 |
| 28-29      |                   |  uint16        | Filtered reading of analog 6, scale 0-10230 |
| 30-31      | REGA_SONAR_RAW    |  uint16        | raw sonar distance values for sonar1, in mm |
| 32-33      |                   |  uint16        | same for  sonar2                            |
| 34-35      |                   |  uint16        | same for sonar3                             |
| 36-37      | REGA_SONAR        |  uint16        | filtered sonar distance for sonar1, in units of 0.1mm |
| 38-39      |                   |  uint16        | same for sonar2             |
| 40-41      |                   |  uint16        | same for sonar3             |
| 42         | REGA_WHO_AM_I     |  uint8t        | 0x11 (decimal 17); used for testing connection|
| 43         |                   |                | unused                      |
| 44-47      | REGA_ENCODER      |   int32        | motor1 encoder value, in ticks|
| 48-51      |                   |   int32        | motor2 encoder value, in ticks|
| 52-53      | REGA_SPEED        |   int16        | motor 1 speed, in encoder ticks/s|
| 54-55      |                   |   int16        | motor 2 speed               |
| 56         | REGA_IMU_STATUS   |   uint8        | IMU status                  |
| 57-59      |                   |                | unused                      |
| 60-61      | REGA_ACCEL        |   int16        | acceleration x, LSB=1/16384 g|
| 62-63      |                   |   int16        | acceleration y              |
| 64-65      |                   |   int16        | acceleration z              |
| 66-67      | REGA_GYRO         |   int16        | angular speed x-axis, LSB=(250.0 / 32768.0) deg/s|
| 68-69      |                   |   int16        | angular speed y             |
| 70-71      |                   |   int16        | angular speed z             |
| 72-75      | REGA_QUAT         |   float(4-byte)| orientation as quaternion, real part|
| 76-79      |                   |   float        | i-part
| 80-83      |                   |   float        | j-part
| 84-87      |                   |   float        | k-part
| 88-89      | REGA_YAW          |   int16        | yaw angle, in units of 1/10 degree   |
| 90-91      | REGA_PITCH        |   int16        | pitch angle, in units of 1/10 degree |
| 92-93      | REGA_ROLL         |   int16        | roll  angle, in units of 1/10 degree |
| 94-95      | REGA_ACCEL_OFFSET |   int16        | x-component of accel offset, in raw units|
| 96-97      |                   |   int16        | y
| 98-99      |                   |   int16        | z
| 100-101    | REGA_GYRO_OFFSET  |   int16        | x-component of gyro offset, in raw units
| 102-103    |                   |   int16        | y
| 104-105    |                   |   int16        | z
| 106        | REGA_MAG_STATUS   |   uint8        | Magnetometer status
| 107        |                   |                | unused
| 108-109    | REGA_MAG          |   int16        | x-compoment of magnetic field, in units of 0.92 milligauss=0.092uT
| 110-111    |                   |   int16        | y
| 112-113    |                   |   int16        | z
| 114-115    | REGA_MAG_OFFSET   |   int16        | x-component  of magnetometer offset currently in use (either from calibration of copied form magUsrOffset)
| 116-117    |                   |   int16        | y
| 118-119    |                   |   int16        | z
| 120        | REGA_GPS_STATUS   |   uint8        | GPS status
| 121-123    |                   |                | unused
| 124-127    | REGA_GPS_LAT      |   int32        | latitude, in units of 10^{-7} degree (about 10cm)
| 128-131    | REGA_GPS_LONG     |   int32        | longitude in units of 10^{-7} degree
| 132-135    | REGA_GPS_TIMESTAMP | uint32        | timestamp of last measurement, in ms
| 136        | REGA_DRIVE_STATUS |   byte         |
| 137        |                   |                | unused
| 138-139    | REGA_DEBUG        | int16          | for debugging purposes
| 140-141    |                   | int16          |
| 142-143    |                   | int16          |
|------------|-------------------|----------------|----------------------------|

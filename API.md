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
| 140-141    |                   | int16          | same
| 142-143    |                   | int16          | same



## REGISTER B - write-only

|register(s)|  Name             | data type     |           Value             |
|---------|---------------------| --------------|-----------------------------|
| 0       | REGB_ANALOG_BITMASK | byte          | bitmask of active analog sensors, LSB=analog1 analog0=vsense - always active
| 1       | REGB_SONAR_BITMASK  | byte          | bitmask of active sonars, LSB=sonar1
| 2-3     | REGB_SONAR_TIMEOUT  | uint16        | timeout for waiting for sonar echo, in us
| 4-5     | REGB_SERVO          | uint16        | pulse width for servo1, in us (500-2500)
| 6-7     |                     | uint16        | same for servo2
| 8-9     |                     | uint16        | same for servo3
| 10-11   |                     | uint16        | same for servo4              
| 12-27   | REGB_MOTOR1_PID     | float[4]      | PID coefficients for Motor1
| 28-41   | REGB_MOTOR2_PID     | float[4]      | PID coefficients for Motor2
| 42      | REGB_ENC_RESET      | byte          | bit 0: reset motor1 encoder; bit 1: motor 2
| 43      | REGB_MOTOR_MODE     | byte          | motor1 mode
| 44      |                     | byte          | motor2 mode
| 45      |                     |               | unused
| 46-47   | REGB_MOTOR_POWER    | int16         | motor1 power, -500...500
| 48-49   |                     | int16         | same, motor 2
| 50-51   | REGB_MOTOR_MAXSPEED | uint16        | Maximal motor1 speed, in enc ticks/s
| 52-53   |                     | uint16        | Same, motor2
| 54-55   |                     |               | unused    
| 56-59   | REGB_MOTOR_TARGET   | int32         | in speed PID mode: motor1 target speed  in enc ticks/s
| 60-63   |                     | int32         | same for motor2
| 64      | REGB_IMU_CONFIG     | byte          | bit0: is IMU active?; bits 2:1 : IMU orientation
| 65      |                     |               | unused
| 66-67   | REGB_GYRO_OFFSET    | int16         | gyro offset x, provided by user
| 68-69   |                     | int16         | y
| 70-71   |                     | int16         | z
| 72-73   | REGB_ACCEL_OFFSET   | int16         | accel offset x, provided by user
| 74-75   |                     | int16         | y
| 76-77   |                     | int16         | z
| 78      | REGB_MAG_CONFIG     | byte          | magnetometrr mode - see definition in mag.h
| 79      |                     |               | unused
| 80-81   | REGB_MAG_OFFSET     | int16         | x-offset for magnetometer provided by user
| 82-83   |                     | int16         | y-offset for magnetometer
| 84-85   |                     | int16         | z-offset for magnetometer
| 86-87   | REGB_MAG_MATRIX     | int16         | (soft iron matrix)*1000 - entry [0][0]
| 88-89   |                     | int16         | [0][1]
| 90-91   |                     | int16         | [0][2]
| 92-93   |                     | int16         | [1][0]
| 94-95   |                     | int16         | [1][1]
| 96-97   |                     | int16         | [1][2]
| 98-99   |                     | int16         | [2][0]
| 100-101 |                     | int16         | [2][1]
| 102-103 |                     | int16         | [2][2]
| 104     | REGB_GPS_CONFIG     | byte          | 1 if we need to activate gps
| 105     | REGB_LOW_VOLTAGE    | uint8_t       | the low voltage threshold, in units of 0.1V.  if voltage is below that, internal neopixel will blink red
| 106     | REGB_NUM_PIXELS     | uint8         | actual number of used neopixels (at most 255);  doesn't include internal neopixel
| 107     | REGB_PIXEL_BRIGHTNESS | uint8       | neopixel brightness, 0-255
| 108-111 | REGB_PIXEL_COLOR    | uint32        | neopixel color data  (nnRRGGBB), where nn is the pixel index (1-255)
| 112     | REGB_PIXEL_COMMAND  | uint8         | command to start/update/stop pixels
| 113     | REGB_DRIVE_MODE     | byte          |
| 114     | REGB_DRIVE_MOTORCONFIG | byte       | info about which motor is left/right, etc. See details in drive.cpp
| 115     |                     |               | unused
| 116-117 | REGB_DRIVE_MAXSPEED | uint16        | maximal possible speed, in encoder ticks/s; determined by the motor
| 118-119 | REGB_DRIVE_MAXTURNSPEED | uint16    | maximal turning speed, in deg/s
| 120-121 | REGB_DRIVE_MINPOWER | uint16        | minimal power necessary for the robot to move when driving straight (0-500)
| 122-123 |                     |               | unused
| 124-139 | REGB_DRIVE_PID_COEF | float[4]      | PID coefficients for driving straight using IMU
| 140-143 | REGB_DRIVE_DISTANCE | int32         | distance to drive, in encoder ticks. Always positive, even when going backwards
| 144-145 | REGB_DRIVE_HEADING  | int16         | target value of heading (as yaw angle), in units of 0.1 deg. Must be between -1800 and 1800
| 146-147 | REGB_DRIVE_TARGETPOWER | int16      | requested  power, -500...500. For driving backwards must be negative
| 148-149 | REGB_DRIVE_RAMPTIME    | uint16     | time for ramping speed from 0 to full, in ms

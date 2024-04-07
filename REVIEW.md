# Review

## Review Meet 2

Date: 30 June, 2017

### Progress so far

- Completed the mechanical structure of the quadcopter
- Designed and soldered the power distribution circuit on a PCB
- Created 4 fundamental libraries for interfacing with Arduino Due
    - `InitTimer.h`: Provides timer counter and delay functions
    - `InitUART.h`: Establishes communication with PC via USB and with GPS
    - `InitTWI.h`: Establishes communication with I2C sensors
    - `InitADC.h`: Initializes Analog to Digital Conversion at pin A0
- Programmed NVIC to generate pin change interrupts for reading receiver signals
- Wrote the ESC calibrate sketch and under - progress flight controller sketch using the previously created libraries
- Wrote a sketch read receiver signals and print on serial monitor in real time

#### I2C Sensors

- MPU 6050 – 3 AXIS GYROSCOPE AND ACCELEROMETER
    - I2c device address : 0x68
    - Register 0x6B PWR_MGMT_1 | Write 0x00 to disable sleep mode
    - Register 1A CONFIG | Write 0x03 | Set DLPF to 43Hz for acc and gyro
    - Register 1B GYRO_CONFIG | Write 0x08 | Set gyro Full Scale to +-500 dps with LSB of 65.5/dps 
    - Register 0x68 SIGNAL_PATH_RESET | Write 0x03 to reset
    - Register 0x6A USER_CONTROL | Write 0x01 to reset
    - Register 0x6B PWR_MGMT_1 | Write 0x80 to device_reset
- HMC5883L – 3 AXIS DIGITAL MAGNETOMETER
    - I2c device address : 0x1E
    - Read/Write Identifier :  Read  ----  0x3D | Write ----  0x3C
    - Register 0x00 CONFIG_REG_A | Write 0x59
	- No. of samples averaged per measurement: 4
	- Typical Data Output Rate (Hz): 75
	- Incorporated Positive applied bias into measurement
    - Register 0x01 CONFIG_REG_B | Write 0x20
	- Sensor Field Range: +-1.3 Ga | Gain: 1090 LSB/Ga
    - Register 0x02 Mode Register | Write 0x08
	- Continuous-Measurement Mode enabled | High Speed i2c enabled

### Results so far

- Can display raw data from MPU 6050 and HMC5883L on the Serial Monitor
- Can display receiver signals on Serial Monitors
- Calibrate the ESCs
- Control the motors via transmitter

### Problems so far

- Debugging the I2C Communication Program was quite tough as there were many unexpected sources of errors
- Creating the InitUART header file was challenging as it required everything one wanted to print on Serial Monitor to be converted to arrays of char
- The float values on conversions to strings, accumulated round off error in the process
- Assigning 16 bit values in two’s complement form to variables was a problem difficult to figure out as it wasn’t a problem in case of Arduino Uno 

### Work to be done

- Write code for Sensor Fusion implementing Kalman Filters
- Write code for storing orientation data into a Quaternion
- Write code for extracting roll, pitch and yaw values from the quaternion
- Write code for PID algorithm to calculate ESC inputs
- Test Flights to calibrate PID gains
- Ensure everything gets done in the specified time limit
- Debugging for errors and improvising the code

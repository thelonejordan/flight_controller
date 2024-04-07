# ITSP PROJECT ABSTRACT

PROJECT TITLE: ***Kalman Filter Based IMU Sensor and its application in quadcopter navigation system***

TEAM: **ANTIMONY**

MEMBERS:

1. Jyotirmaya Mahanta (Team Leader)
    - jyotirmaya.mahanta@gmail.com
    - 160110092
2. Vishal Kumar
    - kumarvishal4626@gmail.com
    - 160110058
3. Vaibhav Bamnawat
    - jharedajp@gmail.com
    - 160110039
4. Shubham Behere
    - shubhamb61098@gmail.com
    - 160110026



## PARTS LIST

- Teensy 3.x 
- Arduino Due
- MPU-6050(gyroscope & accelerometer) + HMC5883L(magnetometer)       
- 16x2 LCD Display
- Breadboard and Jumper Wires
- Brushless Motors x 4
- 3s 3000mAh 30c lipo battery pack
- 10x4.5 propellers - 2 x CW + 2 x CCW
- SD card(1/2gb based on availability)

## ESTIMATED COST

12k INR (including delivery charges estimates)

## PREFERRED SLOT

SLOT 1 - 1st MAY to 6th JUNE

## PROJECT DESCRIPTION

### Introduction

An inertial measurement unit (IMU) is an electronic device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, using a combination of motion sensors(such as gyroscopes, accelerometers & sometimes magnetometers).

If we want to control the orientation of an object(say a quadcopter) it is necessary to know the real time orientation of the quadcopter. Knowing the real time orientation, we can then generate a set of commands to navigate the orientation of the quadcopter towards the required orientation. This is precisely why IMU Sensors are used to determine the orientation in real time. Now let’s consider the type output the motion sensors provide.

A gyroscope outputs angular velocity. Thus, a gyroscope is sufficient to detect change in angle by integrating angular velocity over time. But in the process, the noise in the output also gets integrated, and results in drift from the actual value. So gyroscopes are reliable only for short periods of time and  are prone to errors in the long run.

An accelerometer outputs proper acceleration. It can be used to sense orientation (because direction of weight changes), coordinate acceleration, vibration, shock. When the sensor is still, it outputs accurate values. But in real situations, vibrations and shocks are inevitable, for example, in case of quadcopter, the vibrations from rotating propellers makes the accelerometer output too noisy and unusable. 

### Objective and Approach

Neither gyroscope nor accelerometer can supply a very good & accurate measurement of orientation as implied above. So clearly, the objective is to extract good quality data from the motion sensors and minimise error by implementing a particular approach.

The approach is to combine the gyroscope (, magnetometer) and accelerometer data by using mathematical algorithm(dead reckoning), namely Kalman Filters. The algorithm is recursive and works in a two-step process. It can run in real time, using only the present input measurements and the previously calculated state and its uncertainty matrix; no additional past information is required. To combine data from all sensors to produce more accurate results is also known as Sensor Fusion(watch this). 

NOTE 1 : The accelerometer is unresponsive to yaw movement. So sensor fusion is achieved along the yaw axis by combining data from gyroscope and magnetometer.

NOTE 2 : The Teensy 3.x(up to 180 MHz) board is required for sensor fusion as the arduino( uno 20 MHz) boards are relatively slow.

### Application

For the application part, the IMU Sensor is used as navigation system for the quadcopter. The quadcopter flight controller will use PID algorithm to drive the motors.

Regarding constructing flight controller algorithms,

1. **Reading receiver inputs:**
    receiver outputs  pulses of 1000 to 2000 microsecond. I have hooked up four receiver channels to four digital pins of the arduino and enabled interrupt. So whenever the digital pins toggle,the interrupt is active. So a variable can be set to keep track of the state(high or low) of the pin. Using `micros()`, record the rising time and falling time, subtract to get the pulse length.

2. **Reading gyro, accelerometer & magnetometer data:**
    Using Wire library(`I2c`), we can get the raw data from these sensors easily. We just have look at the register mappings and descriptions. For mpu 6050 I have already worked on that. 

3. **Controlling motors using ESCs:**
    Escs take input just as we give input to a servo. We just have to create pulse ranging from 1000 to 2000 us. Many possibilities are there such as 
    ```cpp
    digitalWrite(escPin, HIGH);
    delayMicroseconds(pulse_length)   // 1000<pulse_length<2000
    digitalWrite(escPin, LOW);
    ```
    or

    ```cpp
    timer =micros() + pulse_length;
    digitalWrite(escPin, HIGH);
    while(micros()<timer);
    digitalWrite(escPin, LOW); 
    ```
4. **Keep track of battery voltage:**
    This can be done using analogRead.

5. **PID algorithm:**
    In my case, the pid algorithm is used to achieve certain angles. For example, if we command the quadcopter to roll by a certain angle( by moving the roll stick of the transmitter), the pid setpoint is set to the desired angle. And the work of the algorithm is to match the desired angle with the real angle(determined using imu sensor). For example , roll stick:
    - 1000us --- -30 degree
    - 1500us ---  0 degree
    - 2000us --- +30 degree        
    
    The algorithm is straightforward

    ```
    output = P_Gain*error + I_gain*(integral of error over time) + D_Gain*(d(error)/dt)

    where, error = set point(desired angle) - IMU_output(real angle)
    ```

    dt will be fixed by setting the loop time to a constant value.
    And the gain values are constants which can only be determined by test flights.
    The output will be added or subtracted to the the throttle value and then, will be fed to the escs.

6. **Quadcopter Dynamics:**
    Its pretty simple and straightforward. It relates the speed of each motors to the particular maneuver we want to perform. And converting them into code is not too painful as described above. We just have to add or subtract from the throttle value.

## PLAN OF ACTION

Main Project Part

### WEEK 1

- Write arduino code to configure registers and extract raw data from gyroscope, accelerometers and magnetometer using Wire(I2C) library. This requires reading the datasheet, especially register maps and descriptions.
- Understand the type of output of the sensors and convert them into usable form(the gyroscope does not directly output angular velocity in degree per second). Firstly, we have to find the LSB(sensitivity ratio) of the output of each sensor. 
- Write the calibration procedure for each of the sensors to remove zero errors.
- Write code for integration of gyroscope data( angle from angular velocity), and implement Euler angles to correlate the x, y and z axis with each other. 

### WEEK 2

- Understand Kalman filters theoretically.
- Write arduino code for implementation of kalman filters on all the sensors.
- Determine the uncertainty matrix, which is unique for each sensor.

### WEEK 3

- Learn and write code in Processing to display the output of the Kalman filters on the screen.
Application Part
- Learn and solder the electrical components of the quadcopter together.
- Learn and write code for reading receiver signals.
- Learn and write code for reading data from IMU sensor.

### WEEK 4

- Learn and write code for checking battery voltage.
- Learn and write code for controlling brushless motors by supplying output signals to ESCs and then write code for the ESC calibration.
- Learn quadcopter flight dynamics(for example how to change the speed of each motor in order to execute specific flight maneuvers such as roll, pitch and yaw).

### WEEK 5

- Learn PID algorithm and write code for the same.
- Determine the PID gain values by trial and error method(test flights).
- Integrate all the above codes into a single flight controller code.
- Debugging and test flights.

NOTE : Teensy 3.x supports arduino code(via a bootloader available online).

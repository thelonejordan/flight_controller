#include <Wire.h>                     //Include the Wire.h library so we can communicate with the gyro
#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM


//Declaring Global Variables
#define LED 12 
byte pin_last_state[4];                                                  //contains the last state of pins 8, 9, 10 & 11                                                  
byte gyro_address, gyro_type, error, clockspeed_ok;
byte channel_assign[4], channel_input_pin[4];                            //pins assigned to channels
byte roll_axis, pitch_axis, yaw_axis;                                    //gyro axes assigned to movements
byte receiver_check_byte, gyro_check_byte;
volatile int receiver_input_pin[4];                                      //contains the latest reciever inputs(pulse length of signal)
int center_channel[4];                                                   //center values of each channel
int high_channel[4];                                                     //high values of each channel
int low_channel[4];                                                      //low values of each channel
int cal_int;                                                         
unsigned long timer, timer_[4], current_time;
float gyro_axis[3];                                   
float gyro_axis_cal[3];                                                  //calibration values


//Setup routine
void setup(){
  pinMode(LED, OUTPUT);     //set LED pin to output
  
  PCICR  |= (1 << PCIE0);   // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change
  
  Wire.begin();             //Start the I2C as master
  Serial.begin(57600);      //Start the serial connetion @ 57600bps
  
  delay(250);               //Give the gyro time to start 
}

void loop(){
  intro();                  //Show the PROJECT INTRO
  
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("System check"));
  Serial.println(F("==================================================="));
  delay(1000);
  Serial.println(F("Checking I2C clock speed."));
  delay(1000);
  
  TWBR = 12;                      //Set the I2C clock speed to 400kHz.
  
  #if F_CPU == 16000000L          //If the clock speed is 16MHz include the next code line when compiling
    clockspeed_ok = 1;            //Set clockspeed_ok to 1
  #endif                          //End of if statement

  if(TWBR == 12 && clockspeed_ok){
    Serial.println(F("I2C clock speed is correctly set to 400kHz."));
  }
  else{
    Serial.println(F("I2C clock speed is not set to 400kHz."));
    error = 1;
  }
  
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Transmitter setup"));
    Serial.println(F("==================================================="));
    delay(1000);
    Serial.print(F("Checking for valid receiver signals."));
    wait_for_receiver();         //Wait 10 seconds until all receiver inputs are valid                    
    Serial.println(F(""));
  }
  
  if(error == 0){                //Quit the program in case of an error
    delay(2000);
    Serial.println(F("Place all sticks and subtrims in the center position within 10 seconds."));
    for(int i = 9;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
    }
    Serial.println(" ");
    //Store the central stick positions
    center_channel[1] = receiver_input_pin[1];
    center_channel[2] = receiver_input_pin[2];
    center_channel[3] = receiver_input_pin[3];
    center_channel[4] = receiver_input_pin[4];
    Serial.println(F(""));
    Serial.println(F("Center positions stored."));
    Serial.print(F("Digital input 08 = "));
    Serial.println(center_channel[1]);
    Serial.print(F("Digital input 09 = "));
    Serial.println(center_channel[2]);
    Serial.print(F("Digital input 10 = "));
    Serial.println(center_channel[3]);
    Serial.print(F("Digital input 11 = "));
    Serial.println(center_channel[4]);
    Serial.println(F(""));
    Serial.println(F(""));
  }
  if(error == 0){  
    Serial.println(F("Move the throttle stick to full throttle and back to center"));
    //Check for throttle movement
    check_receiver_inputs(1);
    Serial.print(F("Throttle is connected to digital input "));
    Serial.println((channel_assign[3] & 0b00000111) + 7);
    if(channel_assign[3] & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
    
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the roll stick to simulate left wing up and back to center"));
    //Check for roll movement
    check_receiver_inputs(2);
    Serial.print(F("Roll is connected to digital input "));
    Serial.println((channel_assign[1] & 0b00000111) + 7);
    if(channel_assign[1] & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the pitch stick to simulate nose up and back to center"));
    //Check for pitch movement
    check_receiver_inputs(3);
    Serial.print(F("Pitch is connected to digital input "));
    Serial.println((channel_assign[2] & 0b00000111) + 7);
    if(channel_assign[2] & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the yaw stick to simulate nose right and back to center"));
    //Check for yaw movement
    check_receiver_inputs(4);
    Serial.print(F("Yaw is connected to digital input "));
    Serial.println((channel_assign[4] & 0b00000111) + 7);
    if(channel_assign[4] & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Gently move all the sticks simultaneously to their extends"));
    Serial.println(F("When ready put the sticks back in their center positions"));
    //Register the min and max values of the receiver channels
    register_min_max();
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("High, low and center values found during setup"));
    Serial.print(F("Throttle values:"));
    Serial.print(low_channel[channel_input_pin[3]]);
    Serial.print(F(" - "));
    Serial.print(center_channel[channel_input_pin[3]]);
    Serial.print(F(" - "));
    Serial.println(high_channel[channel_input_pin[3]]);
    Serial.print(F("Roll values:"));
    Serial.print(low_channel[channel_input_pin[1]]);
    Serial.print(F(" - "));
    Serial.print(center_channel[channel_input_pin[1]]);
    Serial.print(F(" - "));
    Serial.println(high_channel[channel_input_pin[1]]);
    Serial.print(F("Pitch values:"));
    Serial.print(low_channel[channel_input_pin[2]]);
    Serial.print(F(" - "));
    Serial.print(center_channel[channel_input_pin[2]]);
    Serial.print(F(" - "));
    Serial.println(high_channel[channel_input_pin[2]]);
    Serial.print(F("Yaw values:"));
    Serial.print(low_channel[channel_input_pin[4]]);
    Serial.print(F(" - "));
    Serial.print(center_channel[channel_input_pin[4]]);
    Serial.print(F(" - "));
    Serial.println(high_channel[channel_input_pin[4]]);
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    check_to_continue();
  }
    
  if(error == 0){
    //What gyro is connected
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro search"));
    Serial.println(F("==================================================="));
    delay(2000);
    
    Serial.println(F("Searching for MPU-6050 on address 0x68/104"));
    delay(1000);
    if(search_gyro(0x68, 0x75) == 0x68){
      Serial.println(F("MPU-6050 found on address 0x68"));
      gyro_type = 1;
      gyro_address = 0x68;
    }
    
    if(gyro_type == 0){
      Serial.println(F("Searching for MPU-6050 on address 0x69/105"));
      delay(1000);
      if(search_gyro(0x69, 0x75) == 0x68){
        Serial.println(F("MPU-6050 found on address 0x69"));
        gyro_type = 1;
        gyro_address = 0x69;
      }
    }
    
    if(gyro_type == 0){
      Serial.println(F("Searching for ITG3200D on address 0x68/104"));
      delay(1000);
      if(search_gyro(0x68, 0x00) == 0x68){
        Serial.println(F("ITG3200D found on address 0x68"));
        gyro_type = 2;
        gyro_address = 0x68;
      }
    }
    
    if(gyro_type == 0){
      Serial.println(F("Searching for ITG3200D on address 0x69/105"));
      delay(1000);
      if(search_gyro(0x69, 0x00) == 0xD3){
        Serial.println(F("ITG3200D found on address 0x69"));
        gyro_type = 2;
        gyro_address = 0x69;
      }
    }
    
    if(gyro_type == 0){
      Serial.println(F("No gyro device found!!! "));
      error = 1;
    }
    
    else{
      delay(3000);
      Serial.println(F(""));
      Serial.println(F("==================================================="));
      Serial.println(F("Gyro register settings"));
      Serial.println(F("==================================================="));
      start_gyro(); //Setup the gyro for further use
    }
  }
  
  //If the gyro is found we can setup the correct gyro axes.
  if(error == 0){
    delay(3000);
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro calibration"));
    Serial.println(F("==================================================="));
    Serial.println(F("Don't move the quadcopter!! Calibration starts in 3 seconds"));
    delay(3000);
    Serial.println(F("Calibrating the gyro, this will take +/- 8 seconds"));
    Serial.print(F("Please wait"));
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++){              //Take 2000 readings for calibration.
      if(cal_int % 100 == 0)Serial.print(F("."));                //Print dot to indicate calibration.
      gyro_signalen();                                           //Read the gyro output.
      gyro_axis_cal[1] += gyro_axis[1];                          //Add gyro_x value to gyro_x_cal.
      gyro_axis_cal[2] += gyro_axis[2];                          //Add gyro_y value to gyro_y_cal.
      gyro_axis_cal[3] += gyro_axis[3];                          //Add gyro_z value to gyro_z_cal.
      delay(4);                                                  //Wait 3 milliseconds before the next loop.
    }
    //Now that we have 2000 measures, we need to divide by 2000 to get the average gyro offset.
    gyro_axis_cal[1] /= 2000;                                    //Divide the gyro_x_cal total by 2000.
    gyro_axis_cal[2] /= 2000;                                    //Divide the gyro_y_cal total by 2000.
    gyro_axis_cal[3] /= 2000;                                    //Divide the gyro_z_cal total by 2000.
    
    //Show the calibration results
    Serial.println(F(""));
    Serial.print(F("Axis 1 offset="));
    Serial.println(gyro_axis_cal[1]);
    Serial.print(F("Axis 2 offset="));
    Serial.println(gyro_axis_cal[2]);
    Serial.print(F("Axis 3 offset="));
    Serial.println(gyro_axis_cal[3]);
    Serial.println(F(""));
    
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro axes configuration"));
    Serial.println(F("==================================================="));
    
    //Detect the left wing up movement
    Serial.println(F("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));
    //Check axis movement
    check_gyro_axes(1);
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(roll_axis & 0b00000011);
      if(roll_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();

      //Detect the nose up movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds"));
      //Check axis movement
      check_gyro_axes(2);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(pitch_axis & 0b00000011);
      if(pitch_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();
      
      //Detect the nose right movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Rotate the nose of the quadcopter 45 degree to the right within 10 seconds"));
      //Check axis movement
      check_gyro_axes(3);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(yaw_axis & 0b00000011);
      if(yaw_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();
    }
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("LED test"));
    Serial.println(F("==================================================="));
    digitalWrite(LED, HIGH);
    Serial.println(F("The LED should now be lit"));
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    check_to_continue();
    digitalWrite(LED, LOW);
  }
  
  Serial.println(F(""));
  
  if(error == 0){
    Serial.println(F("==================================================="));
    Serial.println(F("Final setup check"));
    Serial.println(F("==================================================="));
    delay(1000);
    if(receiver_check_byte == 0b00001111){
      Serial.println(F("Receiver channels ok"));
    }
    else{
      Serial.println(F("Receiver channel verification failed!!! (ERROR 6)"));
      error = 1;
    }
    delay(1000);
    if(gyro_check_byte == 0b00000111){
      Serial.println(F("Gyro axes ok"));
    }
    else{
      Serial.println(F("Gyro exes verification failed!!! (ERROR 7)"));
      error = 1;
    }
  }     
  
  if(error == 0){
    //If all is good, store the information in the EEPROM
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Storing EEPROM information"));
    Serial.println(F("==================================================="));
    Serial.println(F("Writing EEPROM"));
    delay(1000);
    Serial.println(F("Done!"));
    EEPROM.write(0, center_channel[1] & 0b11111111);
    EEPROM.write(1, center_channel[1] >> 8);
    EEPROM.write(2, center_channel[2] & 0b11111111);
    EEPROM.write(3, center_channel[2] >> 8);
    EEPROM.write(4, center_channel[3] & 0b11111111);
    EEPROM.write(5, center_channel[3] >> 8);
    EEPROM.write(6, center_channel[4] & 0b11111111);
    EEPROM.write(7, center_channel[4] >> 8);
    EEPROM.write(8, high_channel[1] & 0b11111111);
    EEPROM.write(9, high_channel[1] >> 8);
    EEPROM.write(10, high_channel[2] & 0b11111111);
    EEPROM.write(11, high_channel[2] >> 8);
    EEPROM.write(12, high_channel[3] & 0b11111111);
    EEPROM.write(13, high_channel[3] >> 8);
    EEPROM.write(14, high_channel[4] & 0b11111111);
    EEPROM.write(15, high_channel[4] >> 8);
    EEPROM.write(16, low_channel[1] & 0b11111111);
    EEPROM.write(17, low_channel[1] >> 8);
    EEPROM.write(18, low_channel[2] & 0b11111111);
    EEPROM.write(19, low_channel[2] >> 8);
    EEPROM.write(20, low_channel[3] & 0b11111111);
    EEPROM.write(21, low_channel[3] >> 8);
    EEPROM.write(22, low_channel[4] & 0b11111111);
    EEPROM.write(23, low_channel[4] >> 8);
    EEPROM.write(24, channel_assign[1]);
    EEPROM.write(25, channel_assign[2]);
    EEPROM.write(26, channel_assign[3]);
    EEPROM.write(27, channel_assign[4]);
    EEPROM.write(28, roll_axis);
    EEPROM.write(29, pitch_axis);
    EEPROM.write(30, yaw_axis);
    EEPROM.write(31, gyro_type);
    EEPROM.write(32, gyro_address);
    //Write the EEPROM signature
    EEPROM.write(33, 'J'); 
    EEPROM.write(34, 'M');
    EEPROM.write(35, 'B');
        
    
    //To make sure evrything is ok, verify the EEPROM data.
    Serial.println(F("Verify EEPROM data"));
    delay(1000);
    if(center_channel[1] != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
    if(center_channel[2] != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
    if(center_channel[3] != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
    if(center_channel[4] != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;
    
    if(high_channel[1] != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
    if(high_channel[2] != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
    if(high_channel[3] != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
    if(high_channel[4] != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;
    
    if(low_channel[1] != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
    if(low_channel[2] != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
    if(low_channel[3] != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
    if(low_channel[4] != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;
    
    if(channel_assign[1] != EEPROM.read(24))error = 1;
    if(channel_assign[2] != EEPROM.read(25))error = 1;
    if(channel_assign[3] != EEPROM.read(26))error = 1;
    if(channel_assign[4] != EEPROM.read(27))error = 1;
    
    if(roll_axis != EEPROM.read(28))error = 1;
    if(pitch_axis != EEPROM.read(29))error = 1;
    if(yaw_axis != EEPROM.read(30))error = 1;
    if(gyro_type != EEPROM.read(31))error = 1;
    if(gyro_address != EEPROM.read(32))error = 1;
    
    if('J' != EEPROM.read(33))error = 1;
    if('M' != EEPROM.read(34))error = 1;
    if('B' != EEPROM.read(35))error = 1;
  
    if(error == 1)Serial.println(F("EEPROM verification failed!!! (ERROR 5)"));
    else Serial.println(F("Verification done"));
  }
  
  
  if(error == 0){
    Serial.println(F("Setup is finished."));
    Serial.println(F("You can now calibrate the esc's and upload the YMFC-AL code."));
  }
  else{
   Serial.println(F("The setup is aborted due to an error."));
   Serial.println(F("Check the Q and A page of the YMFC-AL project on:"));
   Serial.println(F("www.brokking.net for more information about this error."));
  }
  while(1);
}

//Search for the gyro and check the Who_am_I register
byte search_gyro(int gyro_address, int who_am_i){
  Wire.beginTransmission(gyro_address);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  timer = millis() + 100;
  while(Wire.available() < 1 && timer > millis());
  byte data = Wire.read();
  return data;
}

void start_gyro(){
  //Setup the ITG3200D
  if(gyro_type == 2){
    //On power up, the ITG-3200 defaults to the internal oscillator.
    //It is highly recommended that the device is configured to use one of the gyros 
    //(or an external clock) as the clock reference, due to the improved stability.
    
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro
    Wire.write(0x3E);                                            //PW_MGM register
    Wire.write(0x03);                                            //Set the register bits as 00000011
    Wire.endTransmission();                                      //End the transmission

    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro
    Wire.write(0x3E);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 1);                           //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x3E is set to:"));
    Serial.println(Wire.read(),BIN);
    
    //Set full scale to 2000 dps and DLPF to 8kHz.
    //Sample rate divider is set to 0 by default.
    
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro
    Wire.write(0x16);                                            //DLPF_CONFIG register
    Wire.write(0x18);                                            //Set the register bits as 00011000 (2000 dps full scale)
    Wire.endTransmission();                                      //End the transmission

    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro
    Wire.write(0x16);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 1);                           //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x16 is set to:"));
    Serial.println(Wire.read(),BIN);
    }
    
  //Setup the MPU-6050
  if(gyro_type == 1){
    
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro
    Wire.write(0x6B);                                            //PWR_MGMT_1 register
    Wire.write(0x00);                                            //Set to zero to turn on the gyro
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro
    Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 1);                           //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro
    Wire.write(0x1B);                                            //GYRO_CONFIG register
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500 dps full scale)
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro (address 1101001)
    Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 1);                           //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
}

void gyro_signalen(){
  if(gyro_type == 2){
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro
    Wire.write(0x1D);                                            //Start reading @ register 1Dh and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 6);                           //Request 6 bytes from the gyro
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    gyro_axis[1] = ((Wire.read()<<8)|Wire.read());               //Read high and low part of the angular data
    gyro_axis[2] = ((Wire.read()<<8)|Wire.read());               //Read high and low part of the angular data
    gyro_axis[3] = ((Wire.read()<<8)|Wire.read());               //Read high and low part of the angular data
    
    if(cal_int == 2000){                                         
      gyro_axis[1] -= gyro_axis_cal[1];                          //Compensate only after calibration
      gyro_axis[2] -= gyro_axis_cal[2];                          //Compensate only after calibration
      gyro_axis[3] -= gyro_axis_cal[3];                          //Compensate only after calibration
    }
  }
  if(gyro_type == 1){
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro
    Wire.write(0x43);                                            //Start reading @ register 43h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address,6);                            //Request 6 bytes from the gyro
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    gyro_axis[1]=Wire.read()<<8|Wire.read();                     //Read high and low part of the angular data
    gyro_axis[2]=Wire.read()<<8|Wire.read();                     //Read high and low part of the angular data
    gyro_axis[3]=Wire.read()<<8|Wire.read();                     //Read high and low part of the angular data

    if(cal_int == 2000){                                         
      gyro_axis[1] -= gyro_axis_cal[1];                          //Compensate only after calibration
      gyro_axis[2] -= gyro_axis_cal[2];                          //Compensate only after calibration
      gyro_axis[3] -= gyro_axis_cal[3];                          //Compensate only after calibration
    }
  }
}

//Check if a receiver input value is changing within 30 seconds
void check_receiver_inputs(byte movement){
  byte trigger = 0;
  int pulse_length;
  timer = millis() + 30000;
  while(timer > millis() && trigger == 0){
    delay(250);
    if(receiver_input_pin[1] > 1750 || receiver_input_pin[1] < 1250){
      trigger = 1;
      receiver_check_byte |= 0b00000001;
      pulse_length = receiver_input_pin[1];
    }
    if(receiver_input_pin[2] > 1750 || receiver_input_pin[2] < 1250){
      trigger = 2;
      receiver_check_byte |= 0b00000010;
      pulse_length = receiver_input_pin[2];
    }
    if(receiver_input_pin[3] > 1750 || receiver_input_pin[3] < 1250){
      trigger = 3;
      receiver_check_byte |= 0b00000100;
      pulse_length = receiver_input_pin[3];
    }
    if(receiver_input_pin[4] > 1750 || receiver_input_pin[4] < 1250){
      trigger = 4;
      receiver_check_byte |= 0b00001000;
      pulse_length = receiver_input_pin[4];
    } 
  }
  if(trigger == 0){
    error = 1;
    Serial.println(F("No stick movement detected in the last 30 seconds!!!"));
  }
  //Assign the stick to the function.
  else{
    //throttle===========================================================
    if(movement == 1){ 
      channel_assign[3] = trigger;
      channel_input_pin[3] = trigger;
      if(pulse_length < 1250)channel_assign[3] += 0b10000000;
    }
    //roll===============================================================
    if(movement == 2){
      channel_assign[1] = trigger;
      channel_input_pin[1] = trigger;
      if(pulse_length < 1250)channel_assign[1] += 0b10000000;
    }
    //pitch==============================================================
    if(movement == 3){
      channel_assign[2] = trigger;
      channel_input_pin[2] = trigger;
      if(pulse_length < 1250)channel_assign[2] += 0b10000000;
    }
    //yaw================================================================
    if(movement == 4){
      channel_assign[4] = trigger;
      channel_input_pin[4] = trigger;
      if(pulse_length < 1250)channel_assign[4] += 0b10000000;
    }
  }
}

//checks if pitch stick is moved down and back to center
void check_to_continue(){           
  byte continue_byte = 0;
  while(continue_byte == 0){
    if(channel_assign[2] == 0b00000001 && receiver_input_pin[1] > center_channel[1] + 150)continue_byte = 1;     //if pitch is digital input 8
    if(channel_assign[2] == 0b10000001 && receiver_input_pin[1] < center_channel[1] - 150)continue_byte = 1;     //if pitch is digital input 8
    if(channel_assign[2] == 0b00000010 && receiver_input_pin[2] > center_channel[2] + 150)continue_byte = 1;     //if pitch is digital input 9
    if(channel_assign[2] == 0b10000010 && receiver_input_pin[2] < center_channel[2] - 150)continue_byte = 1;     //if pitch is digital input 9
    if(channel_assign[2] == 0b00000011 && receiver_input_pin[3] > center_channel[3] + 150)continue_byte = 1;     //if pitch is digital input 10
    if(channel_assign[2] == 0b10000011 && receiver_input_pin[3] < center_channel[3] - 150)continue_byte = 1;     //if pitch is digital input 10
    if(channel_assign[2] == 0b00000100 && receiver_input_pin[4] > center_channel[4] + 150)continue_byte = 1;     //if pitch is digital input 11
    if(channel_assign[2] == 0b10000100 && receiver_input_pin[4] < center_channel[4] - 150)continue_byte = 1;     //if pitch is digital input 11
    delay(100);
  }
  wait_sticks_zero();        
}

//Check if the transmitter sticks are in the central positions
void wait_sticks_zero(){
  byte zero = 0;
  while(zero < 15){
    if(receiver_input_pin[1] < center_channel[1] + 20 && receiver_input_pin[1] > center_channel[1] - 20)zero |= 0b00000001;
    if(receiver_input_pin[2] < center_channel[2] + 20 && receiver_input_pin[2] > center_channel[2] - 20)zero |= 0b00000010;
    if(receiver_input_pin[3] < center_channel[3] + 20 && receiver_input_pin[3] > center_channel[3] - 20)zero |= 0b00000100;
    if(receiver_input_pin[4] < center_channel[4] + 20 && receiver_input_pin[4] > center_channel[4] - 20)zero |= 0b00001000;
    delay(100);
  }
}

//Check if the receiver values are valid within 10 seconds
void wait_for_receiver(){
  byte zero = 0;
  timer = millis() + 10000;
  while(timer > millis() && zero < 15){
    if(receiver_input_pin[1] < 2100 && receiver_input_pin[1] > 900)zero |= 0b00000001;
    if(receiver_input_pin[2] < 2100 && receiver_input_pin[2] > 900)zero |= 0b00000010;
    if(receiver_input_pin[3] < 2100 && receiver_input_pin[3] > 900)zero |= 0b00000100;
    if(receiver_input_pin[4] < 2100 && receiver_input_pin[4] > 900)zero |= 0b00001000;
    delay(500);
    Serial.print(F("."));
  }
  if(zero == 0){            //no valid receiver signals found within 10 seconds
    error = 1;
    Serial.println(F("."));
    Serial.println(F("No valid receiver signals found!!!"));
  }
  else Serial.println(F(" OK"));
}

//Register the min and max receiver values and exit when the sticks are back in the neutral position
void register_min_max(){
  byte zero = 0;
  //initialize low_channel for the following if statement to work
  low_channel[1] = receiver_input_pin[1];
  low_channel[2] = receiver_input_pin[2];
  low_channel[3] = receiver_input_pin[3];
  low_channel[4] = receiver_input_pin[4];
  while(receiver_input_pin[1] < center_channel[1] + 20 && receiver_input_pin[1] > center_channel[1] - 20);  //wait till movement starts
  Serial.println(F("Measuring endpoints...."));
  while(zero < 15){    //stop when all sticks are at centre positions
    if(receiver_input_pin[1] < center_channel[1] + 20 && receiver_input_pin[1] > center_channel[1] - 20)zero |= 0b00000001;  
    if(receiver_input_pin[2] < center_channel[2] + 20 && receiver_input_pin[2] > center_channel[2] - 20)zero |= 0b00000010;
    if(receiver_input_pin[3] < center_channel[3] + 20 && receiver_input_pin[3] > center_channel[3] - 20)zero |= 0b00000100;
    if(receiver_input_pin[4] < center_channel[4] + 20 && receiver_input_pin[4] > center_channel[4] - 20)zero |= 0b00001000;
    if(receiver_input_pin[1] < low_channel[1])low_channel[1] = receiver_input_pin[1];
    if(receiver_input_pin[2] < low_channel[2])low_channel[2] = receiver_input_pin[2];
    if(receiver_input_pin[3] < low_channel[3])low_channel[3] = receiver_input_pin[3];
    if(receiver_input_pin[4] < low_channel[4])low_channel[4] = receiver_input_pin[4];
    if(receiver_input_pin[1] > high_channel[1])high_channel[1] = receiver_input_pin[1];
    if(receiver_input_pin[2] > high_channel[2])high_channel[2] = receiver_input_pin[2];
    if(receiver_input_pin[3] > high_channel[3])high_channel[3] = receiver_input_pin[3];
    if(receiver_input_pin[4] > high_channel[4])high_channel[4] = receiver_input_pin[4];
    delay(100);
  }
}

//Check if the angular position of a gyro axis is changing within 10 seconds
//movement values:
// 1 : throttle
// 2 : roll
// 3 : pitch
// 4 : yaw
void check_gyro_axes(byte movement){
  byte trigger_axis = 0;
  float gyro_angle_axis[3];
  //Reset all axes
  gyro_angle_axis[1] = 0;
  gyro_angle_axis[2] = 0;
  gyro_angle_axis[3] = 0;
  gyro_signalen();
  timer = millis() + 10000;    
  while(timer > millis() && gyro_angle_axis[1] > -30 && gyro_angle_axis[1] < 30 && gyro_angle_axis[2] > -30 && gyro_angle_axis[2] < 30 && gyro_angle_axis[3] > -30 && gyro_angle_axis[3] < 30){
    gyro_signalen();
    if(gyro_type == 2){
      gyro_angle_axis[1] += gyro_axis[1] * 0.0002782;              //0.0002782 = 1/ 14.375 (LSB degr/s) / 250(Hz)
      gyro_angle_axis[2] += gyro_axis[2] * 0.0002782;
      gyro_angle_axis[3] += gyro_axis[3] * 0.0002782;
    }
    if(gyro_type == 1){
      gyro_angle_axis[1] += gyro_axis[1] * 0.0000611;              // 0.0000611 = 1 / 65.5 (LSB degr/s) / 250(Hz)
      gyro_angle_axis[2] += gyro_axis[2] * 0.0000611;
      gyro_angle_axis[3] += gyro_axis[3] * 0.0000611;
    }
    
    delayMicroseconds(3700); //Loop is running @ 250Hz. +/-300us is used for communication with the gyro
  }
  //Assign the moved axis to the corresponding function (pitch, roll, yaw)
  if((gyro_angle_axis[1] < -30 || gyro_angle_axis[1] > 30) && gyro_angle_axis[2] > -30 && gyro_angle_axis[2] < 30 && gyro_angle_axis[3] > -30 && gyro_angle_axis[3] < 30){
    gyro_check_byte |= 0b00000001;
    if(gyro_angle_axis[1] < 0)trigger_axis = 0b10000001;
    else trigger_axis = 0b00000001;
  }
  if((gyro_angle_axis[2] < -30 || gyro_angle_axis[2] > 30) && gyro_angle_axis[1] > -30 && gyro_angle_axis[1] < 30 && gyro_angle_axis[3] > -30 && gyro_angle_axis[3] < 30){
    gyro_check_byte |= 0b00000010;
    if(gyro_angle_axis[2] < 0)trigger_axis = 0b10000010;
    else trigger_axis = 0b00000010;
  }
  if((gyro_angle_axis[3] < -30 || gyro_angle_axis[3] > 30) && gyro_angle_axis[1] > -30 && gyro_angle_axis[1] < 30 && gyro_angle_axis[2] > -30 && gyro_angle_axis[2] < 30){
    gyro_check_byte |= 0b00000100;
    if(gyro_angle_axis[3] < 0)trigger_axis = 0b10000011;
    else trigger_axis = 0b00000011;
  }
  
  if(trigger_axis == 0){
    error = 1;
    Serial.println(F("No angular motion is detected in the last 10 seconds!!!"));
  }
  else
  if(movement == 1)roll_axis = trigger_axis;
  if(movement == 2)pitch_axis = trigger_axis;
  if(movement == 3)yaw_axis = trigger_axis;
  
}

//This interrupt routine is called every time input 8, 9, 10 or 11 changed state
//Update last channel state and timer of each channel
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                        //Is input 8 high?
    if(pin_last_state[1] == 0){                                                //Input 8 changed from 0 to 1
      pin_last_state[1] = 1;                                                   //Remember current input state
      timer_[1] = current_time;                                                //Store the current time in timer
    }
  }
  else if(pin_last_state[1] == 1){                                             //Input 8 is not high and changed from 1 to 0
    pin_last_state[1] = 0;                                                     //Remember current input state
    receiver_input_pin[1] = current_time - timer_[1];                          //Channel 1 pulse length is current_time - timer_[1]
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                       //Is input 9 high?
    if(pin_last_state[2] == 0){                                                //Input 9 changed from 0 to 1
      pin_last_state[2] = 1;                                                   //Remember current input state
      timer_[2] = current_time;                                                //Store the current time in timer
    }
  }
  else if(pin_last_state[2] == 1){                                             //Input 9 is not high and changed from 1 to 0
    pin_last_state[2] = 0;                                                     //Remember current input state
    receiver_input_pin[2] = current_time - timer_[2];                          //Channel 2 pulse length is current_time - timer_[2]
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                       //Is input 10 high?
    if(pin_last_state[3] == 0){                                                //Input 10 changed from 0 to 1
      pin_last_state[3] = 1;                                                   //Remember current input state
      timer_[3] = current_time;                                                //Store the current time in timer
    }
  }
  else if(pin_last_state[3] == 1){                                             //Input 10 is not high and changed from 1 to 0
    pin_last_state[3] = 0;                                                     //Remember current input state
    receiver_input_pin[3] = current_time - timer_[3];                          //Channel 3 pulse length is current_time - timer_[3]

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                       //Is input 11 high?
    if(pin_last_state[4] == 0){                                                //Input 11 changed from 0 to 1
      pin_last_state[4] = 1;                                                   //Remember current input state
      timer_[4] = current_time;                                                //Store the current time in timer
    }
  }
  else if(pin_last_state[4] == 1){                                             //Input 11 is not high and changed from 1 to 0
    pin_last_state[4] = 0;                                                     //Remember current input state
    receiver_input_pin[4]  = current_time - timer_[4];                         //Channel 4 pulse length is current_time - timer_[4]
  }
}

//Intro subroutine
void intro(){
  Serial.println(F("==================================================="));
  delay(1000);
  Serial.println(F(""));
  Serial.println(F("ITSP 2K17"));
  delay(500);
  Serial.println(F("ANTIMONY FLIGHT CONTROLLER"));
  delay(1000);
  Serial.println(F(""));
  Serial.println(F("INITIALIZING THE SETUP PROGRAM"));
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  delay(1000);
}

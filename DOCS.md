# ITSP PROJECT DOCUMENTATION

I am using Arduino Due in my project. I recently bought it from Amazon.

## GETTING STARTED WITH ARDUINO DUE

First I decided to start learning arduino due using Arduino IDE. Just for testing, I uploaded an arduino code to extract data out of MPU 6050 using i2c bus and display it through Serial display. On the very first run everything works fine. But when I removed the programming cable and reattached it to the PC and opened the serial monitor, I didn’t get any data. I rechecked everything and still got the same problem. So, I started debugging the code to find if the program is going into infinite loop or something else is wrong.

What I found was the code was going into infinite loop in the following line of code

```cpp
Wire.requestFrom(0x68 , 14);
while (Wire.available() < 14) Serial.println('.'); //Debug
```

On searching online, I found that many people have been complaining about problems regarding the i2c communication in arduino due. I found many arduino forums discussing the same. I tried to search how to fix the problem but I couldn’t get any proper solution. I found some chunks of code which manipulated register settings which I didn’t understand.

Rather than looking up at the internal header files of Arduino, that may be causing the problem, I decided to switch to Atmel studio so that I can understand the register settings and port manipulation better. Also the header files of Arduino IDE contain many lines of codes that are irrelevant for the user and can sometimes cause head-scratching problems like the one above.

Arduino Due runs on `atsam3x8e` or simply `sam3x8e`, an ARM microprocessor. Unlike AVR processors such as atmega328p (on Adruino Uno), ARM Processors run at 3.3V logic level, are faster and more complex. I downloaded the `sam3x8e` datasheet from [here](http://www.atmel.com/Images/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf) and an unofficial Arduino Due pin out diagram from [here](http://www.robgray.com/temp/Due-pinout.pdf). The official Arduino Due schematic is available [here](https://www.arduino.cc/en/uploads/Main/arduino-Due-schematic.pdf).  For my project, the things I needed to run on due was to:

- Find a way to `UPLOAD` programs from Atmel studio to `sam3x8e` without using any external programmer
- Make `sam3x8e` run at `FULL CLOCK SPEED` i.e. 84MHz
- Declare Pins as `INPUT` / `OUTPUT` and drive the required logic level
- Create `TIMER` functions similar to `micros()` or `delay()` (as in Arduino IDE) to create delays of specific time interval
- Activate `TWI` Interface to communicate with I2C sensors
- Activate `UART` Function to communicate with the `PC` (Serial Monitor of Arduino IDE)
- Activate and enable `INTERRUPT` service to generate interrupt whenever state of pins toggle
- Activate `ANALOG` Pins and use them as `INPUT`

And the following points are why I need them in my project. Some of them are obvious, so I marked them obvious. 
- `UPLOAD`: Obvious.
- `FULL CLOCK SPEED`: The faster, the better off course. 
- `INPUT` / `OUTPUT`: Obvious.
- `TIMER`: Obvious.
- `TWI`: Most of the sensors in my project use I2C Interface.
- `UART`: To print data in the Serial Monitor and to talk with the GPS. 
- `INTERRUPT`: To read receiver signals while flying.
- `ANALOG`: To determine battery voltage while flying.

## FIND A WAY TO UPLOAD PROGAMS FROM ATMEL STUDIO WITHOUT EXTERNAL PROGRAMMER

Arduino Due has two ports: The Programming Port (`ATMEGA16U2`) and the Native USB Port (`SAM3X`).
At first, I tried to copy the way Arduino IDE uploads the program to `atsam3xe` via Programming Port (`Atmega16U2`). I checked the “Show Verbose Output during Upload” of the Arduino IDE and copied the “Commands and Arguments” to create a new external tool in the Atmel Studio. Although it worked for Arduino Uno and Leonardo (AVR Processor based boards), it didn’t work for Arduino Due. I am not mentioning the exact way I did it, as it didn’t work anyway. What I learnt was: just like `avrdude.exe` is required to program AVR Processors, `bossac.exe` is required to program ARM Processors.

As I couldn’t figure out how to upload code via Programming Port, I switched to Native USB Port. I found this [website](http://www.elecrom.com/program-arduino-due-atmel-studio/) which neatly describes how to upload via Native USB Port. I followed the steps and it worked just fine.


## MAKE SAM3X8E RUN AT 84MHz

On creating a new project on Atmel Studio and selecting device as `sam3x8e`, in the main program, a function called `SystemInit()` was called by default. On searching what it does, I found that it configures the system to run at 84MHz. The code in the declaration of `SystemInit()` is well commented and can be easily understood by referring the datasheet.


## DECLARE PINS AS OUTPUT AND RUN REQUIRED LOGIC LEVEL

Going through the [datasheet](http://www.atmel.com/Images/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf), under section 31. Parallel Input/Output Controller, I found all the register settings needed to declare a pin as output. To declare pin 13(which is connected to on-board LED) of Arduino Due which is pin B27 of sam3x8e (see the [pin out](http://www.robgray.com/temp/Due-pinout.pdf)), the code is:

```cpp   
PIOB->PIO_PER |= PIO_PER_P27;              //PIO Enable Register (not required though)
PIOB->PIO_OER |= PIO_OER_P27 ;             //Output Enable Register  
```

The first line is not required as PIO Controller is enabled by default after reset. But if the pin is configured to work as a peripheral (specially assigned functions), the `PIO` Controller needs to be enabled first.

To drive it high:

```cpp
PIOB->PIO_SODR |= PIO_SODR_P27;            //Set Output Data Register
```

To drive it low:

```cpp
PIOB->PIO_CODR |= PIO_CODR_P27;            //Clear Output Data Register
```

## DECLARE PINS AS INPUT AND READ THEIR LOGIC LEVEL

Going further in the same section as above, I found the register settings for declaring pins as input. To declare pin 12 of Arduino Due which is pin D8 of sam3x8e (see the [pin out](http://www.robgray.com/temp/Due-pinout.pdf)), the code is:

```cpp
PIOD->PIO_PER |= PIO_PER_P8;               //PIO Enable Register (not required though)
PIOD->PIO_ODR |= PIO_ODR_P8;               //Output Disable Register  
```

The status of the pin (`HIGH` or `LOW`) can be read from the `PDSR` (Pin Data Status Register). In the following code, the value of state is 1 when pin 12 is `HIGH`, and 0 when pin 12 is `LOW`.

```cpp
uint16_t state = (PIOD->PIO_PDSR & PIO_PDSR_P8)>>8;  //Read the PIO_PDSR register
```

But the code didn’t work at all. I rechecked the [datasheet](http://www.atmel.com/Images/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf) and found that unlike just declaring pins as input/output, there was an additional instruction that said: 

***"Reading the I/O line levels requires the clock of the `PIO` controller to be enabled, otherwise `PIO_PDSR` reads the levels present on the I/O line at the time the clock was disabled."***

I searched the datasheet, but didn’t find a clue how to enable the clock. So I advanced further, and while writing the code for UART initialization, I stumbled upon this [forum](http://forum.arduino.cc/index.php?topic=179431.msg1342932#msg1342932), where someone mentioned about enabling the peripheral clock for UART. So I referred to the Peripheral Identifier section of the datasheet and found a list of `PID`s (Peripheral Identifiers) and their corresponding peripheral. I found the peripheral `PIOD` with `PID` 14. Then I included the line of code (mentioned below) in my ‘declaring pins as input ‘ program, and pin 12 is successfully declared as input and everything works fine. `PCER0` stands for Peripheral Clock Enable Register 0.

```cpp
PMC->PMC_PCER0 |= PMC_PCER0_PID14;             //Enable PIOD Clock
```

Afterwards, I found that the PCER0 register is actually write protected. Zero needs to be written in the WPEN bit (Write Protect Enable) to disable the Write Protect Mode. 

But my code still runs without any problem. Maybe the Write Protect Mode was already undone by `SystemInit()`. As my code was working anyway, I didn’t bother to check up. And following is the code to disable the Write Protect Mode.

```cpp
PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);            //Disable the Write Protect Mode
```

## CREATE TIMER FUNCTIONS TO CREATE DELAYS OF SPECIFIC INTERVALS

In my project, I need a precision of at most 1 us. So I need a clock frequency of at least 1 MHz. There are two sections in the datasheet through which we can create such timers. One is Section 13 `RTT` (Real Time Timer) and the other is Section 36 `TC` (Timer Counter). 

The `RTT` runs through the `SWCLK` (`SLOW CLOCK`) that has a clock frequency of 32.768 kHz, thus, is not suitable for this project.

The `TC` can be made to run from 5 internal clock sources, that are `MCK/2`, `MCK/8`, `MCK/32`, `MCK/128` and `SWCLK` (`MCK` refers to `MAIN CLOCK`). As sam3x8e is running at 84 MHz (MCK), the usable frequencies are `MCK/2`, `MCK/8` and `MCK/32`. I chose to use `MCK/8` or 10.5 MHz for my clock frequency. 

I found this [blog](http://ko7m.blogspot.nl/2015/01/arduino-due-timers-part-1.html) which may be useful. The following is the code used for setting up the timer.

```cpp
//Configure PMC
//Disable the Write Protect Mode
PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);        

//Enable TC0 Peripheral Clock       
PMC->PMC_PCER0 |= PMC_PCER0_PID27;               

//Disable the Write Protect Mode
TC0->TC_WPMR &= ~(TC_WPMR_WPEN);             
    
//Set Clock Source to MCK/8
TC0->TC_CHANNEL[0].TC_CMR |= TC_CMR_TCCLKS_TIMER_CLOCK2;            

//Set Wave select to updown
TC0->TC_CHANNEL[0].TC_CMR |= TC_CMR_WAVE | TC_CMR_WAVSEL_UPDOWN;    

//Enable Clock and trigger to start counter
TC0->TC_CHANNEL[0].TC_CCR |= TC_CCR_CLKEN | TC_CCR_SWTRG;    
```

The following line of code is used to load the real time counter value onto a variable.

```cpp
//Read current value from Counter Value Register
uint32_t counter = TC0->TC_CHANNEL[0].TC_CV;          
```

## ACTIVATE TWI INTERFACE TO COMMUNICATE WITH I2C SENSORS

This part is described in the datasheet under Section 33 Two-Wire Interface (`TWI`).There are two `TWI` channels. I’m using the second channel (`TWI1`) which are pins `SDA` and `SCL` of Arduino Due and  pins `B12` and `B13` of `sam3x8e` (see the [pin out](http://www.robgray.com/temp/Due-pinout.pdf)). 

First of all, I programmed the `PIO` controller to dedicate `TWD` and `TWCK` as peripheral lines. Then, enabled the peripheral clock. And set the mode of operation to Master Mode. The steps of using `TWI` in different modes are neatly displayed in a flowchart at the end of the `TWI` section.

The line of code to set up the speed to 400 kHz actually overwrites the line of code above it which sets the speed to 100 kHz. To run at 100 kHz, just uncomment the line corresponding to 400 kHz.

The following is the code to set up the `I2C` interface at 400 kHz. Note that the prefix `0x` is to represent the number as hexadecimal.

```cpp
//Disable PIO Controller
PIOB->PIO_PDR |= PIO_PDR_P12 | PIO_PDR_P13;          
//Peripheral A selected by default

//Disable the Write Protect Mode
PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);                                  

//Enable TWI peripheral Clock
PMC->PMC_PCER0 |= PMC_PCER0_PID23;

//Wave Generator - Set TWI Clock to 100kHz
TWI1->TWI_CWGR = 0;
TWI1->TWI_CWGR = TWI_CWGR_CKDIV(1)|TWI_CWGR_CHDIV(0xD4)|TWI_CWGR_CLDIV(0xD4);

//Wave Generator - Set TWI Clock to 400kHz
TWI1->TWI_CWGR = 0;
TWI1->TWI_CWGR = TWI_CWGR_CKDIV(0)|TWI_CWGR_CHDIV(0x65)|TWI_CWGR_CLDIV(0x65);

//SVDIS: Disable the slave mode. MSEN: Enable the master mode.
TWI1->TWI_CR |= TWI_CR_SVDIS | TWI_CR_MSEN;
TWI1->TWI_MMR = 0;

//Write the Device i2c address.
TWI1->TWI_MMR |= TWI_MMR_DADR(0x68);
```

The following is the code to read a single byte from `0x75` (`WHO_AM_I`) Register of an `I2C` device with device address `0x68` (`MPU 6050`).

```cpp
//Set Transfer Direction Bit to 1, and internal address size to 1 byte
TWI1->TWI_MMR |= TWI_MMR_IADRSZ_1_BYTE | TWI_MMR_MREAD;

//The Internal Address
TWI1->TWI_IADR = TWI_IADR_IADR(0x75);

//Read Single Byte
TWI1->TWI_CR |= TWI_CR_START | TWI_CR_STOP;

//Read Status register, wait until RXRDY is 1
while(!(TWI1->TWI_SR & TWI_SR_RXRDY));

//Read Receive Holding register
data = TWI1->TWI_RHR;

//Read Status Register
while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));	
```

The following is the code to read multiple byte from `0x3B` (`ACC_X_HIGH_BYTE`) Register of an `I2C` device with device address `0x68` (`MPU 6050`).

```cpp
//The 'device address' is used to access slave, set Transfer Direction Bit to 1
TWI1->TWI_MMR |= TWI_MMR_IADRSZ_1_BYTE | TWI_MMR_MREAD;

//The Internal Address
TWI1->TWI_IADR = TWI_IADR_IADR(0x3B);

//START
TWI1->TWI_CR |= TWI_CR_START;
int numofbytes = 14;


for(int i = 0; i<numofbytes-1; i++){

	//Read Status register, wait until RXRDY is 1
	while(!(TWI1->TWI_SR & TWI_SR_RXRDY));

	//Read Receive Holding register
	data = TWI1->TWI_RHR;
}

//STOP
TWI1->TWI_CR |= TWI_CR_STOP;

//Read Status register, wait until RXRDY is 1
while(!(TWI1->TWI_SR & TWI_SR_RXRDY));

//Read Receive Holding register
data = TWI1->TWI_RHR;

//Read Status Register
while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));
```

The following is the code to write a single byte i.e. 0x00 from 0x00 (PWR_MGMT) Register of an I2C device with device address 0x68 (MPU 6050).

```cpp
//The 'device address' is used to access slave, set Transfer Direction Bit to 1
TWI1->TWI_MMR |= TWI_MMR_IADRSZ_1_BYTE;
TWI1->TWI_MMR &= ~(TWI_MMR_MREAD);

//The Internal Address
TWI1->TWI_IADR = TWI_IADR_IADR(0x00);

//Load data to be sent
TWI1->TWI_THR = 0x00;

//STOP
TWI1->TWI_CR |= TWI_CR_STOP;

//Read Status register, wait until TXRDY is 1
while(!(TWI1->TWI_SR & TWI_SR_TXRDY));

//Read Status Register
while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));
```

## ACTIVATE UART FUNCTION TO COMMUNICATE WITH THE PC

This part is described in the datasheet under Section 34 Universal Asynchronous Receiver Transceiver (`UART`). Pin RX0 and TX0 of Arduino Due (pins `PA8` and `PA9` of `sam3x8e`) are the `UART` pins. The following code initializes the UART communication at 57600 bps. I also posted the code in this [forum](http://forum.arduino.cc/index.php?topic=179431.msg1342932#msg1342932).

```cpp
//Enable UART Peripheral Clock
PMC->PMC_PCER0 |= PMC_PCER0_PID8;

//Initialize RX and TX pins
PIOA->PIO_PDR |= PIO_PDR_P8|PIO_PDR_P9;//Peripheral A(RX & TX) are enabled by default

//Disable PDC Channel
UART->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

//Reset and disable receiver and transmitter
UART->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

//Configure Mode
UART->UART_MR |= UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;//No Parity and normal CH mode

//Configure Rate Generator
UART->UART_BRGR |= UART_BRGR_CD(91);    //Baud rate set to 57600 bps  
	
// Configure interrupts
UART->UART_IDR = 0xFFFFFFFF;
UART->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;	

// Enable receiver and transmitter
UART->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
```

To transmit data, write data to the `THR` register and wait until the `TXRDY` bit is set in the `UART` Status Register. The following example sends A via `UART`.

```cpp
UART->UART_THR = ‘A’;                         //Write to UART_THR to send ‘A’
while(!(UART->UART_SR & UART_SR_TXRDY));      //Wait till data is sent and THR is empty
```

## ACTIVATE INTERRUPT SERVICE TO ACT WHENEVER PIN STATE TOGGLES

Using interrupts on `GPIO` pins is quite straight forward. It is described in Section 10 ARM Cortex M3 Processor under subsection: Nested Vectored Interrupt Controller (`NVIC`). First `NVIC` needs to be set up to enable Handler functions. The code to be run in the interrupt is written under the Handler functions. Here is a link that I referred. Below is the code to enable Edge Detection Interrupt on pin 12 of Arduino Due. That is pin D8 of `sam3x8e` (see the pin out). 

```cpp
//Enable Interrupts
PIOD->PIO_IER |= PIO_IER_P8;

//Configure NVIC
NVIC_EnableIRQ(PIOD_IRQn);

//Interrupt Sub Routine
void PIOD_Handler(void){

	//code inside interrupt goes here
	
//read interrupt status register
	PIOD->PIO_ISR;

}
```

## ACTIVATE ANALOG PINS AND USE THEM AS INPUT

## I2C REGISTER SETTINGS

register settings of MPU 6050:

```
Reset value of all registers: 0x00 (13 to 117)
Exceptions: Register 107 : 0x40
            Register 117 : 0x68

The device will come up in sleep mode upon power-up.

Register 1A CONFIG | Write 0x03 | Set DLPF to 43Hz for acc and gyro

Register 1B GYRO_CONFIG | Write 0x08 | Set gyro FS to +-500 dps LSB 65.5/dps | Write 0xE8 to also enable self test

Register 1C ACCEL_CONFIG | Write 0x10 | Set accel FS to +-8g LSB 4096/g| Write 0xF0 to also enable self test


Register 3B 59 | ACCEL_XOUT[15:8] 
Register 3C 60 | ACCEL_XOUT[7:0] 
Register 3D 61 | ACCEL_YOUT[15:8] 
Register 3E 62 | ACCEL_YOUT[7:0] 
Register 3F 63 | ACCEL_ZOUT[15:8] 
Register 40 64 | ACCEL_ZOUT[7:0]
Register 41 65 | TEMP_OUT[15:8]        Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
Register 42 66 | TEMP_OUT[7:0]         The scale factor and offset for the temperature sensor are found in the Electrical Specifications table (Section 6.4 of the MPU-6000/MPU-6050 Product Specification document).
Register 43 67 | GYRO_XOUT[15:8] 
Register 44 68 | GYRO_XOUT[7:0]
Register 45 69 | GYRO_XOUT[15:8] 
Register 46 70 | GYRO_XOUT[7:0]
Register 47 71 | GYRO_XOUT[15:8] 
Register 48 72 | GYRO_XOUT[7:0]

Register 0x68 SIGNAL_PATH_RESET | Write 0x03 to reset

Register 0x6A USER_CONTROL | Write 0x01 to reset

Register 0x6B PWR_MGMT_1 | Write 0x80 to device_reset

Register 0x6B PWR_MGMT_1 | Write 0x00 to disable sleep mode
```

register settings of HMC5883L

```
Device i2c address: 0x1E

Read/Write Identifier :  Read  ----  0x3D
                         Write ----  0x3C

Register 0x00 CRA (Configuration Register A) 
	Default: 0x10
	Write: 0x59 B01011001
	No. of samples averaged per measurement: 4
	Typical Data Output Rate (Hz): 75
	Incorporated Positive applied bias into measurement

NOTE:  Magnitude of Magnetic Field at the Earth's surface ranges from 25 to 65 microteslas (0.25 to 0.65 gauss).

Register 0x01 CRB (Configuration Register B)
	Default: 0x20
	Sensor Field Range: +-1.3 Ga
	Gain: 1090 LSB/Ga

Register 0x02 Mode Register
	Default: 0x01
	Write: B10000000 0x80
	Continuous-Measurement Mode enabled
    High Speed i2c enabled

03  Data Output X MSB Register
04  Data Output X LSB Register
05  Data Output Z MSB Register
06  Data Output Z LSB Register
07  Data Output Y MSB Register
08  Data Output Y LSB Register


10  Identification Register A |  'H'
11  Identification Register B |  '4'
12  Identification Register C |  '3'
```

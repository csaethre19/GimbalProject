### ECE5780 Mini Project: 3-Axis Gimbal

![image](https://github.com/csaethre19/GimbalProject/assets/45043278/d43bef98-7a9f-40f5-9aa6-5e0a1bd2dede)


Group members:

- Charlotte Saethre
- Kirra Kotsenburg
- Alex Leavitt
- Nicholas Ronnie

Project Summary:
  The STM32F072 is programmed to act as a gimbal driver, handeling all inputs/outputs needed to implement a 3 axis gimbal. This requires 3 total motor drivers, as well as 3 orientation sensors in order to achieve a target orientation while simultaniously monitoring the orientation of the base of the gimbal to ensure that the mechanical bounds of the system are not violated. A UART line acts as an all inclusive interface for the setup, monitoring, and instruction of the gimbal.
Additonal modes of controlling the gimbal include both PWM and Analog voltage instruction of gimbal orientation.

The following summarizes the achieved functionality as of 4/19/2024:

  1. Unrestricted Yaw​-----------------------(24 wire Slip ring enables continuous rotation in the yaw axis)
  2. +/-80 deg. Roll, Pitch*​----------------(Current implementation of IMU limits target angle ranges from +80 deg to -80 deg. In reality the system hardware is limited to +/-180 deg)
  3. +/- 5 deg accuracy*​--------------------(Maximum deviation from target angle is estimated at around 5 degrees)
  4. UART interface-------------------------(reports all useful system information and can be used to configure settings, set desired orientation. Protocal details listed below)
  5. PWM instruction------------------------(1-2ms square waves set desired angle when PWM control is enabled)​
  6. ADC instruction------------------------(0-3.3v analog voltages set desired angle when ADC control is enabled)
  7. ~15 Wires for Customizable Payload-----(15/24 slip ring wires are not required for functionality listed above)

The following are key elements of the STM32 code, additional details are provided in sections of read-me specific to each item:

1- UART interface for gimbal control

2- PWM input for gimbal instruction

3- ADC input for gimbal instruction

4- 2 x Brushless Motor Control Signals

5- 1 x DC Motor Control Signals

6- I2C interface - 2 x MPU6050, 1 x QMC5883L

7- Current Monitoring

PCB Schematic of STM32F072: ![image](https://github.com/csaethre19/GimbalProject/assets/45043278/33eb72a9-7e3a-4762-b990-faef41e200b4)


DETAILS OF KEY ELEMENTS BELOW: 

1-----UART interface for gimbal control: <br>
  The following list of commands define the UART protocol used to read information from the system and set desired orientation and input types:
  - rdpitch         - prints the currently calculated Kalman angle pitch to console
  - rdroll          - prints the currently calculated Kalman angle roll to console
  - rdyaw           - prints the currently calculated Kalman angle yaw to console
  - wrpitch <value> - sets the desired pitch value to <value> specified via command
  - wrroll <value>  - sets the desired roll value to <value> specified via command
  - wryaw <value>   - sets the desired yaw value to <value> specified via command
  - pwm             - sets type of input to pwm, disabling the currently set type
  - adc             - sets type of input to adc, disabling the currently set type
  - pwm/adc off     - disables pwm and adc

2-----PWM input for gimbal instruction:

   The PWM inputs are mapped to a range of -80 <-> 80. This range is mapped to a square wave HIGH duration ranging from 1ms to 2ms, with a 1.5ms high time representing a middle point and a desired angle of 0 deg.
   Timer 15 Ch1, Ch2 and Timer 17 Ch1 are utlized in capture/compare mode to determine the time that the rising/falling edge of incoming square waves arrive. If a falling edge arrives, the time elapsed since the most recent rising edge is determined and this determined value is stored into a global variable within PWM_input.c. Note that timer 15 & timer 17 are configured identically, so the same functions can be utlized to interpret the data from both

   The PWM_input.c file implements the following functions:

    a. process_eventTime: Interrupt service routines for the Timer 15, Timer 17 capture compare events call process_eventTime function after they have determined the channel source and whether a rising/falling edge arrived. If a falling edge has arrived, use the currently stored rising edge time to determine the pulse width of the square wave.
    b. provide_channel: This function returns the druation high of the requested PWm channel (1 = Yaw, 2 = Pitch, 3 = Roll) with the provided time high in units of uS.

  STM32F072 Datasheet referred to for creation of PWM input.
   
3-----ADC input for gimbal instruction:
The ADC input is initialized with:
* Clock Prescaler - Asynchronous Clock Mode
* Data Resolution - 12-bit resolution
* Data Alignment - right alignment
* Sampling Time - 1.5 cycles
Three channels are utilized for the Gimbal, mapping to the three orientations: Roll, Pitch, and Yaw.
Roll:
* Pin # - PA5
* Channel # - Channel 5
Pitch:
* Pin # - PA4
* Channel # - Channel 4
Yaw:
* Pin # - PA3
* Channel # - Channel 3
In order to select ADC for input, type either "ADC" or "adc" into the uart terminal. ADC will be enabled and PWM disabled.

4-----2 x Brushless Motor Control Signals:

  Each Brushless Motor requires three square wave signals representing the voltage supplied to the three poles of the motor.
  For the control signals, timer 2 & timer 3 are configured for a 8MHz/4 (psc = 3) = 2MHz incrementation rate. The ARR of both is 1000, making the period of the square wave 0.5ms or a frequency of 2khz. This can be observed through the loud 
  AF buzzing produced by the BLDC motors during operation. Ch1, Ch2, Ch3 are utilized to produce the square waves within both timer 2 and timer 3.
  
These signals are output as follows:

  Timer 2: Ch1 = BLDC_1A(PA0), Ch2 = BLDC_1B(PA1), Ch3 = BLDC_1C(PA2)
  
  Timer 3: Ch1 = BLDC_2A(PA6), Ch2 = BLDC_2B(PA7), Ch3 = BLDC_2C(PB0)
  
  Motor 1 Enable Signal: PB6 (GPIO output)
  
  Motor 2 Enable Signal: PB5 (GPIO output) 
  
For FOC, the orientation of the motor is controlled by providing a desired angle on a range from 0-360. Each channel's DutyCycle is mapped to the amplitude of a sin wave with a offset of 1/3 (120 deg) of the period of the sin wave. 

Ex. Provided Angle = 20 deg. Ch1 Duty Cycle (where -1 = 0%, 1 = 100%) = sin(RADIAN(20)); Ch2 Duty Cycle = sin(RADIAN(20+120)); Ch3 Duty Cycle = sin(RADIAN(20+120+120));

It is important to note that the range of 0-360 does not represent the full rotation of the BLDC motor. Instead, the BLDC motor has 12-16 identical motor poles, where incrementally moving from 0->360 rotates the BLDC motor 1/#motorpoles through its range.

  The BLDCMotor.c file implements the following functions:
  
    a. set_desiredRoll   - Set the targeted roll angle of the system.
    b. set_desiredPitch  - Set the targeted pitch angle of the system.
    c. set_operationMode - A boolean flag is set representing if the provided target angle should be interpreted as an angle relative to the base of the gimbal, or an absolute angle.
    d. BLDC_PID - Compare the target (pitch/roll) angle to the current (pitch/roll) angle. Adjust the output FOC angle to reduce the error in the measured angle. Restriction to mechanical limits is not yet implemented, planned.
    e. BLDC_Output - Provided an Angle and a MotorNumber, set the FOC angle output to the Motor. Currently hardcoded for MotorNum = 1 = Tim2 Ch1,Ch2,Ch3. MotorNum = 2 = Tim3 Ch1, Ch2, Ch3
    f. BLDCEnable        - Provided a motor number, set the GPIO output of the corresponding enable output HIGH
    g. BLDCdisable       - Provided a motor number, set the GPIO output of the corresponding enable output LOW
    
NOTE: The P controller (called PID for some reason) for the position of the BLDC motors attempts to incrementally move the BLDC motor from the current angle towards the desired angle as fast as possible. During the period of each control loop iteration, a maximum reliable instructed change in angle was determined. This limits what is effectively the maximum "rotation rate" of the BLDC motor. Provided the previous angle delivered to the motor, limit the maximum change in angle to within a region that the motor can achieve within the period of the control loop. 
The frequency of the control loop is determined by the TIMER 1 interrupt (currently) generated at a rate of 1KHz.

PCB schematic of BLDC driver:

![image](https://github.com/csaethre19/GimbalProject/assets/45043278/0fef9ef2-f92b-4922-b066-2a6690adbd33)


 Reference Sources for BLDC control: 
 
   FOC control using L6234: https://electronoobs.com/eng_arduino_tut176.php

5-----1 x DC Motor Control Signals:

  Two PWM signals are required to set the duty cycle & direction of spin of the DC motor. These signals are passed to a DC motor driver chip called MP6550GG, which drives current through DC motor itself. These PWM signals for driving the DC motor are generated by Timer 1, using Channels 1 & 2. Output PWM signals on PA9=Ch2, PA8=Ch1.

  The code for driving the DC Motor is straightforward and is specifically tailored to modifying the CCR1/CCR2 registers of timer 1. This implementation is all done within the DCMotor.c file.
  In the current implementation 4/19/2024, the DC motor is used in an unintelligent way, simply modifying the direction/speed of the DC motor irrespective of yaw angles and operation mode.
  
  The DCMotor.c file implements the following functions:
  
    a. set_desiredYaw        - Used to set the desired yaw angle relative to the front of the mounting assembly. NOT CURRENTLY USED
    b. set_operationalModeDC - Sets a boolean flag determining if yaw attempts to maintain a yaw orientation relative to the base assembly, or relative to the magnetic north pole. NOT CURRENTLY USED
    c. DC_PID                - Feedback loop is called in order to monitor and maintain the yaw orientation of the actuated surface, based on the operation mode of the gimbal. NOT CURRENTLY USED
    d. DCSetOutput           - Provided an int in the range of -1000 <-> 1000, a duty cycle and direction for the DC motor is set within the CCR1/CCR2 registers of Timer 1.
    e. initDCOutput          - Initializes the Duty cycle of both CCR1/CCR2 channels in order to instruct the DC motor to not spin.

PCB schematic of DC Motor Driver:

![image](https://github.com/csaethre19/GimbalProject/assets/45043278/e378212f-7061-4e44-a8d6-ae8ad9dacaf7)


    NOTE: While architecture for multiple DC motors is implemented, only one DC motor is utlized (MotorNum = 1).

6-----I2C interface - 2 x MPU6050, 1 x QMC5883L:
   A single I2C (i2c2 = PB10(SCL) & PB11(SDA) is used to interface with three total slave devices sharing the same 400KHz I2C line.
   
   MPU6050: 
   
   2 Struct objects are created to encompass all data relevant to both the stationary and actuated (moving) MPU6050 sensors.
   
   The MPU6050.c file implements the following functions:
   
     a. MPU_init            - Provided with address of an MPU6050, this function confirms connection with the target device, retrieving/confirming the WHO AM I information.
        Additionaly configures control registors (gyro = +/- 1000deg/s, Accel = +/- 8g, sample rate = 1khz, enable low pass filter)
     b. ReadAccelData       - provided a MPU6050 struct, this function utilizes the address of the device to retrieve the current accelerometer data stored in the device, updating struct variables to represent most recent data.
     c. ReadGyroData        - same as ReadAccelData, instead retrieves Gyro Data
     d. CalculateAngleRoll  - Performs operation on three accelerometer values in order to determine Roll orientation of MPU6050.
     e. CalculateAnglePitch - Performs operation on three accelerometer values in order to determine Pitch orientation of MPU6050.
     f. KalmanFilter        - First attempt at Kalman filter, saved for history.
     g. KFilter_2           - Currently used function in order to retrieve most recent MPU6050 data, perform updated Kalman Filter, store a calculated KalmanFilterRoll, KalmanFilterPitch into MPU6050 struct
     h. Kalman_getAngle     - Called within KFilter_2 in order to perform Kalman Filter on provided data for Pitch/Roll data.

PCB Schematic:
![image](https://github.com/csaethre19/GimbalProject/assets/45043278/02d3a804-32c1-4fb0-bdc5-8a644b377340)

     
   Reference Sources for MPU6050 interface: 
     
KalmanFilter / ReadAccelData / ReadGyroData - https://github.com/CarbonAeronautics/Part-XV-1DKalmanFilter

KFilter_2 - https://github.com/ibrahimcahit/STM32_MPU6050_KalmanFilter

Datasheet: https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf


   QMC5883L: 
   
   Struct object encompasses all data relevant to QMC5883L ---- NOTE: file name is HMC5883L. HMC5883L is non-generic version of QMC5883L chip, slight change in register addresses, while having identical functionality, capabilities.
   
   The HMC5883L.c ffile implements the following functions:
   
     a. HMC5883_Init        - Configures two registers for sample continuous data, as well as IDK
     b. HCM5883_ReadRawData - The X and Y magnetometer readings are retrieved in order to determine the horizontal rotation of the sensor relative to magnetic north. This AngleYaw is calculated.
     
   Reference Sources for QMC5883L interface: DataSheet: http://wiki.sunfounder.cc/images/7/72/QMC5883L-Datasheet-1.0.pdf

   NOTE: QMC5883L interface never worked, and is also not required for the demonstration of any functionalities shown as of 4/19/2024. This file is not used in current implementation
  
7-----Current Monitoring:
   Three total shunt resistors are placed on the lowside of the motor driving circuits of each of the three axis. These three analog voltages are routed to PC1=Yaw, PC2=Pitch, PC3=Roll where an ADC is used to estimate the current consumption of each axis.
   NOTE: While current monitoring hardware & configuration of pins is fully established, the decision was made to not implement it in current version of software due to increased complexity of code and limited benefits within the existing highly controlled test environment. A 0 Ohm jumper is currently placed at shunt resistor locations on PCB.






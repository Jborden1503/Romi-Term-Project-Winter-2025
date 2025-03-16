**Table of content:**
 - [Romi Design](#romi-design)
   - [Hardware Design](#hardware-design)
     - [Bill of Materials](#bill-of-materials)
     - [Wiring Diagram](#wiring-diagram)
 - [Program Design and Structure](#program-design-and-structure)
   - [Tasks](#tasks)
   - [Classes](#classes)
 - [Conclusion](#conclusion)
 - [Video Demonstration](#video-demonstration)

<a id="romi-design"></a>
### Romi Design
<a id="hardware-design"></a>
### Hardware Design
For its attempts to complete the course, Romi had the help of a number of sensors. First, an Adafruit BNO055 IMU was used to keep track of headings. This was connected to the Nucleo using I2C connection. Second, a pair of Pololu bump sensors were used for detecting the wall at the end of the course. Third, a Pololu analog IR reflectance sensor was used for line following. We purchased a sensor with a total of 13 channels, but only ended up using 7 of them for simplicity.
<a id="bill-of-materials"></a>
#### Bill of Materials
A list of components used in the project.

<a id="wiring-diagram"></a>
#### Wiring Diagram
A diagram showing the wiring connections between components.

<a id="program-design-and-structure"></a>
### Program Design and Structure
Overview of how the software is organized and structured.

<a id="tasks"></a>
#### Tasks
Details on the cooperative tasks, state machines, and scheduling.

<a id="classes"></a>
#### Classes
* **Encoder:**
  Interfaces with quadrature encoder hardware to track positional changes using timer channels. It maintains internal counters, updating them through the update() method, which handles position increments, decrements, and overflow scenarios. The encoder’s position is accessed via the get_position() method, and can be reset using zero().

* **Motordriver:**
  Provides a motor driver for controlling a Romi robot. It utilizes a hardware timer channel to generate PWM signals for motor speed control while using digital output pins for direction control and enabling/disabling the motor. The set_effort() method accepts an effort value between -100 and 100, setting the PWM duty cycle accordingly and adjusting the motor direction. Additionally, it includes enable() and disable() methods to control the motor's power state. This class takes the pin names for the PWM pin, enable pin and direction pin as well as the timer number and channel for the PWM as inputs for instantiating an object of this class. 

* **IRsensor:**
  Operates an array of IR sensors for line detection and tracking. It measures analog voltage levels and calibrates sensors against defined black and white surface readings through methods set_black() and set_white(), and stores the calibration as a .csv file. Line positions are calculated with read_line(), and get_centroid() returns a weighted average indicating the line’s relative location. This class is designed to be implemented with an array of 7 sensors and will therefore require 7 ADC pins on the MCU. It takes the 7 pin names as inputs and creates ADC objects of the pyb.ADC class within the __init__ method and compiles them into a list so that the other methods can read the sensor values using for loops. 

* **PID:**
  Implements feedback control to regulate system performance, reducing errors between measured values and setpoints. The update() method computes proportional, integral, and derivative terms over time to generate an output that effectively corrects system behavior. A controller object of this class can be implemented with any task or state. It does not require motor or sensor objects. It instead takes Kp, Ki and Kd gains as well as an arbitrary setpoint.

* **Gain_compensator:**
  Adjusts motor control outputs based on real-time battery voltage readings to maintain consistent performance. By measuring battery voltage with an ADC pin, it uses correct_value() to scale the motor effort, compensating for the loss of charge that will affect motor performace. This became necssarey to avoid needing to tune the controller gains continuosly as the battery level changed. The change in battery level changes the voltage to the motors, meaning more agressive gains were needed as the battery level dropped. This class was paired with a volatage divider circuit that provided a 'pick off point' for the ADC pin to read the volatge. The was accolpished by stepping down the volatge by a factor of three using three resistaors in series to make the volatge safe for the MCU, then multiplying by three inside the method to get the actual charge on the batteries. 

* **IMU:**
  Handles communication with the Bosch BNO055 IMU sensor via the I²C protocol, offering orientation data through Euler angles. Methods such as get_heading(), get_roll(), and get_pitch() parse raw IMU data into meaningful orientation angles. It includes capabilities for sensor calibration, mode configuration, and normalization of angle measurements to handle continuous rotations. The class takes a single I2C controller object as an input. The I2C controller is an object of the pyb.I2C class built into MicroPython. 

* **Bumpsensor:**
  Detects physical collisions using an array of bump-switch inputs configured with internal pull-up resistors. The get_state() method returns a boolean status indicating whether any of the sensors have been triggered, enabling immediate collision response in the robot's control logic.

<a id="conclusion"></a>
### Conclusion
Summary of the project outcomes and lessons learned.

<a id="video-demonstration"></a>
### Video Demonstration
A link or reference to the video demonstration of the project.

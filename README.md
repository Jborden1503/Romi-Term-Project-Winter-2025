**Table of content:**
 - [Romi Design](#romi-design)
   - [Hardware Design](#hardware-design)
     - [Bill of Materials](#bill-of-materials)
     - [Wiring Diagram](#wiring-diagram)
 - [Program Design and Structure](#program-design-and-structure)
   - [Tasks](#tasks)
   - [Classes](#classes)
 - [Discussion](#discussion)
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
## Tasks

### Sensor Task

The sensor task is responsible for reading sensor values and keeping track of significant milestones on the course. This task is functionally similar to a mastermind task, but it differs chiefly by its direct interaction with external hardware. Given the simple nature of checking course progress, it was appropriate to roll this function into the sensor task to improve memory consumption. This task is run with a priority of 2 and a period of 10 ms.

The task takes an encoder object, an IR sensor object, and an IMU object. It communicates with the motor task using three shares: the centroid share, the heading share, and the checkpoint share.

- The **centroid share** is used to share the value of the centroid as computed in the sensor task. The sensor task will only put values into this share and does not need to get them out.
- The **heading share** is used to share the value of the heading as computed in the sensor task. This will be shared as a -179-0-180 degree value, while the sensor task itself uses the raw 360-degree reading to determine the robot's true orientation. This is done because it is more useful to the PID controllers in the motor task. The sensor task will only put values into this share and does not need to get them out.
- The **checkpoint share** is used as a flag to tell the motor task that it has reached the end of its current state. When the sensor task determines that the state is finished, using sensor data, a 1 is placed in the checkpoint share. The sensor task will only put a value of 1 into this share and does not read the value.

There are **12 states** in the sensor task. The transitions are determined using either encoder positions or heading angles. When the sensor value falls within the specified range, the state is considered finished, and the sensor task signals the transition using the checkpoint share and setting its own internal state variable to the next state.

The initialization of this task is done inside a run-once `if` statement above the while loop of the main generator function. This looks for a global variable called `sensor_init`, which the main function sets to `True` before starting the scheduler. Once init functions are complete, `sensor_init` is set to `False` so that the init functions will not run again. A more elegant and Pythonic implementation of this code would be to implement the task as a class, where the init functions are performed automatically upon instantiation of a sensor task object.

#### State Descriptions

- **STATE_1**: Line following through the course until checkpoint #4. The sensor task computes the centroid from the IR line sensor and sends it to the motor task via the centroid share. This state also keeps track of the encoder position looking for an encoder reading of 112 to 113 radians. Once this value is reached, the state zeroes the encoder, sets `sensor_state` to `STATE_2`, and puts a 1 in the checkpoint share. Radians were used for the distance in this state, as they proved to be more consistent in testing than a setpoint in millimeters.

- **STATE_2**: Changing the heading of Romi at checkpoint #4. The state reads the raw angle heading, tests it against the desired heading of 180 degrees (with respect to the starting angle), computes the bi-directional angle, and puts it in the heading share. Once the raw angle falls within the desired range, the encoder will be zeroed, a 1 will be put in the checkpoint share, and `sensor_state` is set to `STATE_3`.

- **STATE_3**: Heading following. This state is identical to the previous state, except that the transition point is an encoder distance rather than an angle. This state runs between checkpoint #4 and the turning point at the end of the grid. Once the distance to the end of the grid is read by the encoder, the state will zero the encoder, put a 1 in the checkpoint share, and `sensor_state` is set to `STATE_4`.

- **STATE_4**: Identical to `STATE_2`, but with a different final setpoint. It is a turn-in-place state with the only output being the bi-directional heading. The final heading will be 90 degrees with respect to the starting angle. The state transition process is the same.

- **STATE_5**: Nearly identical to `STATE_1`, with the state output being the centroid for line following. The transition out of this state, however, is triggered by the bump sensor. Each pass through this state, the bump sensors' Booleans are checked to see if Romi has encountered the wall. If one of the Booleans is `True`, then the standard state transition will occur as it has in the other states so far.

- **STATE_6**: Identical to `STATE_2`, but with a different final setpoint. It is a turn-in-place state with the only output being the bi-directional heading. This state is used to turn in place in preparation to drive around the wall. The final heading will be 0 degrees with respect to the starting angle. The state transition process is the same.

- **STATE_7**: Identical to `STATE_3`, but with a different final setpoint. It is a heading-following state with the only output being the bi-directional heading. The heading will be 0 degrees with respect to the starting angle. The state transition process is the same and will be triggered by reaching the desired encoder distance in millimeters.

- **STATE_8**: Identical to `STATE_2`, but with a different final setpoint. It is a turn-in-place state with the only output being the bi-directional heading. The final heading will be 90 degrees with respect to the starting angle. This state is used to turn in place towards the line. The state transition process is the same.

- **STATE_9**: Identical to `STATE_3`, but with a different final setpoint. It is a heading-following state with the only output being the bi-directional heading. The heading will be 90 degrees with respect to the starting angle. The state transition process is the same and will be triggered by reaching the desired encoder distance in millimeters when Romi re-joins the line.

- **STATE_10**: Identical to `STATE_2`, but with a different final setpoint. It is a turn-in-place state with the only output being the bi-directional heading. The final heading will be 180 degrees with respect to the starting angle. This state is used to turn in place on the line. The state transition process is the same.

- **STATE_11**: Identical to `STATE_1`, but with a target encoder distance equal to the distance traversed to avoid the wall. This is the final state transition to the stop state, but it follows the same process.

- **STATE_12**: The stop state. This state does nothing.

### Motor Task

The motor task is responsible for the actuation and control of both of Romi’s motors. It takes a `base_speed`, compensator object, two motor objects, and seven PID controllers as inputs. Each PID controller corresponds to different setpoints around the course. The motor task computes the actuation duty cycle based on sensor data and sets the motor effort. State transitions are triggered by the checkpoint share. This task is run with a priority of 2 and a period of 10 ms. 

There are **12 states** in the motor task. The transitions are determined by the sensor task and communicated via the checkpoint share. The motor task does not interface with any sensors directly and has no way of tracking progress through the state it is in.

The initialization is handled with a `motor_init` global variable that ensures initialization functions only run once. A more Pythonic implementation would be to handle this within a class constructor.


<a id="classes"></a>
#### Classes
* **Encoder:**
  Interfaces with quadrature encoder hardware to track positional changes using timer channels. It maintains internal counters, updating them through the update() method, which handles position increments, decrements, and overflow scenarios. The encoder’s position is accessed via the get_position() method, and can be reset using __zero()__.

* **Motordriver:**
  Provides a motor driver for controlling a Romi robot. It utilizes a hardware timer channel to generate PWM signals for motor speed control while using digital output pins for direction control and enabling/disabling the motor. The __set_effort()__ method accepts an effort value between -100 and 100, setting the PWM duty cycle accordingly and adjusting the motor direction. Additionally, it includes __enable()__ and __disable()__ methods to control the motor's power state. This class takes the pin names for the PWM pin, enable pin and direction pin as well as the timer number and channel for the PWM as inputs for instantiating an object of this class. 

* **IRsensor:**
  Operates an array of IR sensors for line detection and tracking. It measures analog voltage levels and calibrates sensors against defined black and white surface readings through methods __set_black()__ and __set_white()__, and stores the calibration as a .csv file. Line positions are calculated with __read_line()__, and __get_centroid()__ returns a weighted average indicating the line’s relative location. This class is designed to be implemented with an array of 7 sensors and will therefore require 7 ADC pins on the MCU. It takes the 7 pin names as inputs and creates ADC objects of the pyb.ADC class within the __init__ method and compiles them into a list so that the other methods can read the sensor values using for loops. 

* **PID:**
  Implements feedback control to regulate system performance, reducing errors between measured values and setpoints. The __update()__ method computes proportional, integral, and derivative terms over time to generate an output that effectively corrects system behavior. A controller object of this class can be implemented with any task or state. It does not require motor or sensor objects. It instead takes Kp, Ki and Kd gains as well as an arbitrary setpoint.

* **Gain_compensator:**
  Adjusts motor control outputs based on real-time battery voltage readings to maintain consistent performance. By measuring battery voltage with an ADC pin, it uses __correct_value()__ to scale the motor effort, compensating for the loss of charge that will affect motor performace. This became necssarey to avoid needing to tune the controller gains continuosly as the battery level changed. The change in battery level changes the voltage to the motors, meaning more agressive gains were needed as the battery level dropped. This class was paired with a volatage divider circuit that provided a 'pick off point' for the ADC pin to read the volatge. The was accolpished by stepping down the volatge by a factor of three using three resistaors in series to make the volatge safe for the MCU, then multiplying by three inside the method to get the actual charge on the batteries. 

* **IMU:**
  Handles communication with the Bosch BNO055 IMU sensor via the I²C protocol, offering orientation data through Euler angles. Methods such as __get_heading()__, __get_roll()__, and __get_pitch()__ parse raw IMU data into meaningful orientation angles. It includes capabilities for sensor calibration, mode configuration, and normalization of angle measurements to handle continuous rotations. The class takes a single I2C controller object as an input. The I2C controller is an object of the pyb.I2C class built into MicroPython. 

* **Bumpsensor:**
  Detects physical collisions using an array of bump-switch inputs configured with internal pull-up resistors. The __get_state()__ method returns a boolean status indicating whether any of the sensors have been triggered, enabling immediate collision response in the robot's control logic. An obejct of this class can accomodate an array of size n switches, allowing for multiple sensors in one object or a larger sensor to be used.

<a id="conclusion"></a>
### Discussion
Summary of the project outcomes and lessons learned.

<a id="video-demonstration"></a>
### Video Demonstration
A link or reference to the video demonstration of the project.

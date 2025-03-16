from pyb import Pin, Timer, ADC, I2C
from time import ticks_us, ticks_diff, sleep_ms
from os import listdir

class Encoder:
    """
    Encoder driver that updates position on demand.
    """

    def __init__(self, ch_a_pin, ch_b_pin, enc_timer):
        '''@:param ch_a_pin: Type, String. Pin for encoder channel A
        @:param ch_b_pin: Type, String. Pin for encoder channel B
        @:param enc_timer: Type, int. Timer number for encoder '''
        self.timer = Timer(enc_timer, prescaler=0, period=0xFFFF)
        self.ch_a = self.timer.channel(1, pin=ch_a_pin, mode=Timer.ENC_AB)
        self.ch_b = self.timer.channel(2, pin=ch_b_pin, mode=Timer.ENC_AB)
        self.position = 0
        self.current_pos = 0
        self.last_pos = 0
        self.delta = 0

    def update(self):
        '''Updates encoder position. Does not return position'''
        self.last_pos = self.current_pos
        self.current_pos = self.timer.counter()
        self.delta = self.current_pos - self.last_pos
        if self.delta >= 32768:
            self.delta -= 65536
        elif self.delta <= -32768:
            self.delta += 65536
        self.position += self.delta

    def get_position(self):
        ''' Returns encoder position. Must call update method first.'''
        return self.position

    def zero(self):
        '''Sets encoder position and time step to zero.'''
        self.position = 0
        self.current_pos = 0
        self.delta = 0


class Motordriver:
    """A motor driver for Romi."""

    def __init__(self, pwm_pin, dir_pin, enable_pin, timer, channel):
        '''@:param pwm_pin. Type: str. Timer channel pin to be used for PWM output
         @:param dir_pin: Type: str. Pin for motor direction pin
         @:param enable_pin: Type: str. Pin motor enable pin
         @:param timer: Type: int. Timer number for motor PWM
         @:param channel: Type: int. Timer channel for motor PWM'''
        self.dir_pin = Pin(dir_pin, Pin.OUT_PP)
        self.enable_pin = Pin(enable_pin, Pin.OUT_PP)
        self.PWM = timer.channel(channel, mode=Timer.PWM, pin=pwm_pin)

    def set_effort(self, effort):
        '''Sets motor effort in %duty cycle. Saturetion function built into this method. Returns nothing'''
        effort = max(min(effort, 100), -100)
        if effort >= 0:
            self.dir_pin.low()  # forward
            self.PWM.pulse_width_percent(effort)
        else:
            self.dir_pin.high()  # reverse
            self.PWM.pulse_width_percent(-effort)

    def enable(self):
        '''Enables Motor'''
        self.enable_pin.high()

    def disable(self):
        '''Disables motor'''
        self.enable_pin.low()


class IRsensor:
    """
    Driver for an array of 7 analog reflectance sensors.
    """

    def __init__(self, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7):
        '''@:param PIN_1: Type: str. Analog input pin to be used for reading IR sensor values. Sensor #1
        @:param PIN_2: Type: str. Analog input pin to be used for reading IR sensor values. Sensor #2
        @:param PIN_3: Type: str. Analog input pin to be used for reading IR sensor values. Sensor #3
        @:param PIN_4: Type: str. Analog input pin to be used for reading IR sensor values. Sensor #4
        @:param PIN_5: Type: str. Analog input pin to be used for reading IR sensor values. Sensor #5
        @:param PIN_6: Type: str. Analog input pin to be used for reading IR sensor values. Sensor #6
        @:param PIN_7: Type: str. Analog input pin to be used for reading IR sensor values. Sensor #7
        Pins will be configured as ADCs to read analog IR sensor values. '''
        self.pin_1 = Pin(PIN_1, mode=Pin.IN)
        self.pin_2 = Pin(PIN_2, mode=Pin.IN)
        self.pin_3 = Pin(PIN_3, mode=Pin.IN)
        self.pin_4 = Pin(PIN_4, mode=Pin.IN)
        self.pin_5 = Pin(PIN_5, mode=Pin.IN)
        self.pin_6 = Pin(PIN_6, mode=Pin.IN)
        self.pin_7 = Pin(PIN_7, mode=Pin.IN)
        self.adc_list = [
            ADC(self.pin_1), ADC(self.pin_2), ADC(self.pin_3),
            ADC(self.pin_4), ADC(self.pin_5), ADC(self.pin_6), ADC(self.pin_7)
        ]
        self.line_vals = [0] * len(self.adc_list)
        self.black_vals = [0] * len(self.adc_list)
        self.white_vals = [0] * len(self.adc_list)
        self.line_pos_list = [0] * len(self.adc_list)
        self.centroid = 0

    def set_black(self, black_data=None):
        '''This method establishes a datum for 'black' reflectance for each sensor in the array individually. Returns
        a list of integers.'''
        if black_data is None:
            input("Place sensor on BLACK surface and press Enter:")
            for i in range(len(self.adc_list)):
                self.line_vals[i] = self.adc_list[i].read()
                self.black_vals[i] = self.line_vals[i]
                self.line_vals[i] = 0
        else:
            self.black_vals = black_data
        return self.black_vals

    def set_white(self, white_data=None):
        '''This method establishes a datum for 'white' reflectance for each sensor in the array individually. Returns
        a list of integers.'''
        if white_data is None:
            input("Place sensor on WHITE surface and press Enter:")
            for i in range(len(self.adc_list)):
                self.line_vals[i] = self.adc_list[i].read()
                self.white_vals[i] = self.line_vals[i]
                self.line_vals[i] = 0
        else:
            self.white_vals = white_data
        return self.white_vals

    def read_line(self):
        '''This method reads all ADC channels in array and compiles their values into a list. The method also normalizes
        the values to black or white using linear interpolation. Must be called before get_centroid method
         Returns the list of white/black values.'''
        for i in range(len(self.adc_list)):
            self.line_vals[i] = self.adc_list[i].read()
            diff = self.black_vals[i] - self.white_vals[i]
            self.line_pos_list[i] = ((self.line_vals[i] - self.white_vals[i]) / diff) if diff != 0 else 0
        return self.line_pos_list

    def get_centroid(self):
        '''This method takes the list of line values from the read_line method and computes the centroid. Must call
         read_line method first.'''
        total = sum(self.line_pos_list)
        if total != 0:
            self.centroid = sum(val * (i + 1) for i, val in enumerate(self.line_pos_list)) / total
        else:
            self.centroid = 0
        return self.centroid

    def store_cal(self):
        '''Routine for calibrating black and white values from .csv file. Or if no file exists, calibrating and storing
        calibration values in a .csv file.'''
        filelist = listdir()

        if "IR_cal.csv" in filelist:
            print("Found calibration data, skipping calibration")
            black_data = []
            white_data = []
            with open("IR_cal.csv", "r") as file:
                for line in file:
                    try:
                        stripped = line.strip()
                        val, val2 = map(float, stripped.split(","))
                        black_data.append(val)
                        white_data.append(val2)
                    except (ValueError, IndexError):
                        print("Invalid line in calibration file:", line)

            # Apply loaded calibration values
            self.set_black(black_data)
            self.set_white(white_data)

        else:
            input("Accept Black?")
            black_data = self.set_black(None)
            input("Accept White?")
            white_data = self.set_white(None)

            with open("IR_cal.csv", "w") as file:
                for i in range(len(black_data)):
                    file.write("{},{}\n".format(black_data[i], white_data[i]))


class PID:
    ''' Simple PID controller class for constant setpoint. '''
    def __init__(self, Kp, Ki, Kd, setpoint=float(0)):
        '''@:param Kp: Type: float. Proportional gain.
        @:param Ki: Type: float. Integral gain.
        @:param Kd: Type: float. Derivative gain.
        @:param setpoint: Type: float. PID reference value.'''
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = float(setpoint)
        self.integral = 0
        self.prev_error = 0
        self.prev_time = ticks_us()

    def update(self, measured_value):
        '''This method performs PID control
        @:param measured_value: Feedback value
        @:returns actuation signal'''
        current_time = ticks_us()
        dt = ticks_diff(current_time, self.prev_time) / 1_000_000  # seconds
        error = self.setpoint - measured_value
        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        D = self.Kd * derivative
        output = P + I + D
        output = max(min(output, 100), -100)
        self.prev_error = error
        self.prev_time = current_time
        return output

class Gain_compensator:
    ''' For adjusting actuation values based on battery voltage. '''
    def __init__(self,pin):
        '''@:param pin: Board pin to be configured as an ADC'''
        self.pin = Pin(pin, mode= Pin.IN)
        self.adc = ADC(self.pin)
        self.volts = 0
        self.bat_level = 0

    def get_volt(self):
        self.volts = (self.adc.read()/4095)*3.3
        self.bat_level = (self.volts*3)
        return self.volts
    def correct_value(self,value):
        ''' This method corrects a computed duty cycle based on the current battery volatge
        @:param value: Enter a duty cycle to correct it to a 'full' level'''
        self.volts = (self.adc.read()/4095)*3.3
        self.bat_level = (self.volts*3)
        value *= 8.4/self.bat_level
        return value
class IMU: #0x28 is periphiral addr
    ''' This class defines methods and attributes for communicating with a Bosch BNO55 IMU. The communication protocol
     will be I2C using the micropython library, pyb.I2C. This class accepts a controller object created externally.  '''
    def __init__(self, controller):
        ''' Controller object should be a pyb.I2C controller object
        @:param controller: Type: I2C controller object.'''
        self.controller = controller
        self.acc_status = 0
        self.sys_status = 0
        self.gyr_status = 0
        self.mag_status = 0
        self.cal_constants = bytearray(22)
        self.euler_angles = bytearray(6)
        self.heading = 0
        self.roll = 0
        self.pitch = 0
    def cal_status(self):
        ''' The calibration status byte is located at memory addr 0x35. inside of it are 4, 2 bit numbers representing
        the status of the system and each of the sensors in calibration. bits 0&1 are the magnetometer, 2&3 are the
        accelerometer, bits 4&5 are the gyroscope and 6&7 are the system. A value of 0b00 means the sensor is not
        calibrated and a value oif 0b11 or 3 means the system is calibrated. This method parses the byte and determines
        the status of each sensor using bit wise operations. The statuses are returned as booleans. '''
        cal_stat = bytearray(1)
        self.controller.mem_read(cal_stat,0x28,0x35)
        bit_msk = 0b11
        self.sys_status = cal_stat[0] >> 6
        self.sys_status &= bit_msk
        self.gyr_status = cal_stat[0] >> 4
        self.gyr_status &= bit_msk
        self.acc_status = cal_stat[0] >> 2
        self.acc_status &= bit_msk
        self.mag_status = cal_stat[0] & bit_msk
        return self.sys_status, self.acc_status, self.gyr_status, self.mag_status

    def op_mode(self,mode):
        ''' this method takes in "mode" and tests to see which operation mode it corresponds to on the imu. Once a
        valid mode has been found the controller will write the appropriate op code to the OP_MODE register. The
        register address is 0x3D. '''
        if mode == 'NDOF':
            self.controller.mem_write(0b1100,0x28, 0x3D) #(data to write, device adresss, mem address)
        elif mode == 'NDOF_FMC_OFF':
            self.controller.mem_write(0b1011,0x28, 0X3D) #(data to write, device adresss, mem address)
        elif mode == "COMPASS":
            self.controller.mem_write(0b1001,0x28, 0X3D)#(data to write, device adresss, mem address)
        elif mode == "CONFIGMODE":
            self.controller.mem_write(0b0000,0x28, 0X3D)#(data to write, device adresss, mem address)
        else:
            print("Invalid Mode")

    def read_calibration(self): # CAL DATA STARTS AT 0X55
        ''' This method reads 22 bytes of calibration data from the IMU and stores them in a bytearray. some of the
        constants will be split over multiple bytes, so check the IMU documentation for the locations of the MSB and LSB
        the IMU must be put in CONFIGMODE to calibrate. Check status of calibration before using this method.'''
        self.controller.mem_read(self.cal_constants,0x28,0x55)
        return self.cal_constants
    def write_calibration(self, cal_data):
        '''This method writes the calibration constants back to the registers in the IMU's memory. Care should be taken
        to put constants in the appropriate order and with respect to the MSB and lSB. This method assumes that the data
        will be defined externally and accepts it as "cal_data". The assumption is that this data will be retrieved from
        a .txt file on the MCU.'''
        self.cal_constants = cal_data
        self.controller.mem_write(self.cal_constants,0x28,0x55)
    def read_euler(self):
        ''' This method retrieved the euler angles from mem addr's 0x1A->0x1F and stores them in a bytearray.  '''
        self.controller.mem_read(self.euler_angles, 0x28, 0x1A) # read 6 bytes of data starting at 1A
        return self.euler_angles
    def read_rot_vel(self):
        pass
    def get_heading(self):
        '''Combines LSB and MSB from bytearray "euler_angles" to return heading. Must call "read_euler" method first.'''
        self.heading = self.euler_angles[0] + (self.euler_angles[1]<<8) #combine LSB and MSB
        return self.heading/16
    def get_roll(self):
        '''Combines LSB and MSB from bytearray "euler_angles" to return roll. Must call "read_euler" method first.'''
        self.roll = self.euler_angles[2] + (self.euler_angles[3]<<8) #combine LSB and MSB
        return self.roll/16
    def get_pitch(self):
        '''Combines LSB and MSB from bytearray "euler_angles" to return pitch. Must call "read_euler" method first.'''
        self.pitch = self.euler_angles[4] + (self.euler_angles[5]<<8) #combine LSB and MSB
        return self.pitch/16
    def compute_angle(self,angle):
        '''This method will take in an arbirary angle in degrees and correct it for being more than 360 or less than 0.
        '''
        while True:
            if angle > 180 and angle < 360:
                angle -= 360
            else:
                return angle
            return angle
class Bumpsensor:
    def __init__(self,sensorPins):
        self.bumpPins = []
        for pin in sensorPins:
            self.bumpPins.append(pyb.Pin(pin, mode=pyb.Pin.IN, pull=pyb.Pin.PULL_UP))
    def get_state(self):
        """Returns True if any bump sensor is triggered , otherwise False."""
        for pin in self.bumpPins:
            if not pin.value():
                return True
        return False

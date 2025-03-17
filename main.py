## dont delete
from pyb import Pin, Timer, ADC, I2C
from time import ticks_us, ticks_diff, sleep_ms
import cotask
import task_share
from classes import Encoder, Motordriver, IRsensor, IMU, PID, Gain_compensator, Bumpsensor
from os import listdir
from math import pi

STATE_0 = 0
STATE_1 = 1
STATE_2 = 2
STATE_3 = 3
ref_heading = 0.0
checkpoint_heading = 0.0
filelist = listdir()

def rad_to_mm(rads):
    return rads * 35

def wrap_to_180(angle_degs):
    while angle_degs >= 180:
        angle_degs -= 360
    while angle_degs < -180:
        angle_degs += 360
    return angle_degs

# --------------------------------------------------------------------
# Motor Task
#   STATE_1: Line following using pid_line.
#   STATE_2: Rotate using pid_turn (to a setpoint equal to checkpoint_heading - 10°).
#   STATE_3: Stop the motors.
# --------------------------------------------------------------------
def motor_task(shares, left_motor, right_motor, pid_line, pid_turn, base_speed, rotate_speed):
    global motor_init, checkpoint_heading
    if motor_init is True:
        left_motor.enable()
        right_motor.enable()
        compensator = Gain_compensator('PA0')
        motor_state = STATE_1
        motor_init = False

    while True:
        if motor_state == STATE_1:
            # --- LINE FOLLOWING ---
            checkpoint = shares["checkpoint"].get()
            if checkpoint == 1:
                motor_state = STATE_2
                shares["checkpoint"].put(0)
                # Set new target: rotate 10° from the checkpoint heading.
                pid_turn.setpoint = wrap_to_180(checkpoint_heading - 10)
                print("Transitioning to ROTATION state.")
                print("Checkpoint heading =", checkpoint_heading,
                      "New target heading =", pid_turn.setpoint)
            centroid = shares["ir_centroid"].get()
            correction = pid_line.update(centroid)
            correction = compensator.correct_value(correction)
            left_motor.set_effort(base_speed - correction)
            right_motor.set_effort(base_speed + correction)
        elif motor_state == STATE_2:
            # --- ROTATION ---
            current_heading = shares["Heading"].get()  # Expected in [-180, 180)
            correction = pid_turn.update(current_heading)
            correction = compensator.correct_value(correction)
            heading_error = abs(current_heading - pid_turn.setpoint)
            print("Rotating: current =", current_heading,
                  "target =", pid_turn.setpoint,
                  "error =", heading_error)
            if heading_error < 2.0:
                motor_state = STATE_3
                continue
            left_motor.set_effort(correction * rotate_speed)
            right_motor.set_effort(-correction * rotate_speed)
        elif motor_state == STATE_3:
            # --- STOP ---
            left_motor.set_effort(0)
            right_motor.set_effort(0)
        yield 0

# --------------------------------------------------------------------
# Sensor Task
#   STATE_1: Line following – read IR sensor and IMU, compute heading offset relative to ref_heading.
#            When |offset| ≥ 180°, record the checkpoint heading and signal transition.
#   STATE_2: During rotation, update the heading.
#   STATE_3: (Optional) Additional sensor updates if needed.
# --------------------------------------------------------------------
def sensor_task(shares, ir_sensor, imu):
    global sensor_init, ref_heading, checkpoint_heading
    if sensor_init is True:
        sensor_state = STATE_1
        sensor_init = False
    while True:
        if sensor_state == STATE_1:
            ir_sensor.read_line()
            centroid = ir_sensor.get_centroid()
            shares["ir_centroid"].put(centroid)
            imu.read_euler()
            current_heading = imu.get_heading()
            computed_angle = wrap_to_180(imu.compute_angle(current_heading))
            # Compute heading offset relative to the reference.
            angle = wrap_to_180(computed_angle - ref_heading)
            shares["Heading"].put(angle)
            print("Line follow: raw =", current_heading,
                  "computed =", computed_angle,
                  "offset =", angle)
            if abs(angle) >= 180:
                checkpoint_heading = computed_angle
                shares["checkpoint"].put(1)
                sensor_state = STATE_2
                print("Sensor: Reached 180 offset. Checkpoint heading =", checkpoint_heading)
        elif sensor_state == STATE_2:
            imu.read_euler()
            current_heading = imu.get_heading()
            computed_angle = wrap_to_180(imu.compute_angle(current_heading))
            shares["Heading"].put(computed_angle)
            print("Rotation update: raw =", current_heading, "computed =", computed_angle)
        elif sensor_state == STATE_3:
            # Additional sensor state if needed.
            pass
        yield 0

# --------------------------------------------------------------------
# Main Function for State 2
#   - Initializes hardware and calibration.
#   - Creates two PID controllers:
#       pid_line for line following,
#       pid_turn for rotation (setpoint updated during transition).
#   - Starts the sensor and motor tasks.
# --------------------------------------------------------------------
def main_state2():
    global motor_init, sensor_init, ref_heading
    ir_centroid_share = task_share.Share('f', name="ir_centroid")
    heading_share = task_share.Share('f', name="Heading")
    checkpoint_share = task_share.Share('f', name="checkpoint")
    shares = {"ir_centroid": ir_centroid_share, "Heading": heading_share, "checkpoint": checkpoint_share}

    # ----- Hardware Initialization -----
    ir_sensor = IRsensor("PA1", "PC0", "PA4", "PC3", "PB0", "PC2", "PC1")
    ir_sensor.store_cal()
    left_motor = Motordriver(Pin("PA10"), "PB3", "PB5", Timer(1, freq=20000), 3)
    right_motor = Motordriver(Pin("PC9"), "PB10", "PB4", Timer(8, freq=20000), 4)
    controller = I2C(1, I2C.CONTROLLER)
    imu = IMU(controller)
    imu.op_mode('NDOF')
    ref_heading = imu.get_heading()

    if "IMU_cal.txt" in filelist:
        print("Found calibration data, writing constants")
        cal_data = bytearray(22)
        line_num = 1
        with open("IMU_cal.txt", "r") as file:
            for line in file:
                try:
                    stripped = line.strip()
                    values = stripped.split(",")
                    for n in range(len(values)):
                        val = int(values[n])
                        msb = val >> 8
                        lsb = val & 0xFF
                except ValueError:
                    print("Line " + str(line_num) + ", Bad Value")
                except IndexError:
                    print("Index error, idk how")
                else:
                    cal_data.append(msb)
                    cal_data.append(lsb)
                finally:
                    line_num += 1
        imu.write_calibration(cal_data)
    else:
        status = 0
        input("Press enter to calibrate:")
        while True:
            imu_status = imu.cal_status()
            print(f"Status:\n Sys: {imu_status[0]} \nAcc: {imu_status[1]}\nGYR: {imu_status[3]}\nMag: {imu_status[3]}")
            if imu_status[0] == 3:
                print("System Calibrated")
                status += 1
            if imu_status[1] == 3:
                print("Accelerometer Calibrated")
                status += 1
            if imu_status[2] == 3:
                print("Gyroscope Calibrated")
                status += 1
            if imu_status[3] == 3:
                print("Magnetometer Calibrated")
                status += 1
            if status == 4:
                break
            elif status == 3 and imu_status[0] == 0:
                break
            elif imu_status[1] == 3 and imu_status[2] == 3 and imu_status[3] == 3:
                break
            sleep_ms(1000)
        imu.op_mode('CONFIGMODE')
        cal_data = imu.read_calibration()
        filename = "IMU_cal.txt"
        with open(filename, "w") as file:
            file.write(",".join(map(str, cal_data))+"\n")

    imu.op_mode('NDOF')
    ref_heading = imu.get_heading()

    # Create PID controllers:
    desired_centroid = 4.0  # center for line following (for 7-sensor array)
    pid_line = PID(Kp=1, Ki=0.03, Kd=0, setpoint=desired_centroid)
    pid_turn = PID(Kp=1, Ki=0.02, Kd=0, setpoint=0)

    base_speed = float(input("Base Speed: "))
    rotate_speed = 0.2

    L_pins = ['PB2', 'PB1', 'PB15']
    L_bump = Bumpsensor(L_pins)
    R_pins = ['PC12', 'PC10', 'PC11']
    R_bump = Bumpsensor(R_pins)

    encoder = Encoder(Pin("PA6"), Pin("PA7"), 3)
    encoder.zero()

    motor_init = True
    sensor_init = True
    shares["checkpoint"].put(0)
    shares["ir_centroid"].put(4)
    print("FSM: INIT complete. Transitioning to RUN state.")
    sleep_ms(3000)

    sensor = cotask.Task(lambda: sensor_task(shares, ir_sensor, imu),
                           "Sensor_Task", priority=2, period=10)
    motor = cotask.Task(lambda: motor_task(shares, left_motor, right_motor,
                                           pid_line, pid_turn, base_speed, rotate_speed),
                        "Motor_Task", priority=2, period=10)
    cotask.task_list.append(sensor)
    cotask.task_list.append(motor)
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break
    print('\n' + str(cotask.task_list))
    print(task_share.show_all())
    print("State 2 complete.")

if __name__ == '__main__':
    main_state2()

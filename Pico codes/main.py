from imu import MPU6050
from utime import sleep
from machine import Pin, I2C
from pid import Pid
from pid_abstract import PidAbstract
from dc_motor import DcMotor
from pins import *
from mpu6050 import init_mpu6050, get_mpu6050_data
import utime

KP = 1
KI = 0

#Shows Pi is on by turning on LED when plugged in
LED = machine.Pin("LED", machine.Pin.OUT)

pid_abstract = None
prev_angle = 0

right_motor = DcMotor(FRONT_RIGHT_MOTOR_SPEED_CTRL_PIN, FRONT_RIGHT_MOTOR_IN1_PIN, FRONT_RIGHT_MOTOR_IN2_PIN)
left_motor = DcMotor(FRONT_LEFT_MOTOR_SPEED_CTRL_PIN, FRONT_LEFT_MOTOR_IN1_PIN, FRONT_LEFT_MOTOR_IN2_PIN)
LED.on()

sleep(1)

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)


init_mpu6050(i2c)
    

def start():
    global pid_abstract, right_motor, left_motor
    pid_abstract = PidAbstract(pid, sensor, actuator)
    right_motor.rotate(80)
    left_motor.rotate(70)

def sensor():
    global prev_angle
    data = get_mpu6050_data(i2c)
    prev_angle += (data['gyro']['z'] - 0.453) * 0.005
    return prev_angle

def actuator(pid_val):
    #  print(pid_val)
    left_motor.rotate(70 + pid_val)
    
def stop():
    right_motor.rotate(0)
    left_motor.rotate(0)    
    
def right():
    global pid_abstract, right_motor, left_motor, prev_angle
    pid_abstract.pause()
    # print("turning right")
    stop()
    sleep(1)
    right_motor.rotate(-50)
    left_motor.rotate(-50)
    sleep(0.2)
    right_motor.rotate(-50)
    right_motor.rotate(50)
    # print("turned right")
    cur_angle = 0
    while prev_angle <= 90:
        cur_angle += (data['gyro']['z'] - 0.453) * 0.005
        utime.sleep_ms(5)
    stop()
    prev_angle = 0
    pid_abstract.resume()

def left():
    global pid_abstract, right_motor, left_motor, prev_angle
    pid_abstract.pause()
    # print("turning right")
    stop()
    sleep(1)
    right_motor.rotate(-50)
    left_motor.rotate(-50)
    sleep(0.2)
    right_motor.rotate(50)
    right_motor.rotate(-50)
    # print("turned right")
    cur_angle = 0
    while prev_angle >= -90:
        cur_angle += (data['gyro']['z'] - 0.453) * 0.005
        utime.sleep_ms(5)
    stop()
    prev_angle = 0
    pid_abstract.resume()

pid = Pid(
    kp = KP,
    ki = KI,
    kd = 0,
    setpoint = 0,
    output_high_limit = 30,
    output_low_limit = -70,
)

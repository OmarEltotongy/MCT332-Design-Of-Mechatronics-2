import rospy
from rospy import Subscriber, Publisher
from std_msgs.msg import Bool, String
from serial import Serial
import time
import random
from datetime import datetime

ARDUINO_USB = '/dev/ttyACM0'
PICO_USB = '/dev/ttyACM1'

FEED_SERVE_MSG = 's'
FEED_DONE_MSG = 'd'
OBSTACLE_MSG = 'o'
NON_OBSTACLE_MSG = 'n'

feed_already_read = False
robot_already_moving = False
last_message_rec = ''
directions = ["right", "left"]
cur_direction_ind = 0

wanted_time = ""

def send_command_to_pico(function_name, serial_obj):
    text = "%s\r\f" % (function_name + "()")
    serial_obj.write(text.encode())

def vision_callback(data):
    global ser, feed_already_read, pico_serial
    pico_serial.reset_input_buffer()
    # ser.reset_input_buffer()
    if data.data is True and feed_already_read is False:
        feed_already_read = True
        print("QR READ")
        pub.publish(String('FP'))
        # stop_robot()
        ser.write(FEED_SERVE_MSG.encode())
    else:
        pass


def req_clbk(data):
    global feed_already_read, wanted_time
    print(data.data)
    if data.data == 'start':
        # print(data.data)
        feed_already_read = False
    else:
        wanted_time = data.data
        print(wanted_time)

def move_robot():
    global pico_serial, feed_already_read, robot_already_moving 
    if feed_already_read is False and robot_already_moving is False:
        print("start() is sent")
        send_command_to_pico("start", pico_serial)
        print(pico_serial.read(13))
        robot_already_moving = True
    else:
        pass

def stop_robot():
    global pico_serial, robot_already_moving
    print("stop() is sent")
    send_command_to_pico("stop", pico_serial)
    print(pico_serial.read(12))
    robot_already_moving = False
    
def turn_robot():
    global directions, cur_direction_ind, pico_serial, ser, robot_already_moving, feed_already_read
    if not feed_already_read:
        robot_already_moving = True
        print(f"{directions[cur_direction_ind]}() is sent")
        send_command_to_pico(directions[cur_direction_ind], pico_serial)
    
        if directions[cur_direction_ind] == 'left':
            print(pico_serial.read(12))
        elif directions[cur_direction_ind] == 'right':
            print(pico_serial.read(13))
    
        pico_serial.reset_input_buffer()
        ser.reset_input_buffer()
        robot_already_moving = False
    else:
        pass

ser = Serial(ARDUINO_USB, 115200)
pico_serial = Serial(PICO_USB, 115200)

# send_command_to_pico("start", pico_serial)

rospy.init_node("robot_mover_node", anonymous=True)
sub = Subscriber('/vision_topic', Bool, vision_callback)
pub = Publisher('/status_topic', String, queue_size=10)
req_sub = Subscriber('/req_topic', String, req_clbk)


while not rospy.is_shutdown():
    serial_message = ser.read().decode()
    # print(serial_message)
    if serial_message == OBSTACLE_MSG:
        turn_robot()
        if feed_already_read is False:
            pub.publish(String('SP'))
    elif serial_message == FEED_DONE_MSG:
        
        stop_robot()
        # feed_already_read = False
        cur_direction_ind = random.randint(0, 1)
    elif serial_message == NON_OBSTACLE_MSG:
        move_robot()
        if feed_already_read is False:
            pub.publish(String('SP'))
        cur_direction_ind = random.randint(0, 1)
        
    now = datetime.now()
    cur_time = now.strftime("%H:%M")
    if cur_time == wanted_time:
        # print("starting")
        feed_already_read = False

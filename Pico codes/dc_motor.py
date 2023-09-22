from machine import Pin, PWM

U16_MAX = 65535

class DcMotor:
    def __init__(self, speed_ctrl_pin, in1_pin, in2_pin):
        
        self.speed_ctrl_pin = PWM(Pin(speed_ctrl_pin))
        self.in1_pin = Pin(in1_pin, Pin.OUT)
        self.in2_pin = Pin(in2_pin, Pin.OUT)
        
        self.speed_ctrl_pin.freq(500)
        
    def rotate(self, speed_percentage):
        
        if speed_percentage < 0:
            self.in1_pin.value(0)
            self.in2_pin.value(1)
            speed_percentage = -1 * speed_percentage
        else:
            self.in1_pin.value(1)
            self.in2_pin.value(0)
            
        speed_u16 = (speed_percentage / 100) * U16_MAX
        
        self.speed_ctrl_pin.duty_u16(int(speed_u16))

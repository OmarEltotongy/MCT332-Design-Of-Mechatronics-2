from machine import Pin
import utime

PI = 3.14159

class Encoder:
    def __init__(self, clk_pin, dt_pin, pulses_per_rev):
        
        self.clk_pin = Pin(clk_pin, Pin.IN, Pin.PULL_UP)
        self.dt_pin = Pin(dt_pin, Pin.IN, Pin.PULL_UP)
        
        self.pulses_per_rev = pulses_per_rev
        
        self.clk_pin.irq(trigger=Pin.IRQ_FALLING, handler=self.__pulse_counter)
        
        self.dir = 1
        self.pulses = 0
        self.prev_rpm = 0
        
        self.pos_pulses = 0
        
        self.prev_time = utime.ticks_us()
        
        self.prev_pulse_counter_time = utime.ticks_us()
        
    def __pulse_counter(self, pin):
        self.dir = 1
        
        if self.dt_pin.value():
            self.dir = -1
        
        self.pulses += self.dir
        self.pos_pulses += self.dir
        
        # print(self.pos_pulses)
        
        
    def get_avg_rpm(self, alpha=1):
        def low_pass_filer(raw_reading, alpha):
            self.prev_rpm = alpha * raw_reading + (1 - alpha) * self.prev_rpm
            return self.prev_rpm

        revs = self.pulses / self.pulses_per_rev
        dt = utime.ticks_diff(utime.ticks_us(), self.prev_time) * 0.000001
        
        rpm = (revs / dt) * 60
        filtered_rpm = low_pass_filer(rpm, alpha)
        
        self.prev_time = utime.ticks_us()
        self.pulses = 0
        
        return filtered_rpm
    
    def reset_absolute_position(self):
        self.pos_pulses = 0
    
    def get_absolute_position_in_rad(self):
        rads = (self.pos_pulses / self.pulses_per_rev) * 2 * PI
        
        return rads

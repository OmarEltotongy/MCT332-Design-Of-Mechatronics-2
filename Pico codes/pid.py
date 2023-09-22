import utime

class Pid:
    def __init__(self, kp, ki, kd, setpoint, output_high_limit, output_low_limit, initial_reading=0):
        if output_high_limit <= output_low_limit:
            raise Exception("In class 'Pid', 'output_high_limit' must be greater than 'output_low_limit'")
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        self.ki_copy = ki
        
        self.output_high_limit = output_high_limit
        self.output_low_limit = output_low_limit
        
        self.prev_error = self.compute_error(initial_reading)
        self.cur_error = 0
        
        self.prev_time = utime.ticks_us()
        self.cur_time = 0
        
        self.prev_i = 0
        self.prev_d = 0
        
        self.pid_enabled = True
        
    def enable(self):
        self.pid_enabled = True
        
    def disable(self):
        self.pid_enabled = False
        
    def re_adjust_params(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
    def re_adjust_setpoint(self, setpoint):
        self.setpoint = setpoint
        
    def compute_error(self, cur_reading):
        return self.setpoint - cur_reading
    
    def __compute_proportional_term(self, cur_reading):
        return self.kp * self.cur_error
    
    def __compute_integral_term(self, cur_reading):
        dt = utime.ticks_diff(self.cur_time, self.prev_time) * 0.000001
        e = (self.cur_error + self.prev_error) / 2
        
        self.prev_i = self.prev_i + self.ki * e * dt
        
        return self.prev_i
    
    def __compute_derivative_term(self, cur_reading, alpha):
        def low_pass_filter(raw_d, alpha):
            self.prev_d = alpha * raw_d + (1 - alpha) * self.prev_d
            return self.prev_d
            
        dt = utime.ticks_diff(self.cur_time, self.prev_time) * 0.000001
        de = self.cur_error - self.prev_error
        
        if not dt:
            return 0
        
        d = self.kd * (de / dt)
        return low_pass_filter(d, alpha)
    
    def get_pid(self, cur_reading, alpha=1):
        
        if not self.pid_enabled:
            return 0
        
        self.cur_time = utime.ticks_us()
        self.cur_error = self.compute_error(cur_reading)
        
        p = self.__compute_proportional_term(cur_reading)
        i = self.__compute_integral_term(cur_reading)
        d = self.__compute_derivative_term(cur_reading, alpha)
        
        self.prev_time = self.cur_time
        self.prev_error = self.cur_error
        
        pid = p + i + d
        
        if pid > self.output_high_limit:
            pid = self.output_high_limit
            self.ki = 0
            
        elif pid < self.output_low_limit:
            pid = self.output_low_limit
            self.ki = 0
            
        else:
            self.ki = self.ki_copy

        return pid

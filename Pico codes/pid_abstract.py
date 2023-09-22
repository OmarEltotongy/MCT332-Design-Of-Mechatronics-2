from pid import Pid
from machine import Timer

class PidAbstract:
    def __init__(self, pid_instance: Pid, sensor_clbk, actuator_clbk, time_slice=5):
        self.pid_instance = pid_instance
        self.sensor_clbk = sensor_clbk
        self.actuator_clbk = actuator_clbk
        
        self.periodic_timer = Timer(-1)
        self.periodic_timer.init(period=time_slice, mode=Timer.PERIODIC, callback=self.__callback)
        
    def __callback(self, _):
        sensor_reading = self.sensor_clbk()
        
        pid = self.pid_instance.get_pid(sensor_reading)
        
        self.actuator_clbk(pid)

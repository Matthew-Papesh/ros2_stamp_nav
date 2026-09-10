#!/usr/bin/env python3
from typing import Callable

class PID:
    """
    Represents a Proportional Derivative Integral (PID) controller for optimizing dynamic control systems. 
    """
    def __init__(self, kp: float, ki: float, kd: float, process_variable: Callable[[], float], set_point: Callable[[], float], clegg_integration: bool):
        """
        Creates a PID controller instance given specified coefficients and callable suppliers for evaluating error at any given time. 
        param: kp [float] The specified proportional coefficient
        param: ki [float] The specified integral coefficient
        param: kd [float] The specified derivative coefficient
        param: process_variable [Callable[[], float]] The specified process variable supplier to compare to desired system value
        param: set_point [Callable[[], float]] The specified desired value of the system
        """
        self.kp, self.ki, self.kd = kp, ki, kd
        self.error, self.error_sum, self.prev_error = 0.0, 0.0, 0.0
        self.process_variable_def = process_variable
        self.set_point_def = set_point
        self.clegg_integration = clegg_integration

    def output(self) -> float:
        """
        Computes the output of the PID controller based on system error. 
        returns: controller output
        """
        self.prev_error = self.error
        self.error = self.process_variable_def() - self.set_point_def()
        self.error_sum += self.error
        
        output = self.kp*self.error + self.ki*self.error_sum + self.kd*(self.error - self.prev_error)
        if self.clegg_integration:
            self.error_sum = self.error_sum if ((self.error >= 0 and self.prev_error >= 0) or (self.error < 0 and self.prev_error < 0)) else 0.0
        return output
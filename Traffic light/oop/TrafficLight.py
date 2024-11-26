import numpy as np
class TrafficLight:
    def __init__(self, distance, green_start, green_end, cycle_time):
        self.distance = distance
        self.green_start = green_start
        self.green_end = green_end
        self.cycle_time = cycle_time
        self.green_duration = green_end - green_start
        self.offset = green_start % cycle_time

    def is_green(self, time):
        time_in_cycle = time % self.cycle_time
        if self.green_start <= self.green_end:  # No wrapping
            return self.green_start <= time_in_cycle < self.green_end
        else:  # Wrapping around
            return time_in_cycle >= self.green_start or time_in_cycle < self.green_end

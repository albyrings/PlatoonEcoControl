import numpy as np
class Vehicle:
    def __init__(self, vehicle_id, position=0, speed=5):
        self.vehicle_id = vehicle_id
        self.position = position
        self.speed = speed
        self.history = []  # To store position over time for visualization

    def update_position(self, dt):
        self.position += self.speed * dt
        self.history.append(self.position)


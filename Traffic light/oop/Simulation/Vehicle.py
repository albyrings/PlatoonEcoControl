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

class Platoon:
    def __init__(self, num_vehicles, initial_positions=None, speeds=None):
        self.vehicles = []
        for i in range(num_vehicles):
            position = initial_positions[i] if initial_positions else 0
            speed = speeds[i] if speeds else 5
            self.vehicles.append(Vehicle(vehicle_id=i, position=position, speed=speed))

    def update_positions(self, dt):
        for vehicle in self.vehicles:
            vehicle.update_position(dt)

    def adjust_speeds(self, traffic_lights, time):
        for vehicle in self.vehicles:
            for light in traffic_lights:
                if vehicle.position < light.distance and not light.is_green(time):
                    vehicle.speed = 0  # Stop at red light
                else:
                    vehicle.speed = 5  # Resume speed

from matplotlib import pyplot as plt
import numpy as np
class Simulation:
    def __init__(self, platoon, traffic_lights, total_time, dt):
        self.platoon = platoon
        self.traffic_lights = traffic_lights
        self.total_time = total_time
        self.dt = dt
        self.time = 0
        self.results = []

    def run(self):
        while self.time <= self.total_time:
            # Update vehicles
            self.platoon.adjust_speeds(self.traffic_lights, self.time)
            self.platoon.update_positions(self.dt)
            
            # Record the state
            self.results.append({
                "time": self.time,
                "positions": [v.position for v in self.platoon.vehicles],
                "speeds": [v.speed for v in self.platoon.vehicles]
            })
            
            # Increment time
            self.time += self.dt

    def visualize(self):
        plt.figure(figsize=(15, 5))
        for vehicle in self.platoon.vehicles:
            plt.plot(
                np.arange(0, self.total_time + self.dt, self.dt),
                [state["positions"][vehicle.vehicle_id] for state in self.results],
                label=f"Vehicle {vehicle.vehicle_id}"
            )
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.title("Vehicle Positions Over Time")
        plt.legend()
        plt.grid()
        plt.show()

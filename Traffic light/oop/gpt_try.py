# Import necessary modules
from Simulation import Simulation
from Vehicle import Platoon, Vehicle
from TrafficLight import TrafficLight
from constants import *
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import link_energy
import networkx as nx

# Define traffic lights
intersections = [
    {"distance": 300, "green_start": 13, "green_end": 23},
    {"distance": 600, "green_start": 3, "green_end": 13},
    {"distance": 900, "green_start": 28, "green_end": 38},
    {"distance": 1200, "green_start": 15, "green_end": 25},
    {"distance": 1550, "green_start": 5, "green_end": 15},
]
n = len(intersections)

# Constants (Ensure these are defined in constants.py or define them here)
# For demonstration, defining them here
T = 60  # Cycle time in seconds
tf = 300  # Final time in seconds
v_max = 15  # Maximum velocity in m/s
v_min = 5  # Minimum velocity in m/s
h0, h1, h2, h3 = 1, 1, 1, 1  # Energy-related constants
b1, b2 = 1, 1  # More energy-related constants
final_distance = 2000  # Total distance in meters
vehicle_length = 5  # Vehicle length in meters
security_distance = 2  # Security distance in meters

# Update the intersections to include the cycle offset C_i and green duration T_gr_i
for intersection in intersections:
    intersection['T_gr_i'] = intersection['green_end'] - intersection['green_start']
    intersection['C_i'] = intersection['green_start'] % T

# Distances of intersections (including start and end points)
d = [0] + [intersection['distance'] for intersection in intersections] + [final_distance]
n_intersections = len(intersections)

# Initialize t_min and t_max arrays (including start and end points)
t_min = np.zeros(n_intersections + 2)
t_max = np.zeros(n_intersections + 2)

# Set initial times at the starting point
t_min[0] = 0
t_max[0] = 0

# Define the updated s_i function to check if the light is green at time t
def s_i(t, intersection):
    T_cycle = T
    green_start = intersection['C_i']  # Start of green phase in the cycle
    green_end = (intersection['C_i'] + intersection['T_gr_i']) % T_cycle  # End of green phase in the cycle

    # Calculate time within the current cycle
    time_in_cycle = t % T_cycle

    # Check if time falls within the green phase, accounting for wrapping
    if green_start <= green_end:  # Green phase does not wrap around
        if green_start <= time_in_cycle < green_end:
            return 1  # Green
    else:  # Green phase wraps around the end of the cycle
        if time_in_cycle >= green_start or time_in_cycle < green_end:
            return 1  # Green

    return 0  # Red

# Pruning function to compute feasible time intervals
def pruning(t_min, t_max):
    # Forward pass
    for i in range(1, n_intersections + 1):
        # Compute t_i,min
        t_min[i] = t_min[i - 1] + (d[i] - d[i - 1]) / v_max

        # Compute t_i,max
        t_max[i] = t_max[i - 1] + (d[i] - d[i - 1]) / v_min

        # Adjust t_i,max based on final time
        t_i_max_candidate = tf - (d[-1] - d[i]) / v_max
        t_max[i] = min(t_max[i], t_i_max_candidate)

        # Adjust t_i,min if signal is not green
        if s_i(t_min[i], intersections[i - 1]) != 1:
            # Find the next green phase start
            cycles_passed = np.floor(t_min[i] / T)
            t_min[i] = cycles_passed * T + intersections[i - 1]['C_i'] + intersections[i - 1]['T_gr_i']

        # Adjust t_i,max if signal is not green
        if s_i(t_max[i], intersections[i - 1]) != 1:
            # Find the previous green phase end
            cycles_passed = np.floor(t_max[i] / T)
            t_max[i] = cycles_passed * T + intersections[i - 1]['C_i']

        # Ensure t_min[i] ≤ t_max[i]
        if t_min[i] > t_max[i]:
            print(f"No feasible times for intersection {i}")
            break  # Or handle appropriately

    # Backward pass
    for i in range(n_intersections, 0, -1):
        # Check conditions
        if t_max[i] <= t_max[i - 1] or (d[i] - d[i - 1]) / (t_max[i] - t_max[i - 1]) > v_max:
            # Adjust t_{i-1,max}
            t_max[i - 1] = t_max[i] - (d[i] - d[i - 1]) / v_max

            # Adjust t_{i-1,max} if signal is not green
            if i - 2 >= 0 and s_i(t_max[i - 1], intersections[i - 2]) != 1:
                cycles_passed = np.floor(t_max[i - 1] / T)
                t_max[i - 1] = cycles_passed * T + intersections[i - 2]['C_i']

            # Ensure t_min[i - 1] ≤ t_max[i - 1]
            if t_min[i - 1] > t_max[i - 1]:
                print(f"No feasible times after adjustment for intersection {i - 1}")
                break  # Or handle appropriately

    return t_min, t_max

# Initial pruning to compute t_min and t_max
t_min, t_max = pruning(t_min, t_max)

# Set final times
t_min[-1] = tf
t_max[-1] = tf

# List of intersection distances including start and end
list_intersection = [el['distance'] for el in intersections]
list_intersection.append(final_distance)
list_intersection.insert(0, 0)

# Compute Dijkstra's algorithm to find optimal crossing times
def compute_dijkstra(t_min, t_max, distances, t_jump=1):
    """
    Compute the optimal crossing times using Dijkstra's algorithm with energy costs.
    """
    G = nx.DiGraph()
    num_intersections = len(distances) - 1

    # Discretize feasible time intervals for Dijkstra
    time_samples = 10  # Number of discrete time samples per interval
    time_nodes = []
    for i in range(len(t_min)):
        time_range = np.linspace(t_min[i], t_max[i], time_samples)
        time_nodes.append(time_range)

    # Add nodes and edges with energy cost as weights
    for i in range(num_intersections):
        for t1 in time_nodes[i]:
            for t2 in time_nodes[i + 1]:
                # Calculate speed and check feasibility
                dt = t2 - t1
                if dt <= 0:
                    continue  # Invalid time difference
                speed = (distances[i + 1] - distances[i]) / dt
                if v_min <= speed <= v_max and t_min[i + 1] <= t2 <= t_max[i + 1]:
                    # Calculate energy cost using link_energy module
                    e_link = link_energy.compute_link_energy(distances[i + 1], distances[i], t2, t1)
                    if i > 0:  # For edges beyond the first
                        prev_dt = t1 - time_nodes[i - 1][0]
                        if prev_dt <= 0:
                            e_jump = 0  # No jump energy if previous dt is invalid
                        else:
                            prev_speed = (distances[i] - distances[i - 1]) / prev_dt
                            e_jump = link_energy.compute_jump_energy(prev_speed, speed, t_jump)
                    else:
                        e_jump = 0  # No jump energy for the first edge

                    cost = e_link + e_jump
                    G.add_edge((i, t1), (i + 1, t2), weight=cost)

    # Add final destination node
    for t in time_nodes[-1]:
        G.add_edge((num_intersections, t), "final", weight=0)

    # Run Dijkstra's algorithm
    source_nodes = [(0, t) for t in time_nodes[0]]
    min_path = None
    min_cost = float('inf')
    for source_node in source_nodes:
        try:
            path = nx.shortest_path(G, source=source_node, target="final", weight="weight")
            # Calculate total cost
            total_cost = 0
            for j in range(len(path) - 1):
                u = path[j]
                v = path[j + 1]
                total_cost += G[u][v]['weight']
            if total_cost < min_cost:
                min_cost = total_cost
                min_path = path
        except nx.NetworkXNoPath:
            continue  # Try next source node

    if min_path is None:
        print("No path found in Dijkstra's algorithm.")
        return []

    # Extract optimal times
    optimal_times = [node[1] for node in min_path[:-1]]  # Exclude the final node
    return optimal_times

optimal_times = compute_dijkstra(t_min, t_max, list_intersection)

if not optimal_times:
    print("Dijkstra's algorithm did not find a feasible path.")
    exit()

# Objective function: Minimize energy cost
def objective(x):
    n = len(d) - 1
    energy_cost = 0
    for i in range(1, n + 1):
        delta_t = x[i] - x[i - 1]
        if delta_t <= 0:
            return float('inf')  # Penalize invalid solutions
        v_bar = (d[i] - d[i - 1]) / delta_t  # Average velocity
        u_bar = (1 / h1) * (h2 * v_bar**2 + h3 * v_bar + h0)  # Driving input
        energy_cost += delta_t * (b1 * u_bar * v_bar + b2 * u_bar**2)
    return energy_cost

# Define constraints for optimization
constraints = []
for i in range(1, len(t_min)):
    constraints.append({'type': 'ineq', 'fun': lambda x, i=i: x[i] - t_min[i]})  # t_i >= t_min[i]
    constraints.append({'type': 'ineq', 'fun': lambda x, i=i: t_max[i] - x[i]})  # t_i <= t_max[i]
    constraints.append({'type': 'ineq', 'fun': lambda x, i=i: x[i] - x[i - 1] - 1e-3})  # t_i > t_{i-1}

# Initial guess: Use Dijkstra times directly
x0 = optimal_times.copy()

# Optimization bounds
bounds = [(0, 0)]  # t0 is fixed at 0
for i in range(1, len(t_min)):
    # Define bounds around the optimal_times with a window of +/-5 seconds
    lower_bound = max(t_min[i], optimal_times[i] - 5)
    upper_bound = min(t_max[i], optimal_times[i] + 5)
    bounds.append((lower_bound, upper_bound))

print("Optimal Times (Before Optimization):", optimal_times)
print("Bounds:", bounds)
print("t_min:", t_min)

# Run the optimization
result = minimize(objective, x0, bounds=bounds, constraints=constraints, method='SLSQP')

# Display optimization results
optimization_result = result.x
if result.success:
    print("Optimization successful!")
    print("Optimal crossing times:")
    for i, t in enumerate(result.x):
        print(f"t_{i}: {t:.2f} seconds")
    print(f"Total energy cost: {result.fun:.2f}")
else:
    print("Optimization failed:", result.message)

# Function to calculate maximum number of vehicles that can pass based on vehicle length, security distance, and velocity
def calculate_max_vehicles(intersections, vehicle_length, security_distance, velocity):
    """
    Calculate the maximum number of vehicles that can pass during a green phase for each intersection,
    and return the minimum across all intersections to ensure synchronization.
    """
    max_vehicles_per_intersection = []
    time_per_vehicle = (vehicle_length + security_distance) / velocity  # Time required per vehicle

    for intersection in intersections:
        green_duration = intersection['T_gr_i']
        max_vehicles = int(green_duration // time_per_vehicle)
        max_vehicles_per_intersection.append(max_vehicles)

    return min(max_vehicles_per_intersection)

# Function to visualize the crossing times without shifting
def no_shift():
    inner_bounds = bounds.copy()
    inner_bounds.pop(0)
    inner_bounds.pop(-1)
   
    time_for_vehicle = (vehicle_length + security_distance) / v_max  # Adjusted based on velocity

    plt.figure(figsize=(12, 6))
    plt.plot(t_min, list_intersection, label='t_min', color='blue', linestyle='--')
    plt.plot(t_max, list_intersection, label='t_max', color='red', linestyle='--')
    plt.hlines(list_intersection, t_min, t_max, color='gray', alpha=0.5)

    # Plot optimal times
    plt.scatter(optimization_result, list_intersection, color='green', label='Optimal Times')

    # Plot the vehicle path 
    for i in range(1, len(t_min)):
        plt.plot([optimization_result[i - 1], optimization_result[i]], [list_intersection[i - 1], list_intersection[i]], linewidth=2, color='orange')
        # Plot vehicle lanes
        for j in range(calculate_max_vehicles(intersections, vehicle_length, security_distance, v_max)):
            plt.plot([optimization_result[i - 1] + time_for_vehicle * j, optimization_result[i] + time_for_vehicle * j], 
                     [list_intersection[i - 1], list_intersection[i]], linewidth=1, color='purple')

    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.grid(True)
    plt.title('Feasible Time Intervals at Intersections (No Shift)')
    plt.legend()
    plt.show()

# Modified shift function to handle multiple vehicle groups
def shift(n_of_vehicles=3, velocity=v_max):
    """
    Shift the crossing times to accommodate multiple groups of vehicles when the queue exceeds green phase capacity.
    """
    inner_bounds = bounds.copy()
    inner_bounds.pop(0)
    inner_bounds.pop(-1)
 
    # Calculate the time required per vehicle
    time_per_vehicle = (vehicle_length + security_distance) / velocity  # Time per vehicle

    # Calculate the maximum number of vehicles that can pass in one green phase
    max_vehicles_per_wave = calculate_max_vehicles(intersections, vehicle_length, security_distance, velocity)

    print(f"Maximum vehicles per green wave: {max_vehicles_per_wave}")

    # Initialize list to hold groups of vehicles
    vehicle_groups = []
    remaining_vehicles = n_of_vehicles

    while remaining_vehicles > 0:
        if remaining_vehicles > max_vehicles_per_wave:
            group_size = max_vehicles_per_wave
        else:
            group_size = remaining_vehicles
        vehicle_groups.append(group_size)
        remaining_vehicles -= group_size

    print(f"Vehicle groups: {vehicle_groups}")

    # Initialize list to store all shifted optimal times
    all_shifted_times = []

    # Initialize shift offset
    shift_offset = 0

    for idx, group in enumerate(vehicle_groups):
        print(f"Processing group {idx + 1} with {group} vehicles.")

        # Copy the original optimization result
        shifted_times = optimization_result.copy()

        # Shift the crossing times by the cumulative shift offset
        shifted_times += shift_offset

        # Saturate the green phase by scheduling vehicles within the green phase duration
        for i in range(1, len(shifted_times)):
            # Schedule the next vehicle based on the time_per_vehicle
            shifted_times[i] = shifted_times[i - 1] + time_per_vehicle

            # Ensure crossing time does not exceed t_max
            if shifted_times[i] > t_max[i]:
                shifted_times[i] = t_max[i]

        # Append the shifted times for this group
        all_shifted_times.append(shifted_times)

        # Update shift_offset for the next group
        # Find the next available green phase start time after the current group's last crossing time
        last_crossing_time = shifted_times[-2]  # Last intersection crossing time
        next_green_start = np.ceil(last_crossing_time / T) * T + intersections[0]['C_i']
        shift_offset = next_green_start - shifted_times[0]

    # Plotting all shifted groups
    plt.figure(figsize=(12, 6))
    plt.plot(t_min, list_intersection, label='t_min', color='blue', linestyle='--')
    plt.plot(t_max, list_intersection, label='t_max', color='red', linestyle='--')
    plt.hlines(list_intersection, t_min, t_max, color='gray', alpha=0.5)

    colors = ['green', 'purple', 'orange', 'cyan', 'magenta', 'yellow', 'black']  # Colors for different groups

    for idx, shifted_times in enumerate(all_shifted_times):
        color = colors[idx % len(colors)]
        plt.scatter(shifted_times, list_intersection, color=color, label=f'Group {idx + 1} Times')

        # Plot vehicle paths for the group
        for i in range(1, len(shifted_times)):
            plt.plot([shifted_times[i - 1], shifted_times[i]], [list_intersection[i - 1], list_intersection[i]], linewidth=2, color=color)
            # Plot individual vehicle lanes within the group
            for j in range(vehicle_groups[idx]):
                vehicle_shift = j * time_per_vehicle
                plt.plot([shifted_times[i - 1] + vehicle_shift, shifted_times[i] + vehicle_shift], 
                         [list_intersection[i - 1], list_intersection[i]], linewidth=1, color=color, alpha=0.6)

    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.grid(True)
    plt.title('Shifted Crossing Times for Multiple Vehicle Groups at Intersections')
    plt.legend()
    plt.show()

# Example usage: Shift 10 vehicles with default velocity
shift(n_of_vehicles=20, velocity=v_max)

# Optionally, call no_shift() to visualize the original scenario
# no_shift()

import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import link_energy
import networkx as nx

# Constants
v_min = 5  # Minimum speed (m/s)
v_max = 14  # Maximum speed (m/s)
final_time = 200
final_distance = 1800
intersections = [
    {"distance": 300, "green_start": 13, "green_end": 23},
    {"distance": 600, "green_start": 3, "green_end": 13},
    {"distance": 900, "green_start": 20, "green_end": 30},
    {"distance": 1200, "green_start": 15, "green_end": 25},
    {"distance": 1550, "green_start": 5, "green_end": 15},
] 
n = len(intersections)
T = 30  # Traffic light cycle time (seconds)
tf = final_time

# Constants for the energy function
b1 = 0.1  # Coefficient for energy
b2 = 0.05  # Coefficient for energy
h1 = 1.0  # Driving input coefficient
h2 = 0.2  # Driving input coefficient
h3 = 0.1  # Driving input coefficient
h0 = 0.05  # Offset in driving input

# Update the intersections to include the cycle offset C_i and green duration T_gr_i
for intersection in intersections:
    # Assuming the traffic lights repeat their green phases every T seconds
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

# Define the s_i(t) function
def s_i(t, intersection):
    T_cycle = T
    time_in_cycle = (t - intersection['C_i']) % T_cycle
    if 0 <= time_in_cycle < intersection['T_gr_i']:
        return 1  # Green
    else:
        return 0  # Red

# Forward pass (Lines 1-11)
for i in range(1, n_intersections + 1):
    # Line 2: Compute t_i,min
    t_min[i] = t_min[i - 1] + (d[i] - d[i - 1]) / v_max

    # Line 3: Compute t_i,max
    t_max[i] = t_max[i - 1] + (d[i] - d[i - 1]) / v_min

    # Line 4: Adjust t_i,max
    t_i_max_candidate = tf - (d[-1] - d[i]) / v_max
    t_max[i] = min(t_max[i], t_i_max_candidate)

    # Line 5-7: Adjust t_i,min if signal is not green
    if s_i(t_min[i], intersections[i - 1]) != 1:
        t_min[i] = np.floor(t_min[i] / T) * T + intersections[i - 1]['C_i'] + intersections[i - 1]['T_gr_i']

    # Line 8-10: Adjust t_i,max if signal is not green
    if s_i(t_max[i], intersections[i - 1]) != 1:
        t_max[i] = np.floor(t_max[i] / T) * T + intersections[i - 1]['C_i']

    # Ensure t_min[i] ≤ t_max[i]
    if t_min[i] > t_max[i]:
        print(f"No feasible times for intersection {i}")
        break  # Or handle appropriately

# Backward pass (Lines 12-19)
for i in range(n_intersections, 0, -1):
    # Line 13: Check conditions
    if t_max[i] <= t_max[i - 1] or (d[i] - d[i - 1]) / (t_max[i] - t_max[i - 1]) > v_max:
        # Line 14: Adjust t_{i-1,max}
        t_max[i - 1] = t_max[i] - (d[i] - d[i - 1]) / v_max

        # Line 15-17: Adjust t_{i-1,max} if signal is not green
        if i - 2 >= 0 and s_i(t_max[i - 1], intersections[i - 2]) != 1:
            t_max[i - 1] = np.floor(t_max[i - 1] / T) * T + intersections[i - 2]['C_i']

        # Ensure t_min[i - 1] ≤ t_max[i - 1]
        if t_min[i - 1] > t_max[i - 1]:
            print(f"No feasible times after adjustment for intersection {i - 1}")
            break  # Or handle appropriately

# Display the feasible time intervals
for i in range(1, n_intersections + 1):
    print(f"Intersection {i}: t_min = {t_min[i]:.2f}s, t_max = {t_max[i]:.2f}s")

# plot the feasible time intervals, distance on the y axis and time on the x axis

t_min[-1] = tf
t_max[-1] = tf
list_intersection = [el['distance'] for el in intersections ]
list_intersection.append(final_distance)
list_intersection.insert(0, 0)



def compute_dijkstra(t_min, t_max, distances, t_jump=1):
    """
    Compute the optimal crossing times using Dijkstra's algorithm with energy costs.
    :param t_min: Array of minimum feasible times for intersections.
    :param t_max: Array of maximum feasible times for intersections.
    :param distances: List of distances for intersections.
    :param t_jump: Assumed jump time for velocity transitions.
    :return: List of optimal times for each intersection.
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
                speed = (distances[i + 1] - distances[i]) / dt
                if v_min <= speed <= v_max and t_min[i + 1] <= t2 <= t_max[i + 1]:
                    # Calculate energy cost using link_energy module
                    e_link = link_energy.compute_link_energy(distances[i + 1], distances[i], t2, t1)
                    if i > 0:  # For edges beyond the first
                        prev_speed = (distances[i] - distances[i - 1]) / (t1 - time_nodes[i - 1][0])
                        e_jump = link_energy.compute_jump_energy(prev_speed, speed, t_jump)
                    else:
                        e_jump = 0  # No jump energy for the first edge

                    cost = e_link + e_jump
                    G.add_edge((i, t1), (i + 1, t2), weight=cost)

    # Add final destination node
    for t in time_nodes[-1]:
        G.add_edge((num_intersections, t), "final", weight=0)

    # Run Dijkstra's algorithm
    source_node = (0, t_min[0])  # Start at the beginning
    path = nx.shortest_path(G, source=source_node, target="final", weight="weight")

    # Extract optimal times
    optimal_times = [node[1] for node in path[:-1]]  # Exclude the final node
    return optimal_times



# Apply the function
optimal_times = compute_dijkstra(t_min, t_max, list_intersection)

# Display results
for i, time in enumerate(optimal_times):
    print(f"Intersection {i + 1}: Optimal crossing time = {time:.2f}s")
    
    
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

# Constraints: Ensure crossing times remain sequential
constraints = []
for i in range(1, len(t_min)):
    constraints.append({'type': 'ineq', 'fun': lambda x, i=i: x[i] - t_min[i]})  # t_i >= t_min[i]
    constraints.append({'type': 'ineq', 'fun': lambda x, i=i: t_max[i] - x[i]})  # t_i <= t_max[i]
    constraints.append({'type': 'ineq', 'fun': lambda x, i=i: x[i] - x[i - 1] - 1e-3})  # t_i > t_{i-1}

# Initial guess: Use Dijkstra times directly
x0 = optimal_times.copy()

# Optimization bounds
bounds = [(0,0)]
print(optimal_times)

for i in range(1, len(t_min)):
    bound = (max(t_min[i], optimal_times[i] - 5), min(t_max[i], optimal_times[i] + 5))
    bounds.append(bound)

print(bounds)
print(t_min)

# Run the optimization
result = minimize(objective, x0, bounds=bounds, constraints=constraints, method='SLSQP')

# Display results
optimization_result = result.x 
if result.success:
    print("Optimization successful!")
    print("Optimal crossing times:")
    for i, t in enumerate(result.x):
        print(f"t_{i}: {t:.2f} seconds")
    print(f"Total energy cost: {result.fun:.2f}")
else:
    print("Optimization failed:", result.message)

# Debugging information
print("Optimal crossing times (Dijkstra):", optimal_times)
print("Bounds:", bounds)




# plot for every intersection all inside bound of green and outside of bound red

# plot the feasible time wiht optimal times computed and green/red intervals
plt.figure(figsize=(10, 5))
print(f'{t_min=}, {list_intersection=}')
plt.plot(t_min, list_intersection, label='Feasible Time Intervals')
plt.plot(t_max, list_intersection, label='Feasible Time Intervals')
plt.hlines(list_intersection, t_min, t_max, color='gray', alpha=0.5)

# plot optimal times
plt.scatter(optimization_result, list_intersection, color='red', label='Optimal Times')
# plot the line between the optimal times
for i in range(1, len(optimal_times)):
    plt.plot([optimization_result[i - 1], optimization_result[i]], [list_intersection[i - 1], list_intersection[i]], color='red')

# plot the bounds in green 

for i in range(1, len(t_min)):
    plt.plot(bounds[i], [list_intersection[i], list_intersection[i]], color='green', linewidth=2)
    


plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.grid(True)
plt.title('Feasible Time Intervals at Intersections')
plt.show()


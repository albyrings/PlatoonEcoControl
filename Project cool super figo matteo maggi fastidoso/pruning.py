import numpy as np
import matplotlib.pyplot as plt


# Constants
v_min = 5  # Minimum speed (m/s)
v_max = 15  # Maximum speed (m/s)
final_time = 200
final_distance = 1800
intersections = [
    {"distance": 300, "green_start": 13, "green_end": 23},
    {"distance": 600, "green_start": 3, "green_end": 13},
    {"distance": 900, "green_start": 28, "green_end": 38},
    {"distance": 1200, "green_start": 15, "green_end": 25},
    {"distance": 1550, "green_start": 5, "green_end": 15},
] 
n = len(intersections)
T = 30  # Traffic light cycle time (seconds)
tf = final_time

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

plt.figure(figsize=(10, 5))
print(f'{t_min=}, {list_intersection=}')
plt.plot(t_min, list_intersection, label='Feasible Time Intervals')
plt.plot(t_max, list_intersection, label='Feasible Time Intervals')
plt.hlines(list_intersection, t_min, t_max, color='gray', alpha=0.5)

plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.title('Feasible Time Intervals at Intersections')
plt.show()
        


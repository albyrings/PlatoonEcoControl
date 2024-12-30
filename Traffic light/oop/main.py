"""
    TODO: fix optimization shift for plotting
"""



from Simulation import Simulation
from Vehicle import Platoon, Vehicle
from TrafficLight import TrafficLight
from constants import *
import numpy as np
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
# Constants
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

# Define the updated s_i function
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

def pruning(t_min, t_max):
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
    
    return t_min, t_max

t_min, t_max = pruning(t_min, t_max)

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

optimal_times = compute_dijkstra(t_min, t_max, list_intersection)

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


def optimization_algorithm(t_min,t_max, optimal_times):
    constraints = []
    for i in range(1, len(t_min)):
        constraints.append({'type': 'ineq', 'fun': lambda x, i=i: x[i] - t_min[i]})  # t_i >= t_min[i]
        constraints.append({'type': 'ineq', 'fun': lambda x, i=i: t_max[i] - x[i]})  # t_i <= t_max[i]
        constraints.append({'type': 'ineq', 'fun': lambda x, i=i: x[i] - x[i - 1] - 1e-3})  # t_i > t_{i-1}

    # Initial guess: Use Dijkstra times directly
    x0 = optimal_times.copy()

    # Optimization bounds
    bounds = [(t_min[0],t_max[0])]
    print(optimal_times)

    for i in range(1, len(t_min)):
        bound = (max(t_min[i], optimal_times[i] - 5), min(t_max[i], optimal_times[i] + 5))
        bounds.append(bound)

    print(bounds)
    print(t_min)

    # Run the optimization
    result = minimize(objective, x0, bounds=bounds, constraints=constraints, method='SLSQP')
    return result, bounds

result,bounds = optimization_algorithm(t_min,t_max, optimal_times)

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

"""No Shift"""
def max_veichles(inner_bounds,time_for_vehicle = 3):
        l = []
        for i in range(len(inner_bounds)): 
            l.append((inner_bounds[i][1] - optimization_result[i+1]))
        return int(min(l)//time_for_vehicle)
    
    
def no_shift():
    inner_bounds = bounds.copy()
    inner_bounds.pop(0)
    inner_bounds.pop(-1)
   
    time_for_vehicle = 0.5

    plt.figure(figsize=(10, 5))
    plt.plot(t_min, list_intersection, label='Feasible Time Intervals')
    plt.plot(t_max, list_intersection, label='Feasible Time Intervals')
    plt.hlines(list_intersection, t_min, t_max, color='gray', alpha=0.5)

    # plot optimal times

    plt.scatter(optimization_result, list_intersection, color='b', label='Optimal Times')

    # plot the vehicle path 

    for i in range(1, len(t_min)):
        plt.plot(bounds[i], [list_intersection[i], list_intersection[i]], color='green', linewidth=2)
        
        
    for i in range(1, len(optimal_times)):
        plt.plot([optimization_result[i - 1], optimization_result[i]], [list_intersection[i - 1], list_intersection[i]],linewidth = 1, color='orange')
        for j in range(max_veichles(inner_bounds,time_for_vehicle)):
            plt.plot([optimization_result[i - 1] + time_for_vehicle*j, optimization_result[i]+time_for_vehicle*j] , [list_intersection[i - 1], list_intersection[i]],linewidth = 1, color='red')

    



    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.grid(True)
    plt.title('Feasible Time Intervals at Intersections')


    plt.show()


#no_shift()
def shift(bounds, optimization_result, t_min, t_max, n_of_vehicles = 3, time_for_vehicle = 3):
    inner_bounds = bounds.copy()
    inner_bounds.pop(0)
    inner_bounds.pop(-1)
 
    
    max_vehicles = 0
    t_cycle = []
    for intersection in intersections:
        t_cycle.append(intersection['T_gr_i'])
    max_vehicles = min(t_cycle)//time_for_vehicle
    
    if n_of_vehicles > max_vehicles:
        print(f"Number of vehicles saturated the intersection, {max_vehicles} is the maximum number of vehicles allowed")
        
        # divide the platoon of vehicles in independent platoons of less than max_vehicles, and recomputing the optimal times for each platoon
        
       #shift(bounds, optimization_result,t_min, t_max, min(max_vehicles,n_of_vehicles), 1, )
        platoon_number = n_of_vehicles
        passed_vehicles = max_vehicles
        #plt.figure(figsize=(10, 5))
        ix = 0
        while platoon_number > max_vehicles:
            platoon_number -= max_vehicles
            n_of_vehicles = min(platoon_number, max_vehicles)
            
            
            t_min_platoon = np.zeros(n_intersections + 2)
            t_max_platoon = np.zeros(n_intersections + 2)
            
            t_min_platoon[0] = time_for_vehicle*(passed_vehicles+1)
            t_max_platoon[0] = time_for_vehicle*(passed_vehicles+1)
            
            t_min_platoon, t_max_platoon = pruning(t_min_platoon, t_max_platoon)
            
            t_min_platoon[-1] = tf + time_for_vehicle*(passed_vehicles+1)
            t_max_platoon[-1] = tf + time_for_vehicle*(passed_vehicles+1)
            optimal_times_platoon = compute_dijkstra(t_min_platoon, t_max_platoon, list_intersection)
            result_platoons,bounds_platoon = optimization_algorithm(t_min_platoon,t_max_platoon, optimal_times_platoon)
            # Display results
            optimization_result_platoon = result_platoons.x 
            if result_platoons.success:
                print("Optimization successful platoon!")
                print("Optimal crossing times:")
                for i, t in enumerate(result_platoons.x):
                    print(f"t_{i}: {t:.2f} seconds")
                print(f"Total energy cost: {result_platoons.fun:.2f}")
            else:
                print("Optimization failed platoon:", result_platoons.message)
                
            shift_optimal_times_platoon = optimization_result_platoon.copy()
    
    
    
    
            for i in range(len(shift_optimal_times_platoon)):
                shift_optimal_times_platoon[i] = max(shift_optimal_times_platoon[i] - time_for_vehicle*(n_of_vehicles - 1), bounds[i][0])
                
            
            plt.figure(figsize=(10, 5))
            plt.plot(t_min_platoon, list_intersection, label='Feasible Time Intervals')
            plt.plot(t_max_platoon, list_intersection, label='Feasible Time Intervals')
            plt.hlines(list_intersection, t_min_platoon, t_max_platoon, color='gray', alpha=0.5)

            # plot optimal times

            plt.scatter(shift_optimal_times_platoon, list_intersection, color='b', label='Optimal Times')

            #plot the vehicle path 

            for i in range(1, len(t_min)):
                plt.plot(bounds_platoon[i], [list_intersection[i], list_intersection[i]], color='green', linewidth=2)
                
                
            for i in range(1, len(shift_optimal_times_platoon)):
                plt.plot([shift_optimal_times_platoon[i - 1], shift_optimal_times_platoon[i]], [list_intersection[i - 1], list_intersection[i]],linewidth = 1, color='orange')
                for j in range(n_of_vehicles):
                    plt.plot([shift_optimal_times_platoon[i - 1] + time_for_vehicle*j, shift_optimal_times_platoon[i]+time_for_vehicle*j] , [list_intersection[i - 1], list_intersection[i]],linewidth = 1, color='red')

            



            plt.xlabel('Time (s)')
            plt.ylabel('Distance (m)')
            plt.grid(True)
            plt.title(f'Feasible Time Intervals at Intersections {ix}')
            ix += 1
            
            passed_vehicles += n_of_vehicles
            
        n_of_vehicles = max_vehicles                
            
                
            
            
            
            
            
            
            

        
    shift_optimal_times = optimization_result.copy()
    
    
    
    
    for i in range(len(shift_optimal_times)):
        shift_optimal_times[i] = max(shift_optimal_times[i] - time_for_vehicle*(n_of_vehicles - 1), bounds[i][0])
           
    
    plt.figure(figsize=(10, 5))
    plt.plot(t_min, list_intersection, label='Feasible Time Intervals')
    plt.plot(t_max, list_intersection, label='Feasible Time Intervals')
    plt.hlines(list_intersection, t_min, t_max, color='gray', alpha=0.5)

    # plot optimal times

    plt.scatter(shift_optimal_times, list_intersection, color='b', label='Optimal Times')

    # plot the vehicle path 

    for i in range(1, len(t_min)):
        plt.plot(bounds[i], [list_intersection[i], list_intersection[i]], color='green', linewidth=2)
        
        
    for i in range(1, len(shift_optimal_times)):
        plt.plot([shift_optimal_times[i - 1], shift_optimal_times[i]], [list_intersection[i - 1], list_intersection[i]],linewidth = 1, color='orange')
        for j in range(n_of_vehicles):
            plt.plot([shift_optimal_times[i - 1] + time_for_vehicle*j, shift_optimal_times[i]+time_for_vehicle*j] , [list_intersection[i - 1], list_intersection[i]],linewidth = 1, color='red')

    



    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.grid(True)
    plt.title('Feasible Time Intervals at Intersections')


    plt.show()

shift(bounds, optimization_result,t_min, t_max, 31, 1, )
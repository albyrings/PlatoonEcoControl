
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

from pruning import *
from dikijstra import *
from link_energy import *
from constants import *

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


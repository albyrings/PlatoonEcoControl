import numpy as np
from scipy.integrate import quad

# Constants for energy computation
b1 = 0.1  # Coefficient for link energy
b2 = 0.05  # Coefficient for link energy
h1 = 1.0  # Coefficient for driving input
h2 = 0.2  # Coefficient for driving input
h3 = 0.1  # Coefficient for driving input
h0 = 0.05  # Constant driving input offset

def compute_link_energy(d_i, d_prev, t_i, t_prev):
    """
    Compute the energy consumption for a link.
    
    :param d_i: Distance at the current node (meters).
    :param d_prev: Distance at the previous node (meters).
    :param t_i: Time at the current node (seconds).
    :param t_prev: Time at the previous node (seconds).
    :return: Link energy consumption (E_link).
    """
    delta_t = t_i - t_prev
    v_bar = (d_i - d_prev) / delta_t  # Average velocity
    u_bar = (1 / h1) * (h2 * v_bar**2 + h3 * v_bar + h0)  # Driving input
    e_link = delta_t * (b1 * u_bar * v_bar + b2 * u_bar**2)
    return e_link

def compute_jump_energy(v_start, v_end, t_jump):
    """
    Compute the energy consumption for a jump (velocity transition).
    
    :param v_start: Initial velocity (m/s).
    :param v_end: Final velocity (m/s).
    :param t_jump: Duration of the jump (seconds).
    :return: Jump energy consumption (E_jump).
    """
    def integrand(t):
        v_t = v_start + (v_end - v_start) * (t / t_jump)  # Linear velocity transition
        u_t = (1 / h1) * (h2 * v_t**2 + h3 * v_t + h0)  # Driving input
        return b1 * u_t * v_t + b2 * u_t**2

    e_jump, _ = quad(integrand, 0, t_jump)
    return e_jump

def compute_total_energy(distances, times, t_jump=1):
    """
    Compute the total energy consumption for a path.
    
    :param distances: List of distances for nodes (meters).
    :param times: List of corresponding times for nodes (seconds).
    :param t_jump: Assumed jump time for velocity transitions (seconds).
    :return: Total energy consumption (E_total).
    """
    total_energy = 0
    for i in range(1, len(distances)):
        # Link energy
        e_link = compute_link_energy(distances[i], distances[i - 1], times[i], times[i - 1])
        
        # Jump energy
        if i > 1:
            v_start = (distances[i - 1] - distances[i - 2]) / (times[i - 1] - times[i - 2])
            v_end = (distances[i] - distances[i - 1]) / (times[i] - times[i - 1])
            e_jump = compute_jump_energy(v_start, v_end, t_jump)
        else:
            e_jump = 0  # No jump energy for the first edge
        
        total_energy += e_link + e_jump
    return total_energy

def energy_details(distances, times, t_jump=1):
    """
    Compute energy details (link and jump energy) for each edge.
    
    :param distances: List of distances for nodes (meters).
    :param times: List of corresponding times for nodes (seconds).
    :param t_jump: Assumed jump time for velocity transitions (seconds).
    :return: List of dictionaries with energy details for each edge.
    """
    energy_data = []
    for i in range(1, len(distances)):
        # Link energy
        e_link = compute_link_energy(distances[i], distances[i - 1], times[i], times[i - 1])
        
        # Jump energy
        if i > 1:
            v_start = (distances[i - 1] - distances[i - 2]) / (times[i - 1] - times[i - 2])
            v_end = (distances[i] - distances[i - 1]) / (times[i] - times[i - 1])
            e_jump = compute_jump_energy(v_start, v_end, t_jump)
        else:
            e_jump = 0  # No jump energy for the first edge
        
        energy_data.append({
            "edge": i,
            "e_link": e_link,
            "e_jump": e_jump,
            "total": e_link + e_jump
        })
    return energy_data

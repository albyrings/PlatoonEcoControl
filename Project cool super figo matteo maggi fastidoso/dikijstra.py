import numpy as np
import networkx as nx
import link_energy
import pruning

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
                if pruning.v_min <= speed <= pruning.v_max and t_min[i + 1] <= t2 <= t_max[i + 1]:
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


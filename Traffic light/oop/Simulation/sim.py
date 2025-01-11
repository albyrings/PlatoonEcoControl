import numpy as np
import math
import matplotlib.pyplot as plt
from act import *# Assumendo che le funzioni precedenti siano salvate in shift_code.py

# Configurazioni iniziali per il test
n_of_vehicles = 13  # Numero di veicoli nel plotone
time_for_vehicle = 1  # Tempo di attraversamento tra veicoli
disturbance_vehicle = 4  # Veicolo che subisce il disturbo

def simulate_platoon_shift(n_of_vehicles, time_for_vehicle, disturbance_vehicle):
    """
    Simula il movimento del plotone con un disturbo su un veicolo casuale.
    """
    global bounds, optimization_result, t_min, t_max

    # Simula i tempi di attraversamento iniziali del plotone
    
    
    print(f"Simulazione iniziale del movimento del plotone con {n_of_vehicles} veicoli")
    shift_sim(n_of_vehicles, time_for_vehicle)

    # Introduci il disturbo sul veicolo specifico
    print(f"Introduzione del disturbo sul veicolo {disturbance_vehicle}")
    disturbance_time = 4 #np.random.uniform(3.5, 5)  # Durata del disturbo (oltre il limite di 3s)
    if disturbance_time > time_for_vehicle and disturbance_vehicle > 0:
        
        print(f"Richiamo della funzione shift per gli altri veicoli")
        remaining_vehicles = n_of_vehicles - disturbance_vehicle
        n_of_disturbance = math.ceil(disturbance_time / time_for_vehicle)
        new_n_of_vehichle = n_of_vehicles + n_of_disturbance
        print(f"Numero di veicoli rimanenti: {remaining_vehicles}", f"Numero di veicoli aggiunti: {n_of_disturbance}", f"Numero di veicoli totali: {new_n_of_vehichle}")
        shift_sim(new_n_of_vehichle, time_for_vehicle, disturbance_vehicle,n_of_disturbance)

# Chiamata alla funzione di simulazione
simulate_platoon_shift(n_of_vehicles, time_for_vehicle, disturbance_vehicle)

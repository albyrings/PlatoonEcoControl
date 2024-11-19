import numpy as np

# TODO:
# the shift to the last moment of previous green phase for t_max seems hard coded
# need to check and fix, i copied the matplab code but was of poor quality
# and the pseudo code don't really handle it well either (or so it seems)

# For platoon:
# assume a new variable l wich is the lenght of platoon
# try to find of much of l can pass at each green phase
#l = 20 #meters


# made a callable function or class to use it online
def pruning_algorithm(simulation):
  distance = simulation.distances
  tf = simulation.tf
  v_max = simulation.v_max
  v_min = simulation.v_min
  T_cycle = simulation.T
  T_green = simulation.T_gr
  s = simulation.state
  offset = simulation.passo
  time = simulation.time
  n_intersections = simulation.n 
  teta = simulation.teta
  t_min = [0]
  t_max = [0]
  for intersection in range(1,n_intersections+1):


    t_min_temp = ((distance[(intersection)] - distance[intersection-1])/v_max + t_min[intersection-1])
    t_min_temp = np.round(t_min_temp*(1/offset))/(1/offset)
    t_min.append(t_min_temp)

    t_max_temp = (distance[intersection] - distance[intersection-1])/v_min + t_max[intersection-1]

    if intersection < n_intersections:
      t_max_temp = np.minimum(t_max_temp, tf - (distance[n_intersections+1] - distance[intersection])/v_max)
      t_max_temp = np.round(t_max_temp*(1/offset))/(1/offset)

      t_max.append(t_max_temp)
    else:
      t_max_temp = np.round(t_max_temp*(1/offset))/(1/offset)
      t_max.append(t_max_temp)


    #given now ti_min and ti_max
    #I check if the traffic light of the i-th intersection at the
    #given time is in the green state. If it is not, ti_min is set to the first
    #instant of the next green phase, and ti_max is anticipated to the last
    #instant of the previous green phase.

    coordinate_min = np.where(np.abs(time - t_min[intersection]) < offset * 0.2)[0]
    if coordinate_min.size == 0:
      coordinate_min = np.array([0])

    if s[intersection][coordinate_min] != 1:
      t_min[intersection] = (np.floor(t_min[intersection]/T_cycle) + 1) * T_cycle + teta[intersection-1] + offset

    coordinate_max = np.where(np.abs(time - t_max[intersection]) < offset * 0.2)[0]

    if coordinate_max.size == 0:
      coordinate_max = np.array([len(time)-1])

    if s[intersection][coordinate_max] != 1: ###.any() #t_max anticipato all'ultimo istante della precedente fase verde
      if intersection == 1:
          t_max[intersection] = np.floor(t_max[intersection]/T_cycle)*T_cycle - (T_cycle-T_green) + teta[intersection - 1] - offset
      else:
        t_max[intersection] = np.floor(t_max[intersection]/T_cycle)*T_cycle + T_green + teta[intersection - 1]



  # missing v
  # what does below?
  # problem prob here??
  # t_max is 60 with v of 5, we can use any v in [5,15] (in the paper is 6.18 the v for the fist intersect)

  # trying to follow uglt matlab code
  t_max.append(tf)
  t_min.append(tf)

  if t_max[n_intersections+1] <= t_max[n_intersections] or ((distance[n_intersections+1] - distance[n_intersections])/(t_max[n_intersections+1]-t_max[n_intersections])) > v_max:
    t_max_temp = t_max[n_intersections+1] - ((distance[n_intersections+1]-distance[n_intersections])/v_max)
    t_max[n_intersections] = np.round(t_max_temp*(1/offset))/(1/offset)
    coordinate_max = np.where(np.abs(time - t_max[n_intersections]) < offset * 0.2)[0]
    if coordinate_max.size == 0:
        coordinate_max = np.array([len(time)-1])

    if s[n_intersections-1][coordinate_max] != 1:
        t_max_temp = np.floor(t_max[n_intersections]/T_cycle)*T_cycle + teta[n_intersections-1] -1 - T_green
        t_max[n_intersections] = np.round(t_max_temp*(1/offset))/(1/offset)





  for i in range(n_intersections,1,-1):

        if t_max[i] <= t_max[i-1] or ((distance[i] - distance[i-1])/(t_max[i]-t_max[i-1])) > v_max:

          t_max_temp = t_max[i] - (distance[i]-distance[i-1])/v_max
          t_max[i-1] = np.round(t_max_temp*(1/offset))/(1/offset)
          coordinate_max = np.where(np.abs(time - t_max[i-1]) < offset * 0.2)[0]
          if coordinate_max.size == 0:
              coordinate_max = np.array([len(time)-1])

          if s[i-1][coordinate_max] != 1:
            if teta[i-1]>(T_cycle-T_green):
              t_max_temp = np.floor(t_max[i-1]/T_cycle)*T_cycle + teta[i-1] - (T_cycle - T_green)
              t_max[i-1] = np.round(t_max_temp*(1/offset))/(1/offset)
            else:

              t_max_temp = np.floor(t_max[i-1]/T_cycle)*T_cycle + T_green + teta[i-1]
              t_max[i-1] = np.round(t_max_temp*(1/offset))/(1/offset)
              
  return t_min, t_max


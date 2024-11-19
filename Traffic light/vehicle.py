import numpy as np
from sympy import *
import networkx as nx
import copy
from networkx.algorithms.shortest_paths.weighted import dijkstra_path
import scipy


class Vehicle:
    def __init__(self, traffic_signal):
        self.traffic_signal = traffic_signal
        self.t_min = [0]
        self.t_max = [0]
        self.offset = traffic_signal.offset
        self.s = traffic_signal.s
        self.vehicle_lenght = 3
        self.optimal_path = []
        self.position = 0



    def set_position(self, position):
        self.position = position
        
    def perform_calculations(self):
        s_matrix = self.traffic_signal.get_signal_matrix()
        # Perform some calculations with s_matrix
        print(s_matrix)

    def pruning_algorithm(self):
        n_intersections, T_cycle,T_green, tf, teta, offset, v_min, v_max, distance, time = self.traffic_signal.get_parameters()
        s = self.traffic_signal.get_signal_matrix()

        for intersection in range(1,n_intersections+1):
          t_min_temp = ((distance[(intersection)] - distance[intersection-1])/v_max + self.t_min[intersection-1])
          t_min_temp = np.round(t_min_temp*(1/offset))/(1/offset)
          self.t_min.append(t_min_temp)

          t_max_temp = (distance[intersection] - distance[intersection-1])/v_min + self.t_max[intersection-1]

          if intersection < n_intersections:
            t_max_temp = np.minimum(t_max_temp, tf - (distance[n_intersections+1] - distance[intersection])/v_max)
            self.t_max_temp = np.round(t_max_temp*(1/offset))/(1/offset)

            self.t_max.append(t_max_temp)
          else:
            t_max_temp = np.round(t_max_temp*(1/offset))/(1/offset)
            self.t_max.append(t_max_temp)


          #given now ti_min and ti_max
          #I check if the traffic light of the i-th intersection at the
          #given time is in the green state. If it is not, ti_min is set to the first
          #instant of the next green phase, and ti_max is anticipated to the last
          #instant of the previous green phase.

          coordinate_min = np.where(np.abs(time - self.t_min[intersection]) < offset * 0.2)[0]
          if coordinate_min.size == 0:
            coordinate_min = np.array([0])

          if s[intersection][coordinate_min] != 1:
           self.t_min[intersection] = (np.floor(self.t_min[intersection]/T_cycle) + 1) * T_cycle + teta[intersection-1] + offset

          coordinate_max = np.where(np.abs(time - self.t_max[intersection]) < offset * 0.2)[0]

          if coordinate_max.size == 0:
            coordinate_max = np.array([len(time)-1])

          if s[intersection][coordinate_max] != 1: ###.any() #t_max anticipato all'ultimo istante della precedente fase verde
            if intersection == 1:
                self.t_max[intersection] = np.floor(self.t_max[intersection]/T_cycle)*T_cycle - (T_cycle-T_green) + teta[intersection - 1] - offset
            else:
              self.t_max[intersection] = np.floor(self.t_max[intersection]/T_cycle)*T_cycle + T_green + teta[intersection - 1]



        # missing v
        # what does below?
        # problem prob here??
        # t_max is 60 with v of 5, we can use any v in [5,15] (in the paper is 6.18 the v for the fist intersect)

        # trying to follow uglt matlab code
        self.t_max.append(tf)
        self.t_min.append(tf)

        if self.t_max[n_intersections+1] <= self.t_max[n_intersections] or ((distance[n_intersections+1] - distance[n_intersections])/(self.t_max[n_intersections+1]-self.t_max[n_intersections])) > v_max:
          t_max_temp = self.t_max[n_intersections+1] - ((distance[n_intersections+1]-distance[n_intersections])/v_max)
          self.t_max[n_intersections] = np.round(t_max_temp*(1/offset))/(1/offset)
          coordinate_max = np.where(np.abs(time - self.t_max[n_intersections]) < offset * 0.2)[0]
          if coordinate_max.size == 0:
              coordinate_max = np.array([len(time)-1])

          if s[n_intersections-1][coordinate_max] != 1:
              t_max_temp = np.floor(self.t_max[n_intersections]/T_cycle)*T_cycle + teta[n_intersections-1] -1 - T_green
              self.t_max[n_intersections] = np.round(t_max_temp*(1/offset))/(1/offset)





        for i in range(n_intersections,1,-1):

              if self.t_max[i] <= self.t_max[i-1] or ((distance[i] - distance[i-1])/(self.t_max[i]-self.t_max[i-1])) > v_max:

                t_max_temp = self.t_max[i] - (distance[i]-distance[i-1])/v_max
                self.t_max[i-1] = np.round(t_max_temp*(1/offset))/(1/offset)
                coordinate_max = np.where(np.abs(time - self.t_max[i-1]) < offset * 0.2)[0]
                if coordinate_max.size == 0:
                    coordinate_max = np.array([len(time)-1])

                if s[i-1][coordinate_max] != 1:
                  if teta[i-1]>(T_cycle-T_green):
                    t_max_temp = np.floor(self.t_max[i-1]/T_cycle)*T_cycle + teta[i-1] - (T_cycle - T_green)
                    self.t_max[i-1] = np.round(t_max_temp*(1/offset))/(1/offset)
                  else:

                    t_max_temp = np.floor(self.t_max[i-1]/T_cycle)*T_cycle + T_green + teta[i-1]
                    self.t_max[i-1] = np.round(t_max_temp*(1/offset))/(1/offset)

    def get_pruned_matrix(self):
       # need a deep copy for later use
      #distruggiam tutto ciò che è fuori dal pruning
      temp_s = copy.deepcopy(self.s)


      for i in range(temp_s.shape[0]):
        for j in range(1,temp_s.shape[1]+1):
          if j*self.offset < self.t_min[i] or j*self.offset > self.t_max[i]:

            temp_s[i][j-1] = 0

      temp_s[i][j-1]=1 #poiché j*offset è maggiore di tmax nell'ultimo ciclo lo pone a 0 ma a noi interessa quindi lo poniamo ad 1 a mano
      return temp_s

    def compute_node_matrix(self,s):
      print(s)

      def median(v,i):

        nodes = []
        switch = False
        #print(self.s.shape)

        for j in range(0, s.shape[1]):

          if v[j] == 0 and switch == True:
            switch = False

          elif v[j] == 1 and switch == False:

            switch = True
            counter = 0
            while j <= s.shape[1] and v[j] == 1:

              counter += 1
              j += 1
            nodes.append(j-(counter//2))

        return nodes
      node_mat = []

      for i in range(1,s.shape[0]-1):
        node_mat.append(median(s[i],i))
      node_mat.append([s.shape[1]-1])
      node_mat.insert(0,[0])
      print(f'node_mat={node_mat}')
      return node_mat

    def compute_graph_cost(self,node_mat,s):
        t = symbols('t')

        #definisco le funzioni
        p = 0 # Momento
        v = [0,0]
        m = 1190 # Massa Kg
        Rt = 6.066  # Rapporto di trasmissione del cambio
        r = 0.2848 # Raggio della ruota
        g = 9.81  # Accelerazione di gravità (costante)
        a = 1.5 #paper acc fixed
        a0 = 113.5
        a1 = 0.774
        a2 = 0.4212
        b1 = Rt/r
        b2 = 0.1515
        h1 = Rt/(m*r)
        h2 = a2/m
        h3 = a1/m
        h0 = a0/m #posto alpha = 0 ==>  gsin(alpha) = 0

        E_total_list = []
        E_jump_list = []

        def v(delta_d,delta_t):
          return (delta_d/delta_t)

        def v_dot(t,v):
          return h1*u(v)-h2*v_t(t,v)**2-h3*v_t(t,v)-h0

        def v_t(t,v_in):
          return v_in + a*t

        def u(v):
          return 1/h1*(h2*v**2+h3*v+h0)

        def E_link(v):
          return delta_t*(b1*u(v)*v+b2*(u(v)**2))

        def u_t(t,v,v_in):
          return 1/h1*(v_dot(t,v) + h2*(v_t(t,v)**2) + h3*v_t(t,v) + h0)

        def E_jump(t,v,v_in,t_jump):
          f = b1*u_t(t,v,v_dot(t,v))*v_t(t,v_in) + b2*u_t(t,v,v_dot)**2
          intr = integrate(f, (t,0,t_jump))
          E_jump_list.append(intr)
          return intr


        G = nx.DiGraph()

        for ix,layer in enumerate(node_mat):
          for node in layer:
            if ix + 1 < len(node_mat):
              for next_node in node_mat[ix+1]:
                delta_t = next_node*self.offset - node*self.offset
                delta_d = self.traffic_signal.distance[ix+1]-self.traffic_signal.distance[ix]
                v_in = v(delta_t,delta_d)
                v_useful =  v(delta_t,delta_d)
                t_jump = next_node*self.offset - node*self.offset
                #calcolo E_link
                E_link_val = E_link(v_useful)
                #calcolo E_jump
                E_jump_val = E_jump(t,v_useful,v_in,t_jump)
                E_total = E_link_val + E_jump_val
                E_total_list.append(E_total)

                if E_total < 0:
                  print('ciao')
                G.add_edge(node,next_node, weight = E_total)

        # Finding the optimal path

        path = dijkstra_path(G, 0, s.shape[1]-1, weight='weight')
        self.optimal_path = path
        return path, E_total_list, E_jump_list

    def compute_green_time_range(self,s, optimal_path = None):
      time_range_index_list = []

      for ix,intersection in enumerate(self.optimal_path):
        if ix == 0 or ix == s.shape[0] or intersection == s.shape[1]-1:
          continue



        counter = 0

        while s[ix][intersection-counter] != 0:
          counter += 1

        start_range_index = intersection-counter+1


        counter = 0

        while s[ix][intersection+counter] != 0:
          counter += 1

        end_range_index = intersection+counter-1


        time_range_index_list.append((start_range_index,end_range_index))

      return time_range_index_list

    def _compute_time_vector_for_optimization(self):
      t_min_opt = []
      t_max_opt = []

      for i in range(len(self.t_max)):
        if i == 0 or i == len(self.t_max)-1:
          continue
        else:
          t_min_opt.append(self.t_min[i])
          t_max_opt.append(self.t_max[i])

      return t_min_opt, t_max_opt

    def _compute_velocity(self,time,distance):
      delta_d_vec = []
      delta_t_vec = []

      for ix,d in enumerate(distance):
        if ix == 0:
          continue

        delta_d = d - distance[ix-1]
        delta_d_vec.append(delta_d)

      for ix,t in enumerate(time):
        if ix == 0:
          continue

        delta_t = t - time[ix-1]
        delta_t_vec.append(delta_t)




      return np.array(delta_d_vec)/np.array(delta_t_vec)






    def compute_optimal_velocity(self,s, optimal_path = None):
      # utility fuction call to build vectors for optimization
      t_min_opt, t_max_opt = self._compute_time_vector_for_optimization()
      n_intersections, T_cycle,T_green, tf, teta, offset, v_min, v_max, distance, time = self.traffic_signal.get_parameters()
      t0 = 0
      tf = tf
      # below must be made global param of car, beautfiy, just TEMP!!
      p = 0 # Momento
      v = [0,0]
      m = 1190 # Massa Kg
      Rt = 6.066  # Rapporto di trasmissione del cambio
      r = 0.2848 # Raggio della ruota
      g = 9.81  # Accelerazione di gravità (costante)
      a = 1.5 #paper acc fixed
      a0 = 113.5
      a1 = 0.774
      a2 = 0.4212
      b1 = Rt/r
      b2 = 0.1515
      h1 = Rt/(m*r)
      h2 = a2/m
      h3 = a1/m
      h0 = a0/m #posto alpha = 0 ==>  gsin(alpha) = 0

      time_symbols = []
      time_vector = []
      velocity_vector = []

      lb = [] #lower bound for optimization
      ub = [] #upper bound for optimization

      E_jump_vector = []



      for n in range(1,n_intersections+1):
        symb = 't' + str(n)
        n = symbols(symb)
        time_symbols.append(n)

      time_symbols = np.array(time_symbols)

      def v(delta_d,delta_t):
        return (delta_d/delta_t)

      def v_dot(t,v):
        return h1*u(v)-h2*v_t(t,v)**2-h3*v_t(t,v)-h0

      def v_t(t,v_in):
        return v_in + a*t

      def u(v):
        return 1/h1*(h2*v**2+h3*v+h0)

      def u_t(t,v,v_in):
        return 1/h1*(v_dot(t,v) + h2*(v_t(t,v)**2) + h3*v_t(t,v) + h0)

      def u_bar(v):
        return (1/h1)*(h2*v**2 + h3*v + h0)

      def P_bar(v):
        return b1*u_bar(v)*v + b2*u_bar(v)**2

      def E_jump(t,v,v_in,t_jump):
        f = b1*u_t(t,v,v_dot(t,v))*v_t(t,v_in) + b2*u_t(t,v,v_dot)**2
        intr = integrate(f, (t,0,t_jump))
        return intr

      def time_function(time_vect):
        t_computed = []
        temp = time_vect.tolist()
        temp.append(tf)
        temp.insert(0,t0)
        time_vect = np.array(temp)
        for ix,t in enumerate(time_vect):
          if ix -  1 < 0 :
            continue
          t_computed.append(time_vect[ix-1]-t)
        t_computed = np.array(t_computed)
        return t_computed

      def velocity_function(time_vect):
        v_vector =  []
        temp = time_vect.tolist()
        temp.append(tf)
        temp.insert(0,t0)
        time_vect = np.array(temp)

        for ix,t in enumerate(time_vect):
          if ix -  1 < 0 :
            continue

          d_now = distance[ix]
          d_prev = distance[ix-1]
          delta_d = d_now - d_prev
          v_vector.append(delta_d/(t-time_vect[ix-1]))

        v_vector = np.array(v_vector)

        return v_vector


      def J_function(time_symbols):
        # giga ugly heuristics (assumed dot product to allow function to retur scalar)
        f = np.dot(time_function(time_symbols),P_bar(velocity_function(time_symbols)))+ np.sum(E_jump_vector)
        return f




      # compute E_jump summation value

      t = symbols('t')
      for ix,intersection in enumerate(self.optimal_path):
        if ix + 1 > len(self.optimal_path)-1:
          continue



        next_intersection = self.optimal_path[ix+1]
        delta_t = next_intersection*offset - intersection*offset
        delta_d = distance[ix+1]-distance[ix]
        v_in = v(delta_t,delta_d)
        v_useful = v(delta_t,delta_d)

        t_jump = next_intersection*offset - intersection*offset

        E_jump_val = E_jump(t,v_useful,v_in,t_jump)
        E_jump_vector.append(E_jump_val)

      E_jump_vector = np.array(E_jump_vector)


      # compute t bounds for optimizations

      for ix,time in enumerate(self.compute_green_time_range(s, optimal_path=self.optimal_path)):
        start_range = time[0]*offset
        end_range = time[1]*offset

        lower_bound = np.maximum(start_range, t_min_opt[ix])
        upper_bound = np.minimum(end_range, t_max_opt[ix])

        # ugly heruisitcs but no time to fix now
        if lower_bound > upper_bound:
          lower_bound = start_range

        lb.append(lower_bound)
        ub.append(upper_bound)

      ub = np.array(ub)
      lb= np.array(lb)
      print(f'ub:{ub}')
      print(f'lb:{lb}')

      bounds = scipy.optimize.Bounds(lb= lb, ub=ub, keep_feasible=True)

      initial_guess = []

      for ix,t in enumerate(self.optimal_path):
        if ix == 0 or ix == len(self.optimal_path)-1:
          continue
        initial_guess.append(t*offset)

      initial_guess = np.array(initial_guess)
      res = scipy.optimize.minimize(J_function, initial_guess, method='SLSQP', bounds = bounds)

      optimal_times = list(res.x)
      optimal_times.append(200)
      optimal_times.insert(0,0)



      velocity_vec = self._compute_velocity(np.array(optimal_times), np.array(distance))
      return velocity_vec


    def get_distance(self,lead_distance):

      return self.vehicle_lenght + lead_distance

    def velocity_check(self,velocities,s, lead_distance, optimal_path = None):
      """Chcek if velocity allow to follow lead, if then do it"""
      distances =  self.traffic_signal.distance
      n_intersections = self.traffic_signal.n_intersections

      limit_time = self.compute_green_time_range(s, optimal_path=self.optimal_path)
      print(f'limit_time: {limit_time}')
      upper_bound_list = []
      
      time = 0
      for t in limit_time:
        upper_bound_list.append(t[1]*self.traffic_signal.offset)
      

      print(f'upper_bound: {upper_bound_list}')

      for i in range(n_intersections + 1):
        if i+1 > n_intersections:
          continue
        #print("computed time:")
        #print((-distances[i] + distances[i+1] + self.get_distance(lead_distance) )/velocities[i])
        #print("red light:")
        print(upper_bound_list[i])
        
        cross_time = (-distances[i] + distances[i+1] + self.get_distance(lead_distance) )/velocities[i]
        time += cross_time

        valid_lead =  time < upper_bound_list[i]

        if not valid_lead:
         return False


      return True









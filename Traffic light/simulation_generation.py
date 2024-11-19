import numpy as np


class Simulation:
    def __init__(self, n, T, T_gr, tf, teta, passo, v_min, v_max):
        
        self.state, self.distances = self.initialize(n, T, T_gr, tf, teta, passo, v_min, v_max)
        self.T = T
        self.T_gr = T_gr
        self.tf = tf
        self.teta = teta
        self.passo = passo
        self.v_min = v_min
        self.v_max = v_max
        self.time = np.arange(passo, tf + passo, passo)
        self.n = n
        
    def initialize(self, n, T, T_gr, tf, teta, passo, v_min, v_max):
        
        
        t = np.zeros(n + 2)
        t[0] = passo
        ti_min = np.zeros(n + 2)
        ti_max = np.zeros(n + 2)

        # Generate intersection distances
        n = n + 1
        distance = [0]*n
        for j in range(1,n):
            if j !=n :
                distance[j]=distance[j-1]+300

            if j==n-2:
                 distance[j]=distance[j-1]+350
            if j==n-1:
                distance[j]=distance[j-1]+450

        distance.append(distance[-1]+200)

        # Calculate number of cycles
        cicli = int(tf // T)

        # Create time vector and traffic light state matrix
        time = np.arange(passo, tf + passo, passo)
        l = len(time)
        s = np.zeros((n - 1, l))
        

        # Determine traffic light states based on offsets and cycles
        for i in range(n - 1):
            for z in range(cicli + 1):
                for t_val in time:
                    if teta[i] < (T - T_gr):
                        if (t_val - teta[i]) > (z * T) and (t_val - teta[i]) <= (z * T + T_gr):
                            tt = np.where(np.abs(time - t_val) < passo * 0.1)[0]
                            s[i, tt] = 1
                    else:
                        if t_val <= (teta[i] - (T - T_gr)) and z == 0:
                            tt = np.where(np.abs(time - t_val) < passo * 0.1)[0]
                            s[i, tt] = 1
                        elif (t_val - teta[i]) > (z * T) and (t_val - teta[i]) <= (z * T + T_gr):
                            tt = np.where(np.abs(time - t_val) < passo * 0.1)[0]
                            s[i, tt] = 1
        
        ss = np.zeros(time.shape[0])
        ss[0]=1
        sss = np.zeros((7,time.shape[0]))
        sss[0] = ss
        for k in range(0,s.shape[0]):
            sss[k+1] = s[k]

        ss2 = np.zeros(time.shape[0])
        ss2[-1] = 1
        sss[6] = ss2
        

        s = sss
        return s,distance
        








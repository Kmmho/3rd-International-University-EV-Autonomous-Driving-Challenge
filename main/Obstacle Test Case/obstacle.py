import numpy as np

class obstacle_scenario():
    def __init__(self):
        self.obs_x = None
        self.obs_y = None
        self.obs_R = None
        self.obs_u = None 
        self.obs_v = None
        self.num_obs = 0
        
    def set_initial_obstacle(self, point_UTM):
        # 최초탐지 장애물
        x = point_UTM[:, 0]
        y = point_UTM[:, 1]
        self.obs_x = np.array(x)
        self.obs_y = np.array(y)
        self.obs_u = np.zeros(len(x))
        self.obs_v = np.zeros(len(x))
        self.obs_R = 0.5 * np.ones(len(x))
        self.num_obs = len(self.obs_x)
        
    def set_additory_obstacle(self, point_UTM):
        # 추가 탐지 장애물
        x = point_UTM[:, 0]
        y = point_UTM[:, 1]
        self.obs_x = np.hstack(self.obs_x, x)
        self.obs_y = np.hstack(self.obs_y, y)
        self.obs_u = np.hstack(self.obs_u, np.zeros(len(x)))
        self.obs_v = np.hstack(self.obs_v, np.zeros(len(x)))
        self.obs_R = 0.5 * np.ones(len(x))
        self.num_obs = len(self.obs_x)


    def erase_obstacle(self):
        self.obs_x = None
        self.obs_y = None
        self.obs_R = None
        self.obs_u = None 
        self.obs_v = None
        self.num_obs = 0
        
        
        
    def set_obstacle(self, num):
        if num == 1:
            self.obs_x = np.array([1500.0, 2400.0, 4200.0])
            self.obs_y = np.array([1500.0, 1800.0, 4100.0])
            self.obs_R = np.array([400.0, 300.0, 200.0])
            self.obs_u = np.array([0.0, 0.0, 0.0])
            self.obs_v = np.array([0.0, 0.0, 0.0])
            
        elif num == 2:
            self.obs_x = np.array([1500.0,2500.0, 5000.0])
            self.obs_y = np.array([500.0, 800.0, 400.0])
            self.obs_R = np.array([200.0,300.0, 500.0])
            self.obs_u = np.array([0.0, 0.0, 0.0])
            self.obs_v = np.array([0.0, 0.0, 0.0])
            
        elif num == 3:
            self.obs_x = np.array([2000.0,2100.0])
            self.obs_y = np.array([500.0, 500.0])
            self.obs_R = np.array([200.0, 300.0])
            self.obs_u = np.array([-20.0, 10.0])
            self.obs_v = np.array([0.0, 0.0])

        elif num == 4:
            self.obs_x = np.array([1500.0,2100.0])
            self.obs_y = np.array([700.0, 800.0])
            self.obs_R = np.array([300.0, 300.0])
            self.obs_u = np.array([0.0, 0.0])
            self.obs_v = np.array([0.0, 0.0])

        elif num == 5:
            self.obs_x = np.array([1000.0])
            self.obs_y = np.array([400.0])
            self.obs_R = np.array([300.0])
            self.obs_u = np.array([15.0])
            self.obs_v = np.array([0.0])

        elif num == 6:
            self.obs_x = np.array([1500.0,2500.0,3500.0,4500.0,5500.0,6500.0,7500.0, 8500.0])
            self.obs_y = np.array([500.0, 800.0, 400.0, 600.0, 800.0, 400.0, 500.0, 700.0])
            self.obs_R = np.array([200.0,200.0, 300.0, 200.0,200.0,200.0,200.0,200.0 ])
            self.obs_u = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
            self.obs_v = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        
        elif num == 7:
            self.obs_x = np.array([2000.0])
            self.obs_y = np.array([-100.0])
            self.obs_R = np.array([300.0])
            self.obs_u = np.array([0.0])
            self.obs_v = np.array([0.0])

        elif num == 8:
            self.obs_x = np.array([1500.0, 1550.0, 1450.0, 1420.0, 1400.0, 1350.0])
            self.obs_y = np.array([1500.0, 1500.0, 1450.0, 1450.0, 1430.0, 1450.0])
            self.obs_R = np.array([100.0, 100.0, 100.0, 100.0, 100.0, 100.0])
            self.obs_u = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.obs_v = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            
        self.num_obs = self.obs_x.size
    
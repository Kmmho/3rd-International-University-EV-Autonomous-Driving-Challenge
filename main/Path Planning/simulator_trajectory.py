import scipy.optimize
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from obstacle import obstacle_scenario
from mapping import Mapping

class Lambda():
    ## Cost Function Parameter ##
    ## P : Position Parameter -> The higher, the faster go back to global path. However, slope is more steep
    ## S : Speed Parameter -> The higher, the more tracking reference speed
    ## L : Vehicle Limited Parameter -> The higher, the more reflection of vehicle limitaion -> stable trajectory. However, trajectory slope is more gradual. (vice versa to P)
    ## T : Terminal Tracking Parameter -> There is no significant impact of these parameters at our trajectory.
    lambda_P = 30.0
    lambda_S = 9.0
    lambda_L = 10.0
    lambda_T = 1.0

    def get(self):
        return self.lambda_P, self.lambda_S, self.lambda_L, self.lambda_T

class Vehicle():
    V_min = 10
    V_max = 50
    yaw_min = np.deg2rad(-30)
    yaw_max = np.deg2rad(30)



class TrajectoryOptimization():
    ## Dynamics Obstacle의 고려까지 가능한 상황에서 추가적인 Contribution + 시뮬레이터에 달기위한 Syncronization ##
    ## 1. Kalman Filter를 통해 받아오는 속도로 MPC를 풀도록 수정 설계 -> u = V*cos(yaw), v = V*sin(yaw)
    ## 2. MPC의 time consuming issue를 방지하기 위해, 추가적인 알고리즘 사항 생각 -> 매 Step마다 푸는게 아닌, 목적지 Waypoint하나 정해놓고 충돌 판단 될 때마다, Local Planning
    ## 3. 시뮬레이터에 연동시키기 위해 추가 Constraints 고려 -> 중앙선 침범 X, 연석 침범 X -> 자신의 차선과 도로의 차선 상태를 알도록 하는 알고리즘 필요 할 듯

    ####################                   본선 미션 중, 충돌회피가 필요한 미션 구간들은 전부 직선구간이다.                ########################
    ######################## Time Parameterization 작업에서 goal point와 현재 position을 잇는 직선 경로를 써도 무관할 듯 ########################
    
    def __init__(self):
        self.coef_lambda = Lambda()

        self.num_local_waypoint = 31
        self.num_trav_waypoint = 4
        self.horizon_time = 5
        self.tau = np.linspace(0, 1, self.num_local_waypoint)
        self.t_array = self.horizon_time * self.tau
        self.t_diff = np.diff(self.t_array)
        self.FirstRun = False
        self.AddtionalRun = False
        self.mapping = Mapping()

        self.is_obstacle = False
        self.obstacle = obstacle_scenario()

        self.safe_distance = 10
        self.vehicle_con = Vehicle()

        self.past_V = []
        self.past_x = []
        self.past_y = []
        self.past_x_d = []
        self.past_y_d = []
        self.past_psi = []
        self.past_u = []
        self.past_v = []
        
        self.C_u = np.zeros(7)
        self.C_v = np.zeros(7)

        self.R = np.array([(1-self.tau)**6, 6*self.tau*(1-self.tau)**5, 15*self.tau**2*(1-self.tau)**4,
                                        20*self.tau**3*(1-self.tau)**3, 15*self.tau**4*(1-self.tau)**2, 6*self.tau**5*(1-self.tau), self.tau**6])
        
        self.R_dot = np.array([-6*(1-self.tau)**5, -6*(6*self.tau-1)*(1-self.tau)**4,
                                            -30*self.tau*(3*self.tau-1)*(1-self.tau)**3, -60*self.tau**2*(2*self.tau-1)*(1-self.tau)**2,
                                             30*self.tau**3*(3*self.tau**2 - 5*self.tau + 2), 6*(5-6*self.tau)*self.tau**4,
                                             6*self.tau**5])

        self.R_dot_dot = np.array([ 30*(1-self.tau)**4, 
                                                -60*(self.tau-1)**3*(3*self.tau-1),
                                                 30*(15*self.tau**2 - 10*self.tau+1)*(1-self.tau)**2,
                                                -120*self.tau*(5*self.tau**3 - 10*self.tau**2 + 6*self.tau - 1),
                                                 30*self.tau**2*(15*self.tau**2-20*self.tau+6),
                                                -60*self.tau**3*(3*self.tau-2),
                                                 30*self.tau**4])

        self.R_int = np.zeros([7, self.num_local_waypoint])
        self.R_int[0,0] = self.t_array[0] * self.horizon_time/7
        self.R_int[1,0] = -(6*(self.t_array[0]**7/7 - (5*self.t_array[0]**6*self.horizon_time)/6 + 2*self.t_array[0]**5*self.horizon_time**2 - (5*self.t_array[0]**4*self.horizon_time**3)/2 + (5*self.t_array[0]**3*self.horizon_time**4)/3 - (self.t_array[0]**2*self.horizon_time**5)/2))/self.horizon_time**6
        self.R_int[2,0] = (15*(self.t_array[0]**7/7 - (2*self.t_array[0]**6*self.horizon_time)/3 + (6*self.t_array[0]**5*self.horizon_time**2)/5 - self.t_array[0]**4*self.horizon_time**3 + (self.t_array[0]**3*self.horizon_time**4)/3))/self.horizon_time**6
        self.R_int[3,0] = (-20*(self.t_array[0]**7/7 - (self.t_array[0]**6*self.horizon_time)/2 + (3*self.t_array[0]**5*self.horizon_time**2)/5 - (self.t_array[0]**4*self.horizon_time**3)/4))/self.horizon_time**6
        self.R_int[4,0] = (15*self.t_array[0]**7)/(7*self.horizon_time**6) - (5*self.t_array[0]**6)/self.horizon_time**5 + (3*self.t_array[0]**5)/self.horizon_time**4
        self.R_int[5,0] = (-6*(self.t_array[0]**7/7 - (self.t_array[0]**6*self.horizon_time)/6))/self.horizon_time**6
        self.R_int[6,0] = (self.t_array[0]**7)/(7*self.horizon_time**6)


        for i in range(1,self.num_local_waypoint):
            self.R_int[0, i] =  (self.t_array[i]/self.horizon_time-1)**7 * self.horizon_time/7 - (self.t_array[0]/self.horizon_time -1)**7 * self.horizon_time/7

            self.R_int[1, i] = -(6*(self.t_array[i]**7/7 - (5*self.t_array[i]**6*self.horizon_time)/6 + 2*self.t_array[i]**5*self.horizon_time**2 - (5*self.t_array[i]**4*self.horizon_time**3)/2 + (5*self.t_array[i]**3*self.horizon_time**4)/3 - (self.t_array[i]**2*self.horizon_time**5)/2))/self.horizon_time**6

            self.R_int[2, i] = (15*(self.t_array[i]**7/7 - (2*self.t_array[i]**6*self.horizon_time)/3 + (6*self.t_array[i]**5*self.horizon_time**2)/5 - self.t_array[i]**4*self.horizon_time**3 + (self.t_array[i]**3*self.horizon_time**4)/3))/self.horizon_time**6

            self.R_int[3, i] = (-20*(self.t_array[i]**7/7 - (self.t_array[i]**6*self.horizon_time)/2 + (3*self.t_array[i]**5*self.horizon_time**2)/5 - (self.t_array[i]**4*self.horizon_time**3)/4))/self.horizon_time**6

            self.R_int[4, i] = (15*self.t_array[i]**7)/(7*self.horizon_time**6) - (5*self.t_array[i]**6)/self.horizon_time**5 + (3*self.t_array[i]**5)/self.horizon_time**4

            self.R_int[5, i] = (-6*(self.t_array[i]**7/7 - (self.t_array[i]**6*self.horizon_time)/6)) / self.horizon_time**6

            self.R_int[6, i] = (self.t_array[i]**7) / (7*self.horizon_time**6)
            


    def normalize(self, theta):
        return theta + 2*np.pi if (-np.pi < theta < -np.pi/2) else theta
        


    def set_local_goal(self, global_path, x, y, closest_index):
        for i in range(len(global_path)-closest_index-1):
            dist = np.hypot(global_path[closest_index + i, 0] - x, global_path[closest_index + i, 1] - y)
            next_dist = np.hypot(global_path[closest_index + i+1, 0] - x, global_path[closest_index + i+1, 1] - y)
            if dist < next_dist:
                return closest_index + i
            elif closest_index + i == len(global_path):
                return len(global_path)-1
            

    def set_global(self, x, y, yaw, u, v, ref_vel, global_path, closest_index, obs_x, obs_y):
        vel = ref_vel
        self.yaw_g = yaw * np.ones(self.num_local_waypoint)
        self.V_g = vel * np.ones(self.num_local_waypoint)
        self.u_g = self.V_g * np.cos(self.yaw_g[0])
        self.v_g = self.V_g * np.sin(self.yaw_g[0])
        self.x_g = np.arange(x, self.horizon_time * self.u_g[0] + self.u_g[0] * self.t_diff[0], self.t_diff[0] * self.u_g[0])
        self.y_g = np.arange(y, self.horizon_time * self.v_g[0] + self.v_g[0] * self.t_diff[0], self.t_diff[0] * self.v_g[0])
        self.goal_idx = self.set_local_goal(global_path, self.x_g[-1], self.y_g[-1], closest_index)
        self.x_g[-1] = global_path[self.goal_idx, 0]
        self.y_g[-1] = global_path[self.goal_idx, 1]
        self.FirstRun = True
        self.mapping.transform_points(x, y, yaw)
        self.obstacle.set_initial_obstacle(self.mapping.points_UTM)

        ###################### Initial Setting #########################

        self.V_d = self.V_g[0:self.num_local_waypoint]
        self.x_d = self.x_g[0:self.num_local_waypoint]
        self.y_d = self.x_g[0:self.num_local_waypoint]
        self.yaw_d = self.yaw_g[0:self.num_local_waypoint]
        self.u_d = self.u_g[0:self.num_local_waypoint]
        self.v_d = self.v_g[0:self.num_local_waypoint]

        self.u_0 = self.u_d[0]
        self.v_0 = self.v_d[0]

        self.u_0_dot = 0
        self.v_0_dot = 0

        self.u_0_dot_dot = 0
        self.v_0_dot_dot = 0

        ## Planning이 발동되는 시점에서의 위치 값 ##
        self.x_0 = x
        self.y_0 = y
        self.yaw = yaw
        
        ## Planning이 발동되는 시점에서의 Reference 속도 값 ##
        self.C_u[3::] = ref_vel * np.cos(yaw)
        self.C_v[3::] = ref_vel * np.sin(yaw)

        ##################################################################



    def update(self, x, y, psi, V, u, v, u_dot, v_dot, u_dot_dot, v_dot_dot, ref_vel, global_path, closest_index):
        # Update States for next Step
        self.u_0 = u[self.num_trav_waypoint-1]
        self.v_0 = v[self.num_trav_waypoint-1]

        self.u_0_dot = u_dot[self.num_trav_waypoint-1]
        self.v_0_dot = v_dot[self.num_trav_waypoint-1]

        self.u_0_dot_dot = u_dot_dot[self.num_trav_waypoint-1]
        self.v_0_dot_dot = v_dot_dot[self.num_trav_waypoint-1]

        self.x_0 = x[self.num_trav_waypoint-1]
        self.y_0 = y[self.num_trav_waypoint-1]

        self.yaw = psi[self.num_trav_waypoint-1]

        # # Add states
        # self.past_V.append(V[0:self.num_trav_waypoint])
        # self.past_x.append(x[0:self.num_trav_waypoint])
        # self.past_y.append(y[0:self.num_trav_waypoint])
        # self.past_u.append(u[0:self.num_trav_waypoint])            
        # self.past_v.append(v[0:self.num_trav_waypoint])
        # self.past_x_d.append(self.x_d[0:self.num_trav_waypoint])
        # self.past_y_d.append(self.y_d[0:self.num_trav_waypoint])
        # self.past_psi.append(self.yaw_d[0:self.num_trav_waypoint])

        # Update Global Trajectory
        self.u_g = ref_vel * np.ones(self.num_local_waypoint) * np.cos(self.yaw_g[0])
        self.v_g = ref_vel * np.ones(self.num_local_waypoint) * np.sin(self.yaw_g[0])
        self.x_g = np.linspace(self.x_g[self.num_trav_waypoint-1], self.x_g[self.num_trav_waypoint-1] + self.horizon_time*self.u_g[0], self.num_local_waypoint)
        self.y_g = np.linspace(self.y_g[self.num_trav_waypoint-1], self.y_g[self.num_trav_waypoint-1] + self.horizon_time*self.v_g[0], self.num_local_waypoint)
        self.goal_idx = self.set_local_goal(global_path, self.x_g[-1], self.y_g[-1], closest_index)
        self.x_g[-1] = global_path[self.goal_idx, 0]
        self.y_g[-1] = global_path[self.goal_idx, 1]
        
            
        # Update Demanded Trajectory
        self.u_d = self.u_g
        self.v_d = self.v_g
        self.x_d = self.x_g
        self.y_d = self.y_g

        # Update Obstacle Position
        # 시뮬레이터 상에서는 동적장애물에 대한 충돌회피는 없을 것으로 예상되어.. 이 부분은 필요 없을 듯 
        # self.obstacle.obs_x += self.obstacle.obs_u*self.t_array[self.num_trav_waypoint-1]
        # self.obstacle.obs_y += self.obstacle.obs_v*self.t_array[self.num_trav_waypoint-1]



    

    def cost_function(self, x):
        Cp, Cs, Cl, Ct = self.coef_lambda.get()
        C_u_ = np.hstack([self.C_u[0:3], x[0:4]])
        C_v_ = np.hstack([self.C_v[0:3], x[4:9]])

        [x_a, y_a, u_a, v_a, u_a_dot, v_a_dot, u_a_dot_dot, v_a_dot_dot,
         V_a, V_a_dot, psi_a, psi_a_dot                                ] = self.calculate_trajectory_state(C_u_, C_v_)
        
    
        position_cost = sum((self.x_d - x_a)**2 + (self.y_d - y_a)**2) /  (max(self.obstacle.obs_R) + self.safe_distance)**2
        speed_cost = sum((self.u_d - u_a)**2 + (self.v_d - v_a)**2) / (self.num_local_waypoint*((np.mean(self.u_d))**2 + np.mean(self.v_d)**2))

        vehicle_constraints_penalty = (np.maximum(0, self.vehicle_con.V_min - V_a)**2 + 
                                       np.maximum(0, V_a - self.vehicle_con.V_max)**2 +
                                       np.maximum(0, self.vehicle_con.yaw_min - psi_a)**2 +
                                       np.maximum(0, psi_a - self.vehicle_con.yaw_max)**2)

        vehicle_constraints_penalty = sum(vehicle_constraints_penalty)

        terminal_tracking_cost = ((self.yaw_d[self.num_local_waypoint-1] - psi_a[self.num_local_waypoint-1]) / np.pi)**2


        position_cost = min(position_cost, 1e10)
        speed_cost = min(speed_cost, 1e10)
        vehicle_constraints_penalty = min(vehicle_constraints_penalty, 1e10)
        terminal_tracking_cost = min(terminal_tracking_cost, 1e10)

        return Cp*position_cost + Cs*speed_cost + Cl*vehicle_constraints_penalty + Ct*terminal_tracking_cost


    def ineq_trajectory_constraints(self, x):

        ceq = np.zeros(self.num_local_waypoint * self.obstacle.num_obs)
        C_u_ = np.hstack([self.C_u[0:3], x[0:4]])
        C_v_ = np.hstack([self.C_v[0:3], x[4:9]])

        x = self.x_0 + np.transpose(C_u_)@self.R_int
        y = self.y_0 + np.transpose(C_v_)@self.R_int

        # 예측한 시간동안 차량이 움직이는 y값 -> 지금 현재 차량의 y값을 넘지 않도록 제약조건


        if not self.is_obstacle:
            c = np.zeros(self.num_local_waypoint * self.obstacle.num_obs)
        else:
            c = []
            for i in range(self.obstacle.num_obs):
                x_cur_obs = self.obstacle.obs_x[i] + self.obstacle.obs_u[i]*self.t_array
                y_cur_obs = self.obstacle.obs_y[i] + self.obstacle.obs_v[i]*self.t_array
                # x_cur_obs = self.obstacle.obs_x[i]
                # y_cur_obs = self.obstacle.obs_y[i]

                distance = np.sqrt((x_cur_obs - x)**2 + (y_cur_obs - y)**2)

                c[0 + self.num_local_waypoint*(i) : self.num_local_waypoint*(i+1)+1] = distance - (self.obstacle.obs_R[i] + self.safe_distance)
            
            c = np.array(c)


        return c



    def calculate_trajectory_state(self, C_u, C_v):
        x = self.x_0 + np.transpose(C_u)@self.R_int
        y = self.y_0 + np.transpose(C_v)@self.R_int

        u = np.transpose(C_u)@self.R
        v = np.transpose(C_v)@self.R

        u_dot = np.transpose(C_u)@self.R_dot / self.horizon_time
        v_dot = np.transpose(C_v)@self.R_dot / self.horizon_time

        u_dot_dot = np.transpose(C_u)@self.R_dot_dot / self.horizon_time**2
        v_dot_dot = np.transpose(C_v)@self.R_dot_dot / self.horizon_time**2

        V = np.sqrt(u**2 + v**2)
        V_dot = np.sqrt(u_dot**2 + v_dot**2)

        psi = np.array([self.normalize(math.atan2(v[k], u[k]) for k in range(len(v)))])
        psi_dot = np.diff(psi) / self.t_diff

        return x, y, u, v, u_dot, v_dot, u_dot_dot, v_dot_dot, V, V_dot, psi, psi_dot



    def optimize_main(self, x, y, yaw, u, v, ref_vel, global_path, closest_index):
        if not self.FirstRun:
            self.set_global(x ,y , yaw, u, v, ref_vel, global_path, closest_index)
        else:
            self.mapping.transform_points(x, y, yaw)
            self.obstacle.set_additory_obstacle(self.mapping.points_UTM)

        ## Start to solve Optimization Problem
        
        self.C_u[0] = self.u_0
        self.C_v[0] = self.v_0


        self.C_u[1] = self.u_0_dot * self.horizon_time/6 + self.C_u[0]
        self.C_v[1] = self.v_0_dot * self.horizon_time/6 + self.C_v[0]

            
        self.C_u[2] = self.u_0_dot_dot * self.horizon_time**2/30 - self.C_u[0] + 2*self.C_u[1]
        self.C_v[2] = self.v_0_dot_dot * self.horizon_time**2/30 - self.C_v[0] + 2*self.C_v[1]
        

        initial_condition = np.hstack([self.C_u[3::], self.C_v[3::]])

        result = scipy.optimize.minimize(self.cost_function, initial_condition, method='SLSQP', 
                                                                constraints={'type' : 'ineq', 'fun' : self.ineq_trajectory_constraints})
        optimalWaypoints = result.x


        self.C_u = np.hstack([self.C_u[0:3], optimalWaypoints[0:4]])
        self.C_v = np.hstack([self.C_v[0:3], optimalWaypoints[4:9]])

        # 계산된 계수들로 만들어진 새로운 Trajectory를 따라가는 좌표값, 속도값 등등.. 계산
        [x, y, u, v, u_dot, v_dot, u_dot_dot, v_dot_dot, 
         V, V_dot, psi, psi_dot                         ] = self.calculate_trajectory_state(self.C_u, self.C_v)
            


        ### Update Global Path ################################
            
        self.update(x, y, psi, u, v, V, u_dot, v_dot, u_dot_dot, v_dot_dot, ref_vel, global_path, closest_index)

        #######################################################




if __name__ == "__main__":
    a = TrajectoryOptimization()
    a.optimize_main()
        

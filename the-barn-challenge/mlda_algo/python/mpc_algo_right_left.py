import casadi as ca
import rospy
import numpy as np
import time

class NMPC:
    def __init__(self, freq = 20, N = 20):
        self.f = freq # Controller frequency [Hz]
        self.h = 1/self.f
        # self.rate = rospy.Rate(self.f)
        
        self.L = 0.37558 # Distance between 2 wheels
        # self.L = rospy.get_param("/mlda/L")
        # Using rosparam
        # For each wheels
        self.v_max = 1# Max velocity [m/s]
        self.v_min = -0.5# Min velocity [m/s]
        self.v_ref = 0.5 # Reference velocity [m/s]


        self.a_max = 1 # Max acceleration [m/s^2]

        self.w_max = 1 # Max angular vel [rad/s]
        self.w_min = -1 # Max angular vel [rad/s]
        
        self.weight_velocity = 1
        self.weight_position_error = 20
        self.weight_cross_track_error = 0
        self.weight_theta_error = 0
        self.weight_acceleration = 0
        self.weight_inital_theta_error = 0
        self.weight_obs = 1

        self.N = N
        
        
        self.opt_states = None
        # Obstacle avoidance
        pass
    
    def setup(self, rate):
        self.h = 1/rate
        # --- State and control variables ---
        # Variables
        # X = [x0, y0, theta0, vr0, vl0, ar0, al0, (...), xN, yN, thetaN, vrN, vlN, arN, alN]
        self.n = 7 # n is "Degree of freedom" in 1 instance. # N is the number of time step
        
        self.X = ca.MX.sym('X',self.N*self.n)
        
        # Bounds on variables
        lbx = [-np.inf,-np.inf,-np.inf,-self.v_max,-self.v_max,-self.a_max,-self.a_max]*self.N
        self.lbx = np.array(lbx)
        ubx = [np.inf,np.inf,np.inf,self.v_max,self.v_max,self.a_max,self.a_max]*self.N
        self.ubx = np.array(ubx)


        # --- Constraints ---
        # Vehicle dynamics constraints
        # Position (x, y, theta)
        gx = self.X[0::self.n][1:] - self.X[0::self.n][:-1] - 0.5*self.h*(((self.X[3::self.n][1:] + self.X[4::self.n][1:])/2)*np.cos(self.X[2::self.n][1:]) + ((self.X[3::self.n][:-1] + self.X[4::self.n][:-1])/2)*np.cos(self.X[2::self.n][:-1]))
        gy = self.X[1::self.n][1:] - self.X[1::self.n][:-1] - 0.5*self.h*(((self.X[3::self.n][1:] + self.X[4::self.n][1:])/2)*np.sin(self.X[2::self.n][1:]) + ((self.X[3::self.n][:-1] + self.X[4::self.n][:-1])/2)*np.sin(self.X[2::self.n][:-1]))
        gtheta = self.X[2::self.n][1:] - self.X[2::self.n][:-1] - 0.5*self.h*(((self.X[3::self.n][1:] - self.X[4::self.n][1:])/self.L) + ((self.X[3::self.n][:-1] - self.X[4::self.n][:-1])/self.L))
        # Velocity (v_r, v_l)
        gv_r = self.X[3::self.n][1:] - self.X[3::self.n][:-1] - 0.5*self.h*(self.X[5::self.n][1:] + self.X[5::self.n][:-1])
        gv_l = self.X[4::self.n][1:] - self.X[4::self.n][:-1] - 0.5*self.h*(self.X[6::self.n][1:] + self.X[6::self.n][:-1])
        # Positive linear velocity
        gv_min = (self.X[3::self.n][1:] + self.X[4::self.n][1:]) - self.v_min*2
        gv_max = self.v_max*2  - (self.X[3::self.n][1:] + self.X[4::self.n][1:])

        # Minimum angular velocity
        gw_min = ((self.X[3::self.n][1:] - self.X[4::self.n][1:])/self.L) - self.w_min
        # Maximum angular velocity
        gw_max = self.w_max - ((self.X[3::self.n][1:] - self.X[4::self.n][1:])/self.L)


        ### Each of the term above has N-1 columns   
        self.g = ca.vertcat(gx, gy, gtheta, gv_r, gv_l, gv_min, gv_max, gw_min, gw_max)
        


        ### This is not dynamic contraints, because they only relate the existing optimization variables
    
    def solve(self, x_ref, y_ref, theta_ref, X0):
        start_time = time.time()

        
        # === Set constraints bound
        per_step_constraints = 9
        init_value_constraints = 5
        final_value_contraints = 3

        self.lbg = np.zeros((self.N - 1) * per_step_constraints + init_value_constraints + final_value_contraints)
        self.ubg = np.zeros((self.N - 1) * per_step_constraints + init_value_constraints + final_value_contraints)
        # Set inequality bound
        self.ubg[(-4*(self.N - 1)-init_value_constraints-final_value_contraints):(-init_value_constraints -final_value_contraints)] = np.inf # Velocity positive
        

        ## All constraints g
        self.g = self.g[:(self.N - 1) * per_step_constraints]  # Since we are recycling the same instance
        # We have to truncate the previous optimization run
        
        # Init value constraints expression
        for i in range(init_value_constraints):
            g0 = self.X[i::self.n][0] - X0[i]
            self.g = ca.vertcat(self.g, g0)
            
        # Final value constraints expression
        # gmx = self.X[0::self.n][int((self.N-1)/2)] - x_ref[int((self.N-1)/2)]
        # gmy = self.X[1::self.n][int((self.N-1)/2)] - y_ref[int((self.N-1)/2)]
        # gmtheta = self.X[2::self.n][int((self.N-1)/2)] - theta_ref[int((self.N-1)/2)]
            

        def diff_angle(a1,a2):
            diff = max(a1, a2) - min(a1, a2)
            if diff > np.pi:
                diff = 2*np.pi - diff
            return diff
        ref = theta_ref[self.N]
        idx = 0 
        for i in range(self.N, min(len(x_ref), len(theta_ref))):
            # print(i, " Diff angle: ", diff_angle(theta_ref[i],ref))
            idx = i
            if diff_angle(theta_ref[i],ref) > np.pi/25:
                break

        self.v_ref = 1
        print("IDX ", idx, len(theta_ref), len(x_ref), len(y_ref), self.v_ref)

        # idx = self.N - 1
        gfx = self.X[0::self.n][self.N-1] - x_ref[idx]
        gfy = self.X[1::self.n][self.N-1] - y_ref[idx]
        gftheta = self.X[2::self.n][self.N-1] - theta_ref[idx]
        self.g = ca.vertcat(self.g, gfx, gfy, gftheta)


        print("Constraints: ", self.g.shape[0])
        # --- Cost function --- 

        self.weight_velocity = 10
        # self.weight_minimize_angular = 0
        self.weight_position_error = 1
        # self.weight_cross_track_error = 2
        # self.weight_theta_error = 2
        # self.weight_inital_theta_error = 10


        J = 0
        initial_theta_error = (self.X[2::self.n][1] - theta_ref[1])**2
        J += self.weight_inital_theta_error*initial_theta_error
        for i in range(self.N):
            # Position Error cost
            position_error_cost = (self.X[0::self.n][i] - x_ref[i])**2 + (self.X[1::self.n][i] - y_ref[i])**2
            
            # Theta Error cost 
            theta_error_cost = (self.X[2::self.n][i] - theta_ref[i])**2

            # Reference velocity cost
            reference_velocity_cost = (self.X[3::self.n][i] - self.v_ref)**2 + (self.X[4::self.n][i] - self.v_ref)**2
            # reference_velocity_cost = 0

            # Minimize angular velocity
            angular_velocity_cost = -(self.X[3::self.n][i] - self.X[4::self.n][i])**2

            # Cross-track Error cost
            cross_track_error_cost = -(x_ref[i] - self.X[0::self.n][i])*(np.sin(theta_ref[i])) + (y_ref[i] - self.X[1::self.n][i])*(np.cos(theta_ref[i]))

            # Successive control cost
            # if i != (self.N-1):
            #     successive_error = ((self.X[5::self.n][i+1]-self.X[5::self.n][i])*(self.X[5::self.n][i+1]-self.X[5::self.n][i]))+((self.X[6::self.n][i+1]-self.X[6::self.n][i])*(self.X[6::self.n][i+1]-self.X[6::self.n][i]))

            # Cost function calculation
            J += (self.weight_position_error*position_error_cost + self.weight_velocity*reference_velocity_cost + self.weight_theta_error*theta_error_cost + self.weight_cross_track_error*cross_track_error_cost)
        
        
        # === Initial guess
        if self.opt_states == None:
            init_guess = 0
        else:
            # print("Used old values")
            init_guess = ca.vertcat(self.opt_states[7:], self.opt_states[-7:])
            # init_guess = 0
        
        # === Solution
        options = {'ipopt.print_level':1, 'print_time': 0, 'expand': 1} # Dictionary of the options
        # options = {}
        
        
        problem = {'f': J , 'x': self.X, 'g': self.g} # Dictionary of the problem
        solver = ca.nlpsol('solver', 'ipopt', problem,options) #ipopt: interior point method
        
        solution = solver(x0 = init_guess, lbx = self.lbx, ubx = self.ubx, lbg = self.lbg, ubg = self.ubg)
        
        # === Optimal control 
        vr_opt = solution['x'][3::self.n][2]
        vl_opt = solution['x'][4::self.n][2]
        


        v_opt = (vr_opt + vl_opt)/2
        w_opt = (vr_opt - vl_opt)/self.L
        
        vr_opt_list = solution['x'][3::self.n]
        vl_opt_list = solution['x'][4::self.n]
        v_opt_list = (vr_opt_list + vl_opt_list)/2
        w_opt_list = (vr_opt_list - vl_opt_list)/self.L

        # Intial guess for next steps
        self.opt_states = solution['x']
        solve_time = round(time.time() - start_time,5)
        
        
        
        print("============Debugging=======")
        print("V: ", v_opt, " W: ", w_opt)
        # print("Initial state: ", X0[0], " ", X0[1])
        # print("solution at t = 0: ", self.opt_states[0], " ", self.opt_states[1])
        
        # print("V: ", v_opt_list, " W: ", w_opt_list)
        print("Cost: ", solution['f'])
        print("Solve time: ", solve_time)
        
        # print(solution)
            
            
        return v_opt, w_opt
        

    def solve_obs(self, x_ref, y_ref, theta_ref, obs_x, obs_y, X0):
            start_time = time.time()

            obs_num = len(obs_x)
            obs_constraints = obs_num*(self.N-1)

            obs_dist = []
            for i in range(obs_num):
                obs_dist.append(np.sqrt((obs_x[i] - x_ref[0])**2 + (obs_y[i] - y_ref[0])**2))
            obs_dist = np.array(obs_dist)

            careful = False
            safe_dist = 0.4
            if np.count_nonzero(obs_dist < safe_dist) > 0: #m
                careful = True
                final_value_contraints = 0
                print("---------------------===========CAREFUL")
                self.weight_velocity = 5
                self.weight_position_error = 5
                self.weight_theta_error = 0
                self.v_ref = 0.1
            else:

                self.weight_velocity = 1
                self.weight_position_error = 10
                # self.weight_theta_error = 10
                self.weight_inital_theta_error = 0
                final_value_contraints = 3
            # === Set constraints bound
            per_step_constraints = 9
            init_value_constraints = 5 
            # final_value_contraints = 0



            self.lbg = np.zeros((self.N - 1) * per_step_constraints + init_value_constraints + final_value_contraints + obs_constraints)
            self.ubg = np.zeros((self.N - 1) * per_step_constraints + init_value_constraints + final_value_contraints + obs_constraints)
            # Set inequality bound
            self.ubg[(-4*(self.N - 1)-init_value_constraints-final_value_contraints- obs_constraints):(-init_value_constraints -final_value_contraints- obs_constraints)] = np.inf # Velocity positive
            
            self.ubg[(-obs_constraints):] = np.inf # Obstacle avoidance

            ## All constraints g
            self.g = self.g[:(self.N - 1) * per_step_constraints]  # Since we are recycling the same instance
            # We have to truncate the previous optimization run
            
            # Init value constraints expression
            for i in range(init_value_constraints):
                g0 = self.X[i::self.n][0] - X0[i]
                self.g = ca.vertcat(self.g, g0)
                
            # Final value constraints expression
            # gmx = self.X[0::self.n][int((self.N-1)/2)] - x_ref[int((self.N-1)/2)]
            # gmy = self.X[1::self.n][int((self.N-1)/2)] - y_ref[int((self.N-1)/2)]
            # gmtheta = self.X[2::self.n][int((self.N-1)/2)] - theta_ref[int((self.N-1)/2)]
            


            if not careful:
                offset = self.N-1
                gfx = self.X[0::self.n][offset] - x_ref[offset]
                gfy = self.X[1::self.n][offset] - y_ref[offset]
                gftheta = self.X[2::self.n][offset] - theta_ref[offset]
                self.g = ca.vertcat(self.g, gfx, gfy, gftheta)

            safe = 0.3
            for i in range(obs_num):
                gobs = (obs_x[i] - self.X[0::self.n][1:])**2 + (obs_y[i] - self.X[1::self.n][1:])**2 - safe**2
                self.g = ca.vertcat(self.g, gobs)

            # print("Constraints: ", self.g.shape)
            # --- Cost function --- 
            

            J = 0

            initial_theta_error = (self.X[2::self.n][1] - theta_ref[1])**2
            J += self.weight_inital_theta_error*initial_theta_error

            for i in range(self.N):
                # Position Error cost
                position_error_cost = (self.X[0::self.n][i] - x_ref[i])**2 + (self.X[1::self.n][i] - y_ref[i])**2
                
                # Theta Error cost 
                theta_error_cost = (self.X[2::self.n][i] - theta_ref[i])**2

                # Reference velocity cost
                reference_velocity_cost = (self.X[3::self.n][i] - self.v_ref)**2 + (self.X[4::self.n][i] - self.v_ref)**2
                # reference_velocity_cost = 0


                # Cross-track Error cost
                cross_track_error_cost = -(x_ref[i] - self.X[0::self.n][i])*(np.sin(theta_ref[i])) + (y_ref[i] - self.X[1::self.n][i])*(np.cos(theta_ref[i]))

                # Successive control cost
                # if i != (self.N-1):
                #     successive_error = ((self.X[5::self.n][i+1]-self.X[5::self.n][i])*(self.X[5::self.n][i+1]-self.X[5::self.n][i]))+((self.X[6::self.n][i+1]-self.X[6::self.n][i])*(self.X[6::self.n][i+1]-self.X[6::self.n][i]))

                # Cost function calculation
                J += (self.weight_position_error*position_error_cost + self.weight_velocity*reference_velocity_cost + self.weight_theta_error*theta_error_cost + self.weight_cross_track_error*cross_track_error_cost)
            
            
            # === Initial guess
            if self.opt_states == None:
                init_guess = 0
            else:
                # print("Used old values")
                init_guess = ca.vertcat(self.opt_states[7:], self.opt_states[-7:])
                # init_guess = 0
            
            # === Solution
            options = {'ipopt.print_level':1, 'print_time': 0, 'expand': 1} # Dictionary of the options
            # options = {}
            
            
            problem = {'f': J , 'x': self.X, 'g': self.g} # Dictionary of the problem
            solver = ca.nlpsol('solver', 'ipopt', problem,options) #ipopt: interior point method
            
            solution = solver(x0 = init_guess, lbx = self.lbx, ubx = self.ubx, lbg = self.lbg, ubg = self.ubg)
            
            # === Optimal control 
            vr_opt = solution['x'][3::self.n][2]
            vl_opt = solution['x'][4::self.n][2]
            


            v_opt = (vr_opt + vl_opt)/2
            w_opt = (vr_opt - vl_opt)/self.L
            
            vr_opt_list = solution['x'][3::self.n]
            vl_opt_list = solution['x'][4::self.n]
            v_opt_list = (vr_opt_list + vl_opt_list)/2
            w_opt_list = (vr_opt_list - vl_opt_list)/self.L

            # Intial guess for next steps
            self.opt_states = solution['x']
            solve_time = round(time.time() - start_time,5)
            
            
            
            print("============Debugging=======")
            print("V: ", v_opt, " W: ", w_opt)
            # print("Initial state: ", X0[0], " ", X0[1])
            # print("solution at t = 0: ", self.opt_states[0], " ", self.opt_states[1])
            
            # print("V: ", v_opt_list, " W: ", w_opt_list)
            print("Cost: ", solution['f'])
            print("Solve time: ", solve_time)
            
            # print(solution)
                
                
            return v_opt, w_opt
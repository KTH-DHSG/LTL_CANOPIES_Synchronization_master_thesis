import casadi as ca
import casadi.tools as ca_tools

import numpy as np

class MPC_Turtlebot():
    def __init__(self, x_0, x_t, obstacles):
        # sampling time [s]
        self.T = 0.2
        # prediction horizon
        self.N = 50

        # Turtlebot dimention and specifications
        self.rob_diam = 0.25 # [m] #actual max dimension is 0.178
        self.v_max = 0.15 # [m/s]
        self.omega_max = 1.2 # [rad/s]
        
        # Arena limits 
        self.arena_x_max = 1.877 - self.rob_diam/2.
        self.arena_x_min = -2.290 - self.rob_diam/2.
        self.arena_y_max = 2.300 - self.rob_diam/2.
        self.arena_y_min = -2.869 - self.rob_diam/2.

        # Casadi variables for states
        self.states = ca_tools.struct_symSX([
            (
                ca_tools.entry('x'),
                ca_tools.entry('y'),
                ca_tools.entry('theta')
            )
        ])
        # initialize variables
        self.x, self.y, self.theta = self.states[...]
        # number of states
        n_states = self.states.size
        
        # Casadi variables for controls
        self.controls  = ca_tools.struct_symSX([
            (
                ca_tools.entry('v'),
                ca_tools.entry('omega')
            )
        ])
        #initialize variables
        self.v, self.omega = self.controls[...]


        
        ## Unicycle model for the turtlebot
        self.model = ca_tools.struct_SX(self.states)
        self.model['x'] = self.v*ca.cos(self.theta)
        self.model['y'] = self.v*ca.sin(self.theta)
        self.model['theta'] = self.omega
        
        ## Casadi function to return the states
        self.f = ca.Function('f', [self.states, self.controls], [self.model], ['input_state', 'control_input'], ['model'])
        
        ## for MPC multi shooting
        self.optimizing_target = ca_tools.struct_symSX([
            (
                ca_tools.entry('U', repeat=self.N, struct=self.controls),
                ca_tools.entry('X', repeat=self.N+1, struct=self.states)
            )
        ])
        
        self.U, self.X, = self.optimizing_target[...] # data are stored in list [], notice that ',' cannot be missed     

        self.current_parameters = ca_tools.struct_symSX([
            (
                # first n_states are the initial states and the next n_states are the reference states
                ca_tools.entry('P', shape=n_states+n_states),
                # matrix num obstacles x 3, each row is [x, y, radius]
                ca_tools.entry('OBS', shape=obstacles.shape)
            )
        ])
        self.P, self.OBS= self.current_parameters[...]
        
        # TODOD: tune the cost function weights
        # cost function weights for difference in state
        self.Q = np.array([[1.0, 0.0, 0.0],[0.0, 5.0, 0.0],[0.0, 0.0, .1]])
        # cost function weights for difference in control
        self.R = np.array([[0.5, 0.0], [0.0, 0.05]])
             
        # initialize the cost function
        self.obj = 0
        # constrains vector
        self.constr = []
        
        # lower and upper bounds for constrains
        self.lb_constr  = []
        self.ub_constr  = []
        
        # initialize nlp and solver
        self.nlp_prob = {}
        self.opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6, 'ipopt.sb':"yes"}
        self.solver = None
    
        # upper and lower bounds for the states and controls
        self.lb_state  = []
        self.ub_state  = []
        # add constraints to control and states
        for _ in range(self.N):
            # linear velocity
            self.lb_state.append(-self.v_max)
            self.ub_state.append(self.v_max)
            # angular velocity
            self.lb_state.append(-self.omega_max)            
            self.ub_state.append(self.omega_max)
            # x limits of the arena
            self.lb_state.append(self.arena_x_min)
            self.ub_state.append(self.arena_x_max)
            # y limits of the arena
            self.lb_state.append(self.arena_y_min)
            self.ub_state.append(self.arena_y_max)
            # no limits for theta            
            self.lb_state.append(-np.inf)            
            self.ub_state.append(np.inf)        
        # add constraints on state for the N+1 state
        # x limits of the arena
        self.lb_state.append(self.arena_x_min)
        self.ub_state.append(self.arena_x_max)
        # y limits of the arena
        self.lb_state.append(self.arena_y_min)
        self.ub_state.append(self.arena_y_max)
        # no limits for theta            
        self.lb_state.append(-np.inf)            
        self.ub_state.append(np.inf)
        
        # set up initial conditions for MPC
        # initial_state
        self.x_0 = np.array(x_0).reshape(-1, 1)
        # target state
        self.x_t = np.array(x_t).reshape(-1, 1)
        # initial control
        self.u_0 = np.array([0.0, 0.0]*self.N).reshape(-1, 2).T
        # initial feedforward value
        self.ff_value = np.array([0.0, 0.0, 0.0]*(self.N+1)).reshape(-1, 3).T        
        # initial obstacles
        self.obstacles = obstacles        
        # initial parameters for the solver
        self.c_p = self.current_parameters(0)
        self.init_control = self.optimizing_target(0)
        
        # tuning Parameter for the barrier function
        self.alpha = 1.5
        # adding constraints
        self.add_constraints()
        

    def add_constraints(self):
        # empty constrains vector
        self.constr = []
        
        # empty lower and upper bounds for constrains
        self.lb_constr  = []
        self.ub_constr  = []
        # initial condition constraints for multiple shooting
        self.constr.append(self.X[0]-self.P[:3]) 
        for i in range(self.N):
            # define the cost function
            self.obj = self.obj + ca.mtimes([(self.X[i]-self.P[3:]).T, self.Q, self.X[i]-self.P[3:]]) + ca.mtimes([self.U[i].T, self.R, self.U[i]])
            x_next_ = self.f(self.X[i], self.U[i])*self.T + self.X[i]
            # multiple shooting constraint
            self.constr.append(self.X[i+1] - x_next_)
        # add constraints to obstacle distance
        for i in range(self.N+1):
            for j in range(self.OBS.shape[0]):
                if i<self.N :
                    CBF_constr = self.get_CBF(self.X[i], self.U[i], self.OBS[j,:])
                else:
                    CBF_constr = self.get_CBF(self.X[i], ca.vertcat(0, 0), self.OBS[j,:])
                self.constr.append(CBF_constr)
                
        # add contraints bounds
        for _ in range(self.N+1):
            # add constraints for multiple shooting
            # no margins since each state is calculated from the previous state
            # x
            self.lb_constr.append(0.0)
            self.ub_constr.append(0.0)
            # y
            self.lb_constr.append(0.0)
            self.ub_constr.append(0.0)
            # theta
            self.lb_constr.append(0.0)
            self.ub_constr.append(0.0)
            
        # add constraints for collision avoidance
        for _ in range(self.N+1):
            for _ in range(self.OBS.shape[0]):
                # zero as lower bound since we don't want the robot to be inside the obstacle
                self.lb_constr.append(0.0)           
                # infinite as upper bound sincethe robot can be as far as it wants from the obstacle
                self.ub_constr.append(np.inf)
        
        # update the nlp problem
        self.nlp_prob = {'f': self.obj, 'x': self.optimizing_target, 'p':self.current_parameters, 'g':ca.vertcat(*self.constr)}
                    
        # build the new solver
        self.solver = ca.nlpsol('solver', 'ipopt', self.nlp_prob, self.opts_setting)
    
    def get_CBF(self, X, U, obs):
        
        # define the barrier function
        h=ca.sqrt((X[0]-obs[0])**2+(X[1]-obs[1])**2)-(self.rob_diam/2.+obs[2])
        # calcilating the x_dot
        x_dot= self.f(X, U)        
        # bulding the constrint for the barrier function
        constraint = ca.mtimes(ca.jacobian(h, X), x_dot)+self.alpha*h
        
        return constraint
    
    
    def get_next_control(self):
        # updating the parameters
        self.c_p['P'] = np.concatenate((self.x_0, self.x_t))
        self.c_p['OBS'] = self.obstacles
        # updating the initial control
        self.init_control['X', lambda x:ca.horzcat(*x)] = self.ff_value # [:, 0:N+1]
        self.init_control['U', lambda x:ca.horzcat(*x)] = self.u_0
        # solve the optimization problem
        res = self.solver(x0=self.init_control, p=self.c_p, lbg=self.lb_constr, lbx=self.lb_state, ubg=self.ub_constr, ubx=self.ub_state)                   
        # result of the optimization problem is in the series [u0, x0, u1, x1, ...]
        estimated_opt = res['x'].full()
        temp_estimated = estimated_opt[:-3].reshape(-1, 5) 
        # getting all the control inputs
        u_0 = temp_estimated[:, :2].T
        # update the control feedforward value
        self.u_0 = ca.horzcat(u_0[:, 1:], u_0[:, -1])
        # update the feedforward value
        ff_value = temp_estimated[:, 2:].T
        # add the last estimated result now is n_states * (N+1)      
        self.ff_value = np.concatenate((ff_value, estimated_opt[-3:].reshape(3, 1)), axis=1)   

        # return the first control input
        return u_0[:, 0], self.ff_value


    
    


    


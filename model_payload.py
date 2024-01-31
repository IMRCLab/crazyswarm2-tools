import numpy as np
from rowan.functions import _promote_vec, _validate_unit, exp, multiply
from rowan import from_matrix, to_matrix, to_euler, from_euler
import time


def skew(w):
    w = w.reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]]).reshape((3,3))


class ResidualsPayload():
    def __init__(self, data: dict[str, np.ndarray]) -> None:
        self.data = data
        self.delta = np.diff(self.data["timestamp"])
        self.delta_threshold = 0.2
        print(f"sampling time vector: {self.delta}") # in log 182: one delta is approx. 0.004s, one outlier is approx. 1.5s
        
        self.n = len(self.data["timestamp"])
        self.n_payload = 19
        self.n_input = 4
        
        # payload uav model parameters (Khaled's script: uav.py)
        self.mp = 0.005                                # mass of payload [kg]
        self.m  = 0.032                                # quadrotor mass [kg]
        self.mt = self.m + self.mp                     # total mass [kg]
        self.lc = 0.500                                # length of cable [m]
        self.g  = np.array([0, 0, -self.mt * 9.81])    # gravity vector [m/s^2]

        # uav model parameters (Khaled's script: uav.py) 
        self.I = 1e-6 * np.array([[16.571710, 0.830806, 0.718277],
                                  [0.830806, 16.655602, 1.800197],
                                  [0.718277, 1.800197, 29.261652]])
        self.invI = np.linalg.inv(self.I)

        # init required matrices
        self.stacked_states_data    = np.zeros((self.n, self.n_payload))
        self.stacked_inputs_data    = np.zeros((self.n, self.n_input))
        self.stacked_states_model   = np.zeros((self.n, self.n_payload))
        self.stacked_errors         = np.zeros((self.n, self.n_payload))
        
        print("===========================================")
        print("ResidualsPayload object created")
        print("Number of data points: ", self.n)
        print("===========================================")
        print("Constructing stacked states from data...")
        t1 = time.perf_counter()
        self.construct_stacked_states_data()
        t2 = time.perf_counter()
        print("Constructing stacked states from data took: ", t2 - t1, " s")
        print("===========================================")
        print("Constructing stacked inputs from data...")
        t1 = time.perf_counter()
        self.construct_stacked_inputs_data()
        t2 = time.perf_counter()
        print("Constructing stacked inputs from data took: ", t2 - t1, " s")
        print("===========================================")
        print("Constructing stacked states from model...")
        t1 = time.perf_counter()
        self.construct_stacked_states_model()
        t2 = time.perf_counter()
        print("Constructing stacked states from model took: ", t2 - t1, " s")
        print("===========================================")

        print("Computing errors...")
        t1 = time.perf_counter()
        self.compute_errors()
        t2 = time.perf_counter()
        print("Computing errors took: ", t2 - t1, " s")
        print("===========================================")
        print("===========================================")

    def construct_stacked_states_data(self) -> None:
        # (1) p - payload position
        self.stacked_states_data[:, 0] = self.data["fitZOriginalLength.px"]
        self.stacked_states_data[:, 1] = self.data["fitZOriginalLength.py"]
        self.stacked_states_data[:, 2] = self.data["fitZOriginalLength.pz"]
        
        # (2) pv - payload velocity
        self.stacked_states_data[:, 3] = self.data["fitZOriginalLength.pvx"]
        self.stacked_states_data[:, 4] = self.data["fitZOriginalLength.pvy"]
        self.stacked_states_data[:, 5] = self.data["fitZOriginalLength.pvz"]

        # (3) cp - cable unit vector (from the uav to the payload)
        self.stacked_states_data[:, 6] = (self.data["fitZOriginalLength.px"] - self.data["locSrv.x"]) / self.lc # cpx
        self.stacked_states_data[:, 7] = (self.data["fitZOriginalLength.py"] - self.data["locSrv.y"]) / self.lc # cpy
        self.stacked_states_data[:, 8] = (self.data["fitZOriginalLength.pz"] - self.data["locSrv.z"]) / self.lc # cpz

        # (4) pw - payload angular velocity (pw = cp @ cpv)
        cv = np.zeros((self.n, 3))
        cv[:, 0] = (self.data["fitZOriginalLength.pvx"] - self.data["stateEstimateZ.vx"]) / self.lc # cpvx
        cv[:, 1] = (self.data["fitZOriginalLength.pvy"] - self.data["stateEstimateZ.vy"]) / self.lc # cpvy
        cv[:, 2] = (self.data["fitZOriginalLength.pvz"] - self.data["stateEstimateZ.vz"]) / self.lc # cpvz

        self.stacked_states_data[:, 9:12] = np.cross(self.stacked_states_data[:, 6:9], cv) # pwx, pwy, pwz [rad/s]

        # (5) rpy - uav orientation
        phi =   np.radians(self.data["ctrlLee.rpyx"])   # [rad]
        theta = np.radians(self.data["ctrlLee.rpyy"])   # [rad]
        psi =   np.radians(self.data["ctrlLee.rpyz"])   # [rad]

        print("construct_stacked_states_data(): converting Euler angles to quaternions...")
        for i in range(self.n):
            self.stacked_states_data[i, 12:16] = from_euler(phi[i], theta[i], psi[i], convention="xyz")
        print("construct_stacked_states_data(): converting Euler angles to quaternions...done")

        # (6) w - uav angular velocity
        self.stacked_states_data[:, 16] = np.radians(self.data["ctrlLee.omegax"]) # [rad/s]
        self.stacked_states_data[:, 17] = np.radians(self.data["ctrlLee.omegay"]) # [rad/s]
        self.stacked_states_data[:, 18] = np.radians(self.data["ctrlLee.omegaz"]) # [rad/s]

    def construct_stacked_inputs_data(self) -> None:  
        self.stacked_inputs_data[:, 0] = self.data["ctrlLee.thrustSI"]
        self.stacked_inputs_data[:, 1] = self.data["ctrlLee.torquex"]
        self.stacked_inputs_data[:, 2] = self.data["ctrlLee.torquey"]
        self.stacked_inputs_data[:, 3] = self.data["ctrlLee.torquez"]

    def construct_stacked_states_model(self) -> None:
        # the initial model-based state should be the same as the data-based state to create a zero error
        self.stacked_states_model[0, :] = self.stacked_states_data[0, :]

        for i in range(1, self.n):
            # create an error of zero if the sampling time of the log is too large
            if self.delta[i - 1] > self.delta_threshold:
                self.stacked_states_model[i, :] = self.stacked_states_data[i, :]
                continue

            self.stacked_states_model[i, :] = self.stacked_states_data[i-1, :] + self.step(i) * self.delta[i-1]

    def step(self, index: int) -> np.ndarray:
        curr_posl = self.stacked_states_data[index - 1, 0:3]   
        curr_vl   = self.stacked_states_data[index - 1, 3:6]   
        curr_p    = self.stacked_states_data[index - 1, 6:9]   
        curr_wl   = self.stacked_states_data[index - 1, 9:12]  
        curr_q    = self.stacked_states_data[index - 1, 12:16] 
        curr_w    = self.stacked_states_data[index - 1, 16:19]  

        # extract thrust and torque
        fz        = self.stacked_inputs_data[index, 0]
        # fz_g      = fz - self.mp * 9.81
        tau       = self.stacked_inputs_data[index, 1:4]

        # get the next uav orientation and angular velocity (uav model only)
        wdot  = self.invI @ (tau - skew(curr_w) @ self.I @ curr_w)
        wNext = wdot * self.delta[index - 1] + curr_w
        qNext = multiply(curr_q, exp(_promote_vec(curr_w * self.delta[index - 1] / 2))) 
        
        # get the next payload position and translational velocity (payload uav model)
        R_IB  = to_matrix(curr_q)
        pd    = np.cross(curr_wl, curr_p)
        # u     = fz * R_IB * np.array([0, 0, 1]) 
        al    =  (1 / self.mt) * (self.g + (np.vdot(curr_p, R_IB @ np.array([0, 0, fz])) - (self.m * self.lc * (np.vdot(pd, pd)))) * curr_p)
        VlNext   = al * self.delta[index - 1] + curr_vl
        poslNext = curr_vl * self.delta[index - 1] + curr_posl

        # get the next cable unit vector and angular velocity of the unit vector (payload uav model)
        wld  = (1 / (self.lc * self.m)) * (skew(-curr_p) @ R_IB @ np.array([0, 0, fz]))
        wlNext  = wld * self.delta[index - 1] + curr_wl
        pd    =  skew(curr_wl) @ curr_p
        pNext    = pd * self.delta[index - 1] + curr_p

        return np.concatenate((poslNext, VlNext, pNext, wlNext, qNext, wNext)) 

    def reset_model(self) -> None:
        self.reset_states_model()
        self.reset_errors()

    def reset_states_model(self) -> None:
        self.stacked_states_model = np.zeros((self.n, self.n_payload))

    def reset_errors(self) -> None:
        self.stacked_errors = np.zeros((self.n, self.n_payload))
    
    def compute_errors(self) -> None:
        self.stacked_errors = self.stacked_states_data - self.stacked_states_model
    
    def get_error_payload_position_x(self) -> np.ndarray:
        return self.stacked_errors[:, 0]
    
    def get_error_payload_position_y(self) -> np.ndarray:
        return self.stacked_errors[:, 1]
    
    def get_error_payload_position_z(self) -> np.ndarray:
        return self.stacked_errors[:, 2]
    
    def get_error_payload_velocity_x(self) -> np.ndarray:
        return self.stacked_errors[:, 3]
    
    def get_error_payload_velocity_y(self) -> np.ndarray:
        return self.stacked_errors[:, 4]
    
    def get_error_payload_velocity_z(self) -> np.ndarray:
        return self.stacked_errors[:, 5]
    
    def get_error_cable_unit_vector_x(self) -> np.ndarray:
        return self.stacked_errors[:, 6]
    
    def get_error_cable_unit_vector_y(self) -> np.ndarray:
        return self.stacked_errors[:, 7]
    
    def get_error_cable_unit_vector_z(self) -> np.ndarray:
        return self.stacked_errors[:, 8]
    
    def get_error_payload_angular_velocity_x(self) -> np.ndarray:
        return np.degrees(self.stacked_errors[:, 9])
    
    def get_error_payload_angular_velocity_y(self) -> np.ndarray:
        return np.degrees(self.stacked_errors[:, 10])
    
    def get_error_payload_angular_velocity_z(self) -> np.ndarray:
        return np.degrees(self.stacked_errors[:, 11])
    
    def compute_residuals(self) -> None:
        # TODO: define output of this function, what should be plotted?

        pass
        # return self.stacked_errors[:, 0]  # currently outputting the payload position error only
    

import numpy as np


class ResidualsPayload():
    def __init__(self, data: dict[str, np.ndarray]) -> None:
        self.data = data
        self.n = len(self.data["timestamp"])

        self.n_uav = 13
        self.n_payload = 19
        self.n_input = 4

        self.lc = 0.5  
        self.m_uav  = 0.032  
        self.m_payload = 0.05   
        
        self.stacked_states_data = np.zeros((self.n, self.n_payload))
        self.construct_stacked_states_data()

        self.stacked_inputs_data = np.zeros((self.n, self.n_input))
        self.construct_stacked_inputs_data()

        self.stacked_states_model = np.zeros((self.n, self.n_payload))
        self.construct_stacked_states_model()

        # payload uav model parameters (Khaled's script)
        self.mp = 0.005                                # mass of payload [kg]
        self.m  = 0.032                                # quadrotor mass [kg]
        self.mt = self.m + self.mp                     # total mass [kg]
        self.lc = 0.500                                # length of cable [m]
        self.g  = np.array([0, 0, -self.mt * 9.81])    # gravity vector [m/s^2]

        self.stacked_errors = np.zeros((self.n, self.n_payload))

    def construct_stacked_states_data(self) -> None:
        # payload position
        self.stacked_states_data[:, 0] = self.data["fitZOriginalLength.px"]
        self.stacked_states_data[:, 1] = self.data["fitZOriginalLength.py"]
        self.stacked_states_data[:, 2] = self.data["fitZOriginalLength.pz"]
        
        # payload velocity
        self.stacked_states_data[:, 3] = self.data["fitZOriginalLength.pvx"]
        self.stacked_states_data[:, 4] = self.data["fitZOriginalLength.pvy"]
        self.stacked_states_data[:, 5] = self.data["fitZOriginalLength.pvz"]

        # unit vector from uav to payload
        self.stacked_states_data[:, 6] = (self.data["fitZOriginalLength.px"] - self.data["locSrv.x"]) / self.lc
        self.stacked_states_data[:, 7] = (self.data["fitZOriginalLength.py"] - self.data["locSrv.y"]) / self.lc
        self.stacked_states_data[:, 8] = (self.data["fitZOriginalLength.pz"] - self.data["locSrv.z"]) / self.lc

        # angular velocity of the above unit vector
        self.stacked_states_data[:, 9] = (self.data["fitZOriginalLength.pvx"] - self.data["stateEstimateZ.vx"]) / self.lc
        self.stacked_states_data[:, 10] = (self.data["fitZOriginalLength.pvy"] - self.data["stateEstimateZ.vy"]) / self.lc
        self.stacked_states_data[:, 11] = (self.data["fitZOriginalLength.pvz"] - self.data["stateEstimateZ.vz"]) / self.lc

        # uav orientation
        self.stacked_states_data[:, 12] = self.data["locSrv.qw"]
        self.stacked_states_data[:, 13] = self.data["locSrv.qx"]
        self.stacked_states_data[:, 14] = self.data["locSrv.qy"]
        self.stacked_states_data[:, 15] = self.data["locSrv.qz"]

        #   uav angular velocity
        self.stacked_states_data[:, 16] = self.data["ctrlLee.omegax"]
        self.stacked_states_data[:, 17] = self.data["ctrlLee.omegay"]
        self.stacked_states_data[:, 18] = self.data["ctrlLee.omegaz"]

    def construct_stacked_inputs_data(self) -> None:  
        self.stacked_inputs_data[:, 0] = self.data["ctrlLee.thrustSI"]
        self.stacked_inputs_data[:, 1] = self.data["ctrlLee.torquex"]
        self.stacked_inputs_data[:, 2] = self.data["ctrlLee.torquey"]
        self.stacked_inputs_data[:, 3] = self.data["ctrlLee.torquez"]

    def construct_stacked_states_model(self) -> None:
        for i in range(1, self.n):
            delta = self.data["timestamp"][i] - self.data["timestamp"][i-1]
            self.stacked_states_model[i, :] = self.stacked_states_data[i-1, :] + self.step(i) * delta

    def step(self, index: int) -> np.ndarray:
        curr_posl = self.stacked_states_data[index - 1, 0:3]   
        curr_vl   = self.stacked_states_data[index - 1, 3:6]   
        curr_p    = self.stacked_states_data[index - 1, 6:9]   
        curr_wl   = self.stacked_states_data[index - 1, 9:12]  
        curr_q    = self.stacked_states_data[index - 1, 12:16] 
        curr_w    = self.stacked_states_data[index - 1, 16:19]  

        fz        = self.stacked_inputs_data[index, 0]
        fz_g      -= self.mp * 9.81
        tau       = self.stacked_inputs_data[index, 1:4]

        # TODO: continue here to compute the orientation and angular velocity of the uav
        # uavState = uav.states_evolution(control_t)
        # qNext = uav.state[6:10]
        # wNext = uav.state[10::]


    def reset_model(self) -> None:
        self.reset_states_model()
        self.reset_errors()

    def reset_states_model(self) -> None:
        self.stacked_states_model = np.zeros((self.n, self.n_payload))

    def reset_errors(self) -> None:
        self.stacked_errors = np.zeros((self.n, self.n_payload))

    def compute_residuals(self) -> np.ndarray:
        # TODO: define output of this function, what should be plotted?

        return self.stacked_errors[:, 0]  # currently outputting the payload position error only
    
    def compute_error(self) -> np.ndarray:
        self.stacked_errors = self.stacked_states_data - self.stacked_states_model
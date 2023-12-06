# -*- coding: utf-8 -*-
"""
Tool for manipulating and adding data to the automatically generated reports.
"""
import numpy as np
from numpy.polynomial import polynomial as P
from scipy.interpolate import CubicSpline, BSpline, splrep
import matplotlib.pyplot as plt

import uav


class DataHelper:
    def __init__(self) -> None:
        pass
    
    @staticmethod
    def generate_data(data: dict[str, np.ndarray], 
                      event: str, 
                      info: dict[str, str | int | float]) -> tuple[str, np.ndarray]:
        source = data[event].get(info.get("source", None), None)
        t = data[event]["timestamp"]
        t_fit = data[event].get("fitTimestamp", None)
        
        if info.get("derivative", 0) < 0:
            raise ValueError("Derivative must be greater than or equal to 0")

        if info["type"] == "linspace":
            data = DataHelper.generate_data_linspace(source, info["step"])
        elif info["type"] == "poly":
            data = DataHelper.generate_data_poly(t, source, t_fit, info)
        elif info["type"] == "cs":
            data = DataHelper.generate_data_cs(t, source, t_fit, info)
        elif info["type"] == "bs":
            data = DataHelper.generate_data_bs(t, source, t_fit, info)
        elif info["type"] == "residuals_payload":
            data = DataHelper.generate_data_residuals_payload(data[event])
        else:
            raise NotImplementedError

        return info["target"], data

    @staticmethod
    def generate_data_linspace(x: np.ndarray, step: int) -> np.ndarray:
        return np.arange(x[0], x[-1], step)

    @staticmethod
    def generate_data_poly(x: np.ndarray, y: np.ndarray, x_fit: np.ndarray, info: dict[str, str | int | float]) -> np.ndarray:
        p = P.Polynomial.fit(x, y,  info["degree"])
        p = p.deriv(info["derivative"])

        if not info.get("original_length", False):
            return p(x_fit)

        return p(x)
    
    @staticmethod
    def generate_data_cs(x: np.ndarray, y: np.ndarray, x_fit: np.ndarray, info: dict[str, str | int | float]) -> np.ndarray:
        cs = CubicSpline(x, y)

        if not info.get("original_length", False):
            return cs(x_fit, info["derivative"])
        
        return cs(x, info["derivative"])
    
    @staticmethod
    def generate_data_bs(x: np.ndarray, y: np.ndarray, x_fit: np.ndarray, info: dict[str, str | int | float]) -> np.ndarray:
        tck = splrep(x, y, s=info["smoothing"])
        bs = BSpline(*tck)

        if not info.get("original_length", False):
            return bs(x_fit, info["derivative"])
        
        return bs(x, info["derivative"])  
    
    @staticmethod
    def generate_data_residuals_payload(data: dict[str, np.ndarray]) -> np.ndarray:
        res_payload = ResidualsPayload(data)

        return res_payload.compute_residuals()


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

        # self.uav_model = uav.UavModel()
        # self.payload_model = uav.PayloadModel()

    def construct_stacked_states_data(self) -> None:
        self.stacked_states_data[:, 0] = self.data["fitZOriginalLength.px"]
        self.stacked_states_data[:, 1] = self.data["fitZOriginalLength.py"]
        self.stacked_states_data[:, 2] = self.data["fitZOriginalLength.pz"]
        
        self.stacked_states_data[:, 3] = self.data["fitZOriginalLength.pvx"]
        self.stacked_states_data[:, 4] = self.data["fitZOriginalLength.pvy"]
        self.stacked_states_data[:, 5] = self.data["fitZOriginalLength.pvz"]

        self.stacked_states_data[:, 6] = (self.data["fitZOriginalLength.px"] - self.data["locSrv.x"]) / self.lc
        self.stacked_states_data[:, 7] = (self.data["fitZOriginalLength.py"] - self.data["locSrv.y"]) / self.lc
        self.stacked_states_data[:, 8] = (self.data["fitZOriginalLength.pz"] - self.data["locSrv.z"]) / self.lc

        self.stacked_states_data[:, 9] = (self.data["fitZOriginalLength.pvx"] - self.data["stateEstimateZ.vx"]) / self.lc
        self.stacked_states_data[:, 10] = (self.data["fitZOriginalLength.pvy"] - self.data["stateEstimateZ.vy"]) / self.lc
        self.stacked_states_data[:, 11] = (self.data["fitZOriginalLength.pvz"] - self.data["stateEstimateZ.vz"]) / self.lc

        self.stacked_states_data[:, 12] = self.data["locSrv.qw"]
        self.stacked_states_data[:, 13] = self.data["locSrv.qx"]
        self.stacked_states_data[:, 14] = self.data["locSrv.qy"]
        self.stacked_states_data[:, 15] = self.data["locSrv.qz"]

        self.stacked_states_data[:, 16] = self.data["ctrlLee.omegax"]
        self.stacked_states_data[:, 17] = self.data["ctrlLee.omegay"]
        self.stacked_states_data[:, 18] = self.data["ctrlLee.omegaz"]

    def construct_stacked_inputs_data(self) -> None:  
        self.stacked_inputs_data[:, 0] = self.data["ctrlLee.thrustSI"]
        self.stacked_inputs_data[:, 1] = self.data["ctrlLee.torquex"]
        self.stacked_inputs_data[:, 2] = self.data["ctrlLee.torquey"]
        self.stacked_inputs_data[:, 3] = self.data["ctrlLee.torquez"]

    def construct_stacked_states_model(self) -> None:
        pass

    def compute_residuals(self) -> np.ndarray:
        return self.compute_error()
    
    def compute_error(self) -> np.ndarray:
        # e of pos_x, pos_y, pos_z for starters
        pass

    def step(self) -> None:
        pass
    
    
if __name__ == "__main__":
    pass
    # small test
    # data = {"event": {
    #         "timestamp": np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
    #         "source": np.array([10, 20, 15, 30, 15, 10, 5, 10, 15, 20])}}

    # info = {"type": "poly",
    #         "degree": 100,
    #         "derivative": 1,
    #         "source": "source",
    #         "target": "target"}

    # target, generated_data = DataHelper.generate_data(data, "event", info)
    # plt.plot(data["event"]["timestamp"], data["event"]["source"], label="source")
    # plt.plot(data["event"]["timestamp"], generated_data, label="generated")
    # plt.legend()
    # plt.show()

    # another small test
    # x = np.arange(10)
    # y = np.sin(x)
    # cs = CubicSpline(x, y)
    # xs = np.arange(-0.5, 9.6, 0.1)
    # fig, ax = plt.subplots(figsize=(6.5, 4))
    # ax.plot(x, y, 'o', label='data')
    # ax.plot(xs, np.sin(xs), label='true')
    # ax.plot(xs, cs(xs), label="S")
    # ax.plot(xs, cs.derivative(1), label="S'")
    # ax.plot(xs, cs.derivative(2), label="S''")
    # ax.plot(xs, cs.derivative(3), label="S'''")
    # ax.set_xlim(-0.5, 9.5)
    # ax.legend(loc='lower left', ncol=2)
    # plt.show()

    # print(cs.c.shape)
    # csder = cs.derivative(1)
    # print(csder.c.shape)
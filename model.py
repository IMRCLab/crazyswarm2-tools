# -*- coding: utf-8 -*-
"""
Tool for manipulating and adding data to the automatically generated reports.
"""
import numpy as np
from numpy.polynomial import polynomial as P
from scipy.interpolate import CubicSpline, BSpline, splrep
import matplotlib.pyplot as plt


class DataHelper:
    def __init__(self) -> None:
        pass
    
    @staticmethod
    def generate_data(data: dict[str, np.ndarray], 
                      event: str, 
                      info: dict[str, str | int | float]) -> tuple[str, np.ndarray]:
        source = data[event][info["source"]]
        t = data[event]["timestamp"]
        t_fit = data[event].get("fitTimestamp", None)
        
        if info.get("derivative", 0) < 0:
            raise ValueError("Derivative must be greater than or equal to 0")

        if info["type"] == "linspace":
            data = DataHelper.generate_data_linspace(source, info["step"])
        elif info["type"] == "poly":
            data = DataHelper.generate_data_poly(t, source, t_fit, info["derivative"], info["degree"])
        elif info["type"] == "cs":
            data = DataHelper.generate_data_cs(t, source, t_fit, info["derivative"])
        elif info["type"] == "bs":
            data = DataHelper.generate_data_bs(t, source, t_fit, info["derivative"], info["smoothing"])
        else:
            raise NotImplementedError

        return info["target"], data

    @staticmethod
    def generate_data_linspace(x: np.ndarray, step: int) -> np.ndarray:
        return np.arange(x[0], x[-1], step)

    @staticmethod
    def generate_data_poly(x: np.ndarray, y: np.ndarray, x_fit: np.ndarray, derivative: int, degree: int) -> np.ndarray:
        p = P.Polynomial.fit(x, y, degree)
        p = p.deriv(derivative)

        if x_fit is not None:
            return p(x_fit)

        return p(x)
    
    @staticmethod
    def generate_data_cs(x: np.ndarray, y: np.ndarray, x_fit: np.ndarray, derivative: int) -> np.ndarray:
        cs = CubicSpline(x, y)

        if x_fit is not None:
            return cs(x_fit, derivative)
        
        return cs(x, derivative)
    
    @staticmethod
    def generate_data_bs(x: np.ndarray, y: np.ndarray, x_fit: np.ndarray, derivative: int, smoothing: float) -> np.ndarray:
        tck = splrep(x, y, s=smoothing)
        bs = BSpline(*tck)
        bs = bs.derivative(derivative)

        if x_fit is not None:
            return bs(x_fit)
        
        return bs(x)
    

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
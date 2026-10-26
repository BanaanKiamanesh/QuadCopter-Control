import os
import numpy as np
import matplotlib.pyplot as plt

from quadcopter import QuadCopter
from visualization import QuadCopterVisualizer


def Simulate(SimTime):
    Q = QuadCopter()
    Q.ResetController()

    X = np.zeros(Q.SystemOrder)
    X[0:3] = Q.Reference(0.0)[0:3]

    X_hat = X.copy()
    X_hat[0:3] += np.array([0.08, -0.06, -0.10])
    X_hat[3:6] += np.array([0.08, -0.05, 0.10])
    X_hat[6:9] += np.array([0.02, -0.02, 0.03])

    Time = np.arange(0.0, SimTime + 0.5 * Q.ControlDt, Q.ControlDt)
    Motion = np.zeros((len(Time), Q.SystemOrder))
    Estimate = np.zeros((len(Time), Q.SystemOrder))
    Reference = np.zeros((len(Time), Q.SystemOrder))
    Control = np.zeros((len(Time), Q.InputOrder))

    for i, t in enumerate(Time):
        Motion[i] = X
        Estimate[i] = X_hat
        Reference[i] = Q.Reference(t)

        if i == len(Time) - 1:
            break

        U = Q.ControlInput(X_hat, t)
        Control[i] = U

        for _ in range(round(Q.ControlDt / Q.IntegrationDt)):
            X, X_hat = Q.Step(X, X_hat, U, Q.IntegrationDt)

    Control[-1] = Control[-2]

    return Q, Time, Motion, Estimate, Reference, Control


if __name__ == "__main__":
    Q, Time, Motion, Estimate, Reference, Control = Simulate(10.0)

    Visualizer = QuadCopterVisualizer(Q, Time, Motion, Estimate, Reference, Control)
    Visualizer.PrintResults()
    Visualizer.CreatePlotsFigure()
    Animation = Visualizer.CreateAnimation(Fps=20)

    OutputDir = os.path.join(os.path.dirname(__file__), "results")
    os.makedirs(OutputDir, exist_ok=True)
    Visualizer.SavePlotsFigure(os.path.join(OutputDir, "tracking_plots.svg"))
    Visualizer.SaveAnimation(os.path.join(OutputDir, "quadcopter_animation.gif"))

    if plt.get_backend().lower() != "agg":
        plt.show()

import os
import numpy as np
import matplotlib.pyplot as plt

from quadcopter import QuadCopter
from visualization import QuadCopterVisualizer


def Simulate(SimTime):
    Q = QuadCopter()
    Q.ResetController()
    Q.ResetObserver()

    # Gain schedule designed offline along the reference
    Q.LinearizeReference(SimTime)
    Q.DesignController()

    X = np.zeros(Q.SystemOrder)
    X[0:6] = Q.Reference(0.0)[0:6]

    # EKF starts with position, attitude, velocity and rate errors
    Xhat = X.copy()
    Xhat[0:3] += np.array([0.08, -0.06, -0.10])
    Xhat[6:9] += np.array([0.03, -0.03, 0.05])
    Xhat[3:6] += np.array([0.20, -0.15, 0.10])
    Xhat[9:12] += np.array([0.10, -0.10, 0.05])

    Time = np.arange(0.0, SimTime + 0.5 * Q.ControlDt, Q.ControlDt)
    Motion = np.zeros((len(Time), Q.SystemOrder))
    Estimate = np.zeros((len(Time), Q.SystemOrder))
    Reference = np.zeros((len(Time), Q.SystemOrder))
    Control = np.zeros((len(Time), Q.InputOrder))

    for Index, TimeValue in enumerate(Time):
        Motion[Index] = X
        Estimate[Index] = Xhat
        Reference[Index] = Q.Reference(TimeValue)

        if Index == len(Time) - 1:
            break

        # The controller only sees the EKF estimate
        U = Q.ControlInput(Xhat, TimeValue)
        Control[Index] = U

        for _ in range(round(Q.ControlDt / Q.IntegrationDt)):
            Xhat = Q.ObserverStep(Xhat, Q.Measure(X), U, Q.IntegrationDt)
            X = Q.Step(X, U, Q.IntegrationDt)

    Control[-1] = Control[-2]
    return Q, Time, Motion, Estimate, Reference, Control


if __name__ == "__main__":
    Q, Time, Motion, Estimate, Reference, Control = Simulate(10.0)

    Visualizer = QuadCopterVisualizer(Q, Time, Motion, Estimate, Reference, Control)
    Visualizer.PrintResults()
    Visualizer.CreatePlotsFigure()
    Visualizer.CreateTrajectoryFigure()
    Visualizer.CreateAnimation(Fps=20)

    OutputDir = os.path.join(os.path.dirname(__file__), "results")
    os.makedirs(OutputDir, exist_ok=True)
    Visualizer.SavePlotsFigure(os.path.join(OutputDir, "tracking_plots.svg"))
    Visualizer.SaveTrajectoryFigure(os.path.join(OutputDir, "trajectory_3d.svg"))
    Visualizer.SaveAnimation(os.path.join(OutputDir, "quadcopter_animation.gif"))

    if plt.get_backend().lower() != "agg":
        plt.show()

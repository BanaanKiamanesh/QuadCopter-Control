import io
import os
import zipfile
import numpy as np
import matplotlib.pyplot as plt

from quadcopter import QuadCopter
from visualization import QuadCopterVisualizer


def SaveSimulationData(FilePath, **Arrays):
    # .npz archive with fixed timestamps, so the file only changes when the data does
    with zipfile.ZipFile(FilePath, "w", zipfile.ZIP_DEFLATED) as Archive:
        for Name, Array in Arrays.items():
            Buffer = io.BytesIO()
            np.save(Buffer, np.asarray(Array))
            Info = zipfile.ZipInfo(Name + ".npy", date_time=(2026, 1, 1, 0, 0, 0))
            Info.compress_type = zipfile.ZIP_DEFLATED
            Archive.writestr(Info, Buffer.getvalue())


def Simulate(SimTime):
    Q = QuadCopter()
    Q.ResetController()
    Q.ResetObserver()

    X = np.zeros(Q.SystemOrder)
    X[0:6] = Q.Reference(0.0)[0:6]

    # Observer starts with position, attitude, velocity and rate errors
    Z = Q.ObserverFromState(X)
    Z[0:3] += np.array([0.08, -0.06, -0.10])
    Z[3:6] += np.array([0.03, -0.03, 0.05])
    Z[6:9] += np.array([0.20, -0.15, 0.10])
    Z[9:12] += np.array([0.10, -0.10, 0.05])

    Time = np.arange(0.0, SimTime + 0.5 * Q.ControlDt, Q.ControlDt)
    Motion = np.zeros((len(Time), Q.SystemOrder))
    Estimate = np.zeros((len(Time), Q.SystemOrder))
    Reference = np.zeros((len(Time), Q.SystemOrder))
    Control = np.zeros((len(Time), Q.InputOrder))

    for Index, TimeValue in enumerate(Time):
        Motion[Index] = X
        Estimate[Index] = Q.StateFromObserver(Z)
        Reference[Index] = Q.Reference(TimeValue)

        if Index == len(Time) - 1:
            break

        # The controller only sees the observer estimate
        U = Q.ControlInput(Estimate[Index], TimeValue)
        Control[Index] = U

        for _ in range(round(Q.ControlDt / Q.IntegrationDt)):
            Z = Q.ObserverStep(Z, Q.Measure(X), U, Q.IntegrationDt)
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
    SaveSimulationData(os.path.join(OutputDir, "simulation_data.npz"), Name="4. Sliding-Mode Controller + Observer",
                       Time=Time, Motion=Motion, Estimate=Estimate, Reference=Reference, Control=Control)

    if plt.get_backend().lower() != "agg":
        plt.show()

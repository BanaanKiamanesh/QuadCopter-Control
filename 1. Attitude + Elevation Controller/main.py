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


def Simulate(SimTime=10.0):
    Q = QuadCopter()
    X = np.zeros(Q.SystemOrder)
    X[0:6] = Q.Reference(0.0)[0:6]
    X[2] -= 0.20
    X[6:9] = np.array([0.06, -0.05, 0.10])

    Time = np.arange(0.0, SimTime + 0.5 * Q.ControlDt, Q.ControlDt)
    Motion = np.zeros((len(Time), Q.SystemOrder))
    Reference = np.zeros_like(Motion)
    Control = np.zeros((len(Time), Q.InputOrder))

    for Index, TimeValue in enumerate(Time):
        Motion[Index] = X
        Reference[Index] = Q.Reference(TimeValue)
        if Index == len(Time) - 1:
            break
        U = Q.ControlInput(X, TimeValue)
        Control[Index] = U
        for _ in range(round(Q.ControlDt / Q.IntegrationDt)):
            X = Q.Step(X, U, Q.IntegrationDt)

    Control[-1] = Control[-2]
    return Q, Time, Motion, Reference, Control


if __name__ == "__main__":
    Q, Time, Motion, Reference, Control = Simulate()
    Visualizer = QuadCopterVisualizer(Q, Time, Motion, Reference, Control)
    Visualizer.PrintResults()
    Visualizer.CreatePlotsFigure()
    Visualizer.CreateTrajectoryFigure()
    Visualizer.CreateAnimation(Fps=20)

    OutputDir = os.path.join(os.path.dirname(__file__), "results")
    os.makedirs(OutputDir, exist_ok=True)
    Visualizer.SavePlotsFigure(os.path.join(OutputDir, "tracking_plots.svg"))
    Visualizer.SaveTrajectoryFigure(os.path.join(OutputDir, "trajectory_3d.svg"))
    Visualizer.SaveAnimation(os.path.join(OutputDir, "quadcopter_animation.gif"))
    SaveSimulationData(os.path.join(OutputDir, "simulation_data.npz"), Name="1. Attitude + Elevation Controller",
                       Time=Time, Motion=Motion, Reference=Reference, Control=Control)
    if plt.get_backend().lower() != "agg":
        plt.show()

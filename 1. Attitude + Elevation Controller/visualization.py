import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


class QuadCopterVisualizer:
    def __init__(self, Quad, Time, Motion, Reference, Control):
        self.Quad = Quad
        self.Time = Time
        self.Motion = Motion
        self.Reference = Reference
        self.Control = Control

        self.PlotFigure = None
        self.TrajectoryFigure = None
        self.AnimationFigure = None
        self.Animation = None
        self.AnimationFps = 20

    def PrintResults(self):
        Error = self.Motion[:, 6:9] - self.Reference[:, 6:9]
        Zerror = self.Motion[:, 2] - self.Reference[:, 2]
        print("Attitude RMSE (deg):", np.round(np.rad2deg(np.sqrt(np.mean(Error ** 2, axis=0))), 4))
        print("Elevation RMSE (m):", round(np.sqrt(np.mean(Zerror ** 2)), 4))

    def CreatePlotsFigure(self):
        self.PlotFigure, Axes = plt.subplots(3, 2, figsize=(13, 10))
        self.PlotFigure.suptitle("Attitude and Elevation Control")
        for Index, Label in enumerate(("Roll", "Pitch", "Yaw")):
            Ax = Axes.flat[Index]
            Ax.plot(self.Time, np.rad2deg(self.Motion[:, Index + 6]), label="Actual")
            Ax.plot(self.Time, np.rad2deg(self.Reference[:, Index + 6]), "--", label="Reference")
            Ax.set(title=Label, ylabel="Angle (deg)")
            Ax.legend()
        Axes[1, 1].plot(self.Time, self.Motion[:, 2], label="Actual")
        Axes[1, 1].plot(self.Time, self.Reference[:, 2], "--", label="Reference")
        Axes[1, 1].set(title="Elevation", ylabel="Z (m)")
        Axes[1, 1].legend()
        Axes[2, 0].plot(self.Time, self.Control[:, 0], label="Thrust")
        Axes[2, 0].set(title="Collective Thrust", xlabel="Time (s)", ylabel="N")
        for Index, Label in enumerate(("Roll", "Pitch", "Yaw")):
            Axes[2, 1].plot(self.Time, self.Control[:, Index + 1], label=Label)
        Axes[2, 1].set(title="Body Torques", xlabel="Time (s)", ylabel="N m")
        Axes[2, 1].legend()
        for Ax in Axes.flat:
            Ax.grid(True)
        self.PlotFigure.tight_layout()
        return self.PlotFigure

    def CreateTrajectoryFigure(self):
        self.TrajectoryFigure = plt.figure(figsize=(8, 7))
        Ax = self.TrajectoryFigure.add_subplot(111, projection="3d")
        Ax.plot(*self.Motion[:, 0:3].T, label="Actual")
        Ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title="3D Flight (XY Uncontrolled)")
        Ax.legend()
        return self.TrajectoryFigure

    def CreateAnimation(self, Fps=20):
        self.AnimationFps = Fps
        self.AnimationFigure = plt.figure(figsize=(7.6, 6.4), dpi=85)
        Ax = self.AnimationFigure.add_subplot(111, projection="3d")
        Ax.view_init(elev=24, azim=-62)

        Positions = np.vstack((self.Motion[:, 0:3], self.Reference[:, 0:3]))
        Minimum = np.min(Positions, axis=0) - np.array([0.5, 0.5, 0.6])
        Maximum = np.max(Positions, axis=0) + np.array([0.5, 0.5, 0.6])
        Minimum[2] = min(0.0, Minimum[2])

        Ax.set(xlim=(Minimum[0], Maximum[0]),
               ylim=(Minimum[1], Maximum[1]),
               zlim=(Minimum[2], Maximum[2]),
               xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")
        Ax.set_box_aspect(Maximum - Minimum, zoom=0.83)
        Ax.set_title("Attitude + Elevation (XY uncontrolled)", pad=18, fontsize=12)
        Ax.grid(True, alpha=0.25)
        for Pane in (Ax.xaxis.pane, Ax.yaxis.pane, Ax.zaxis.pane):
            Pane.fill = False

        # Horizontal position is not controlled in this example.

        ArmLength = self.Quad.ArmLength
        MotorCoord = np.array([
            [ArmLength, 0.0, 0.0],
            [0.0, ArmLength, 0.0],
            [-ArmLength, 0.0, 0.0],
            [0.0, -ArmLength, 0.0]
        ]).T

        ArmX, = Ax.plot([], [], [], "-o", color="tab:red", linewidth=3, markersize=5)
        ArmY, = Ax.plot([], [], [], "-o", color="tab:blue", linewidth=3, markersize=5)
        Body, = Ax.plot([], [], [], "ok", markersize=5)
        Path, = Ax.plot([], [], [], color="tab:blue", linewidth=1.8, label="Flight path")
        Shadow, = Ax.plot([], [], [], "x", color="0.6", markersize=5)
        TimeText = Ax.text2D(0.03, 0.94, "", transform=Ax.transAxes, fontsize=12)
        Ax.legend(loc="upper right", fontsize=9)

        FrameCount = int(round((self.Time[-1] - self.Time[0]) * Fps))
        FrameIndices = np.rint(np.linspace(0, len(self.Time) - 1, FrameCount)).astype(int)

        def Update(FrameIndex):
            Index = FrameIndices[FrameIndex]
            Center = self.Motion[Index, 0:3]
            Euler = self.Motion[Index, 6:9]
            Motors = self.Quad.RotationMatrix(Euler) @ MotorCoord + Center[:, None]

            ArmX.set_data(Motors[0, [0, 2]], Motors[1, [0, 2]])
            ArmX.set_3d_properties(Motors[2, [0, 2]])
            ArmY.set_data(Motors[0, [1, 3]], Motors[1, [1, 3]])
            ArmY.set_3d_properties(Motors[2, [1, 3]])
            Body.set_data([Center[0]], [Center[1]])
            Body.set_3d_properties([Center[2]])
            Path.set_data(self.Motion[:Index + 1, 0], self.Motion[:Index + 1, 1])
            Path.set_3d_properties(self.Motion[:Index + 1, 2])
            Shadow.set_data([Center[0]], [Center[1]])
            Shadow.set_3d_properties([Minimum[2]])
            TimeText.set_text(f"t = {self.Time[Index]:05.2f} / 10.00 s")
            return ArmX, ArmY, Body, Path, Shadow, TimeText

        self.Animation = FuncAnimation(
            self.AnimationFigure, Update,
            frames=FrameCount, interval=1000.0 / Fps,
            blit=False, repeat=False, cache_frame_data=False
        )
        return self.Animation

    def SaveAnimation(self, FilePath):
        if self.Animation is None:
            self.CreateAnimation()
        self.Animation.save(FilePath, writer=PillowWriter(fps=self.AnimationFps))

    def SavePlotsFigure(self, FilePath):
        self.PlotFigure.savefig(FilePath, format="svg", bbox_inches="tight")

    def SaveTrajectoryFigure(self, FilePath):
        self.TrajectoryFigure.savefig(FilePath, format="svg", bbox_inches="tight")

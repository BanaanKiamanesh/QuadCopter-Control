import numpy as np


class QuadCopter:
    def __init__(self):
        # System Dynamical Properties
        self.Mass = 0.468
        self.Ixx = 0.0023
        self.Iyy = 0.0023
        self.Izz = 0.0046
        self.g = 9.81
        self.ArmLength = 0.17
        self.Thrust2Drag = 0.016

        self.I = np.diag([self.Ixx, self.Iyy, self.Izz])
        self.InvI = np.linalg.inv(self.I)

        self.SystemOrder = 12
        self.InputOrder = 4
        self.ControlDt = 0.01
        self.IntegrationDt = 0.005
        self.MaxTilt = np.deg2rad(30.0)

        # Total thrust and body torques: [T, TauPhi, TauTheta, TauPsi]
        self.MinInput = np.array([1.5, -0.040, -0.040, -0.028])
        self.MaxInput = np.array([8.0, 0.040, 0.040, 0.028])

        # Rotor order: front, right, rear, left
        L = self.ArmLength
        C = self.Thrust2Drag
        self.MixingMatrix = np.array([
            [1.0, 1.0, 1.0, 1.0],
            [0.0, L, 0.0, -L],
            [L, 0.0, -L, 0.0],
            [C, -C, C, -C]
        ])
        self.InvMixingMatrix = np.linalg.inv(self.MixingMatrix)

        # Controller Properties
        self.PositionLambda = np.array([1.6, 1.6, 2.0])
        self.PositionSwitch = np.array([3.0, 3.0, 3.0])
        self.PositionBoundary = np.array([0.25, 0.25, 0.25])

        self.AngularLambda = np.array([4.0, 4.0, 3.0])
        self.AngularSwitch = np.array([10.0, 10.0, 8.0])
        self.AngularBoundary = np.array([0.12, 0.12, 0.12])

        self.ResetController()

    def RotationMatrix(self, Euler):
        Phi, Theta, Psi = Euler

        cPhi, sPhi = np.cos(Phi), np.sin(Phi)
        cThe, sThe = np.cos(Theta), np.sin(Theta)
        cPsi, sPsi = np.cos(Psi), np.sin(Psi)

        return np.array([
            [cThe * cPsi, sPhi * sThe * cPsi - cPhi * sPsi, cPhi * sThe * cPsi + sPhi * sPsi],
            [cThe * sPsi, sPhi * sThe * sPsi + cPhi * cPsi, cPhi * sThe * sPsi - sPhi * cPsi],
            [-sThe, cThe * sPhi, cThe * cPhi]
        ])

    def EulerRateMatrix(self, Euler):
        Phi, Theta = Euler[0:2]
        return np.array([
            [1.0, 0.0, -np.sin(Theta)],
            [0.0, np.cos(Phi), np.cos(Theta) * np.sin(Phi)],
            [0.0, -np.sin(Phi), np.cos(Theta) * np.cos(Phi)]
        ])

    def EulerRates(self, X):
        return np.linalg.solve(self.EulerRateMatrix(X[6:9]), X[9:12])

    def SysDyn(self, X, U):
        # States: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        T = U[0]
        Tau = U[1:4]
        Orient = X[6:9]
        Omega = X[9:12]

        dX = np.zeros(self.SystemOrder)
        dX[0:3] = X[3:6]
        dX[3:6] = np.array([0.0, 0.0, -self.g]) + self.RotationMatrix(Orient)[:, 2] * (T / self.Mass)
        dX[6:9] = self.EulerRates(X)
        dX[9:12] = self.InvI @ (Tau - np.cross(Omega, self.I @ Omega))

        return dX

    def Step(self, X, U, Dt):
        # Fourth-order Runge-Kutta integration
        K1 = self.SysDyn(X, U)
        K2 = self.SysDyn(X + 0.5 * Dt * K1, U)
        K3 = self.SysDyn(X + 0.5 * Dt * K2, U)
        K4 = self.SysDyn(X + Dt * K3, U)
        return X + Dt * (K1 + 2.0 * K2 + 2.0 * K3 + K4) / 6.0

    def Reference(self, TimeValue):
        Xref = np.zeros(self.SystemOrder)
        Xref[0:3] = np.array([np.cos(TimeValue), np.sin(2.0 * TimeValue), 2.5 + np.sin(TimeValue)])
        Xref[3:6] = np.array([-np.sin(TimeValue), 2.0 * np.cos(2.0 * TimeValue), np.cos(TimeValue)])
        Xref[8] = np.sin(TimeValue)
        Xref[11] = np.cos(TimeValue)
        return Xref

    def ReferenceAcceleration(self, TimeValue):
        return np.array([-np.cos(TimeValue), -4.0 * np.sin(2.0 * TimeValue), -np.sin(TimeValue)])

    def WrapAngle(self, Angle):
        return (Angle + np.pi) % (2.0 * np.pi) - np.pi

    def DesiredAttitude(self, Acceleration, Yaw):
        # Convert world-frame acceleration to total thrust and desired attitude.
        Force = Acceleration + np.array([0.0, 0.0, self.g])
        Force[2] = max(Force[2], 0.1)

        Horizontal = np.linalg.norm(Force[0:2])
        MaxHorizontal = Force[2] * np.tan(self.MaxTilt)
        if Horizontal > MaxHorizontal:
            Force[0:2] *= MaxHorizontal / Horizontal

        Forward = np.cos(Yaw) * Force[0] + np.sin(Yaw) * Force[1]
        Sideways = np.sin(Yaw) * Force[0] - np.cos(Yaw) * Force[1]
        Phi = np.arctan2(Sideways, np.hypot(Forward, Force[2]))
        Theta = np.arctan2(Forward, Force[2])

        Thrust = self.Mass * np.linalg.norm(Force)
        DesiredEuler = np.array([Phi, Theta, Yaw])
        return Thrust, DesiredEuler

    def EulerAccelerationToTorque(self, X, EulerAcceleration):
        # Convert the desired Euler acceleration into body torques.
        Phi, Theta = X[6:8]
        EulerDot = self.EulerRates(X)
        PhiDot, ThetaDot = EulerDot[0:2]
        Omega = X[9:12]

        E = self.EulerRateMatrix(X[6:9])
        Edot = np.array([
            [0.0, 0.0, -np.cos(Theta) * ThetaDot],
            [0.0, -np.sin(Phi) * PhiDot,
             -np.sin(Theta) * ThetaDot * np.sin(Phi) + np.cos(Theta) * np.cos(Phi) * PhiDot],
            [0.0, -np.cos(Phi) * PhiDot,
             -np.sin(Theta) * ThetaDot * np.cos(Phi) - np.cos(Theta) * np.sin(Phi) * PhiDot]
        ])

        OmegaDot = E @ EulerAcceleration + Edot @ EulerDot
        return self.I @ OmegaDot + np.cross(Omega, self.I @ Omega)

    def LimitControl(self, U):
        # Static actuator limits followed by nonnegative rotor thrust allocation.
        U = np.clip(U, self.MinInput, self.MaxInput)
        RotorThrusts = np.maximum(self.InvMixingMatrix @ U, 0.0)
        return self.MixingMatrix @ RotorThrusts

    def ResetController(self):
        self.PositionSurface = np.zeros(3)
        self.AngularSurface = np.zeros(3)
        self.DesiredEuler = np.zeros(3)

    def ControlInput(self, X, TimeValue):
        Xref = self.Reference(TimeValue)
        PositionError = Xref[0:3] - X[0:3]
        VelocityError = Xref[3:6] - X[3:6]

        # Outer sliding surface: velocity error + Lambda * position error
        self.PositionSurface = VelocityError + self.PositionLambda * PositionError
        PositionSwitching = np.clip(self.PositionSurface / self.PositionBoundary, -1.0, 1.0)
        Acceleration = (
            self.ReferenceAcceleration(TimeValue)
            + self.PositionLambda * VelocityError
            + self.PositionSwitch * PositionSwitching
        )
        Thrust, self.DesiredEuler = self.DesiredAttitude(Acceleration, Xref[8])

        # Inner sliding surface: Euler-rate error + Lambda * Euler-angle error
        AngularError = self.WrapAngle(self.DesiredEuler - X[6:9])
        DesiredEulerRate = np.array([0.0, 0.0, Xref[11]])
        RateError = DesiredEulerRate - self.EulerRates(X)
        self.AngularSurface = RateError + self.AngularLambda * AngularError
        AngularSwitching = np.clip(self.AngularSurface / self.AngularBoundary, -1.0, 1.0)
        EulerAcceleration = (
            np.array([0.0, 0.0, -np.sin(TimeValue)])
            + self.AngularLambda * RateError
            + self.AngularSwitch * AngularSwitching
        )

        Torque = self.EulerAccelerationToTorque(X, EulerAcceleration)
        return self.LimitControl(np.concatenate(([Thrust], Torque)))

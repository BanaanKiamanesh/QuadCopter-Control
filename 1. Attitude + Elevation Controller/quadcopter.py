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

        self.AngularKp = np.array([0.115, 0.115, 0.115])
        self.AngularKd = np.array([0.023, 0.023, 0.042])
        self.AltitudeKp = 5.0
        self.AltitudeKd = 4.2

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

    def WrapAngle(self, Angle):
        return (Angle + np.pi) % (2.0 * np.pi) - np.pi

    def Reference(self, TimeValue):
        Xref = np.zeros(self.SystemOrder)
        Xref[2] = 2.5 + 0.25 * np.sin(0.6 * TimeValue)
        Xref[5] = 0.15 * np.cos(0.6 * TimeValue)
        Xref[6:9] = [0.04 * np.sin(1.3 * TimeValue),
                     0.04 * np.sin(1.1 * TimeValue),
                     0.25 * np.sin(0.45 * TimeValue)]
        return Xref

    def ReferenceRates(self, TimeValue):
        return np.array([0.052 * np.cos(1.3 * TimeValue),
                         0.044 * np.cos(1.1 * TimeValue),
                         0.1125 * np.cos(0.45 * TimeValue)])

    def ControlInput(self, X, TimeValue):
        Xref = self.Reference(TimeValue)
        Error = self.WrapAngle(Xref[6:9] - X[6:9])
        RateError = self.ReferenceRates(TimeValue) - self.EulerRates(X)

        # Independent attitude PD loops.
        Torque = self.AngularKp * Error + self.AngularKd * RateError

        # Gravity and attitude compensation for elevation.
        ZAcceleration = (-0.09 * np.sin(0.6 * TimeValue)
                         + self.AltitudeKp * (Xref[2] - X[2])
                         + self.AltitudeKd * (Xref[5] - X[5]))
        Phi, Theta = X[6:8]
        Thrust = self.Mass * (self.g + ZAcceleration) / (np.cos(Phi) * np.cos(Theta))
        return self.LimitControl(np.r_[Thrust, Torque])

    def LimitControl(self, U):
        # Static actuator limits followed by nonnegative rotor thrust allocation.
        U = np.clip(U, self.MinInput, self.MaxInput)
        RotorThrusts = np.maximum(self.InvMixingMatrix @ U, 0.0)
        return self.MixingMatrix @ RotorThrusts


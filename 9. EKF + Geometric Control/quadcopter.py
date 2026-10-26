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

        # Nominal model used by the EKF and the controller, with parameter errors
        self.ModelMass = 0.440
        self.ModelI = np.diag([0.0021, 0.0025, 0.0042])
        self.ModelInvI = np.linalg.inv(self.ModelI)

        # Geometric Controller Properties
        # Translational gains on position, velocity and position integral errors
        self.PositionGain = np.array([8.0, 8.0, 8.0])
        self.VelocityGain = np.array([2.0, 2.0, 2.0])
        self.IntegralGain = np.array([5.0, 5.0, 5.0])
        # Attitude gains on the SO(3) errors e_R and e_Omega
        self.AttitudeGain = np.array([0.3, 0.3, 0.6])
        self.RateGain = np.array([0.03, 0.03, 0.06])
        self.IntegralLimit = 0.5
        self.MaxTilt = np.deg2rad(30.0)

        # EKF Properties
        # Measured outputs: [x, y, z, phi, theta, psi], sampled every IntegrationDt
        self.MeasurementIndex = np.array([0, 1, 2, 6, 7, 8])
        self.MeasurementNoise = np.array([1e-3, 1e-3, 1e-3,
                                          np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1)])
        # Process noise densities cover the model errors
        self.ProcessNoise = np.array([1e-4, 1e-4, 1e-4,
                                      0.1, 0.1, 0.1,
                                      1e-4, 1e-4, 1e-4,
                                      0.1, 0.1, 0.1])
        self.InitialCovariance = np.array([0.1, 0.1, 0.1,
                                           0.2, 0.2, 0.2,
                                           0.05, 0.05, 0.05,
                                           0.1, 0.1, 0.1])

        self.ResetController()
        self.ResetObserver()

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

    def EulerRateMatrixDot(self, Euler, EulerDot):
        Phi, Theta = Euler[0:2]
        PhiDot, ThetaDot = EulerDot[0:2]
        return np.array([
            [0.0, 0.0, -np.cos(Theta) * ThetaDot],
            [0.0, -np.sin(Phi) * PhiDot,
             -np.sin(Theta) * ThetaDot * np.sin(Phi) + np.cos(Theta) * np.cos(Phi) * PhiDot],
            [0.0, -np.cos(Phi) * PhiDot,
             -np.sin(Theta) * ThetaDot * np.cos(Phi) - np.cos(Theta) * np.sin(Phi) * PhiDot]
        ])

    def EulerRates(self, X):
        return np.linalg.solve(self.EulerRateMatrix(X[6:9]), X[9:12])

    def Dynamics(self, X, U, Mass, I, InvI):
        # States: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        T = U[0]
        Tau = U[1:4]
        Orient = X[6:9]
        Omega = X[9:12]

        dX = np.zeros(self.SystemOrder)
        dX[0:3] = X[3:6]
        dX[3:6] = np.array([0.0, 0.0, -self.g]) + self.RotationMatrix(Orient)[:, 2] * (T / Mass)
        dX[6:9] = self.EulerRates(X)
        dX[9:12] = InvI @ (Tau - np.cross(Omega, I @ Omega))

        return dX

    def SysDyn(self, X, U):
        return self.Dynamics(X, U, self.Mass, self.I, self.InvI)

    def ModelDyn(self, X, U):
        return self.Dynamics(X, U, self.ModelMass, self.ModelI, self.ModelInvI)

    def Step(self, X, U, Dt):
        # Fourth-order Runge-Kutta integration
        K1 = self.SysDyn(X, U)
        K2 = self.SysDyn(X + 0.5 * Dt * K1, U)
        K3 = self.SysDyn(X + 0.5 * Dt * K2, U)
        K4 = self.SysDyn(X + Dt * K3, U)
        return X + Dt * (K1 + 2.0 * K2 + 2.0 * K3 + K4) / 6.0

    def ModelStep(self, X, U, Dt):
        K1 = self.ModelDyn(X, U)
        K2 = self.ModelDyn(X + 0.5 * Dt * K1, U)
        K3 = self.ModelDyn(X + 0.5 * Dt * K2, U)
        K4 = self.ModelDyn(X + Dt * K3, U)
        return X + Dt * (K1 + 2.0 * K2 + 2.0 * K3 + K4) / 6.0

    def WrapAngle(self, Angle):
        return (Angle + np.pi) % (2.0 * np.pi) - np.pi

    def LimitControl(self, U):
        # Static actuator limits followed by nonnegative rotor thrust allocation.
        U = np.clip(U, self.MinInput, self.MaxInput)
        RotorThrusts = np.maximum(self.InvMixingMatrix @ U, 0.0)
        return self.MixingMatrix @ RotorThrusts

    def FlatAttitude(self, TimeValue):
        # Thrust direction and Euler angles that realise the reference acceleration and yaw
        Acceleration = np.array([-np.cos(TimeValue), -4.0 * np.sin(2.0 * TimeValue), -np.sin(TimeValue)])
        Yaw = np.sin(TimeValue)
        Force = Acceleration + np.array([0.0, 0.0, self.g])

        Forward = np.cos(Yaw) * Force[0] + np.sin(Yaw) * Force[1]
        Sideways = np.sin(Yaw) * Force[0] - np.cos(Yaw) * Force[1]
        Phi = np.arctan2(Sideways, np.hypot(Forward, Force[2]))
        Theta = np.arctan2(Forward, Force[2])
        return np.array([Phi, Theta, Yaw]), np.linalg.norm(Force)

    def FlatAttitudeDerivatives(self, TimeValue):
        # Central differences of the closed-form attitude map
        H = 1e-3
        Before = self.FlatAttitude(TimeValue - H)[0]
        Euler, ForceNorm = self.FlatAttitude(TimeValue)
        After = self.FlatAttitude(TimeValue + H)[0]
        EulerDot = (After - Before) / (2.0 * H)
        EulerDDot = (After - 2.0 * Euler + Before) / H ** 2
        return Euler, EulerDot, EulerDDot, ForceNorm

    def Reference(self, TimeValue):
        # Full reference state from the flat outputs [x, y, z, psi]
        Euler, EulerDot, _, _ = self.FlatAttitudeDerivatives(TimeValue)
        Xref = np.zeros(self.SystemOrder)
        Xref[0:3] = np.array([np.cos(TimeValue), np.sin(2.0 * TimeValue), 2.5 + np.sin(TimeValue)])
        Xref[3:6] = np.array([-np.sin(TimeValue), 2.0 * np.cos(2.0 * TimeValue), np.cos(TimeValue)])
        Xref[6:9] = Euler
        Xref[9:12] = self.EulerRateMatrix(Euler) @ EulerDot
        return Xref

    def Vee(self, Matrix):
        return np.array([Matrix[2, 1], Matrix[0, 2], Matrix[1, 0]])

    def Hat(self, Vector):
        X, Y, Z = Vector
        return np.array([[0.0, -Z, Y], [Z, 0.0, -X], [-Y, X, 0.0]])

    def ReferenceRates(self, TimeValue):
        # Body rate and body acceleration of the flat reference attitude
        Euler, EulerDot, EulerDDot, _ = self.FlatAttitudeDerivatives(TimeValue)
        E = self.EulerRateMatrix(Euler)
        Omega = E @ EulerDot
        OmegaDot = E @ EulerDDot + self.EulerRateMatrixDot(Euler, EulerDot) @ EulerDot
        return Omega, OmegaDot

    def ResetController(self):
        self.PositionIntegral = np.zeros(3)

    def ControlInput(self, X, TimeValue):
        Xref = self.Reference(TimeValue)
        AccelerationRef = np.array([-np.cos(TimeValue), -4.0 * np.sin(2.0 * TimeValue), -np.sin(TimeValue)])
        R = self.RotationMatrix(X[6:9])
        Omega = X[9:12]

        # Desired force in the world frame
        PositionError = X[0:3] - Xref[0:3]
        VelocityError = X[3:6] - Xref[3:6]
        Force = (-self.PositionGain * PositionError - self.VelocityGain * VelocityError
                 - self.IntegralGain * self.PositionIntegral
                 + self.ModelMass * (AccelerationRef + np.array([0.0, 0.0, self.g])))
        Force[2] = max(Force[2], 0.1 * self.ModelMass * self.g)
        Horizontal = np.linalg.norm(Force[0:2])
        MaxHorizontal = Force[2] * np.tan(self.MaxTilt)
        if Horizontal > MaxHorizontal:
            Force[0:2] *= MaxHorizontal / Horizontal

        # Thrust is the desired force along the current body z-axis
        Thrust = Force @ R[:, 2]

        # Desired rotation from the force direction and the reference yaw.
        # b1 stays in the vertical plane of the heading, so the ZYX yaw of Rd is the reference yaw.
        B3 = Force / np.linalg.norm(Force)
        HeadingNormal = np.array([-np.sin(Xref[8]), np.cos(Xref[8]), 0.0])
        B1 = np.cross(HeadingNormal, B3)
        B1 /= np.linalg.norm(B1)
        Rd = np.column_stack((B1, np.cross(B3, B1), B3))

        # Attitude errors on SO(3); the flat reference rates are the feedforward
        OmegaRef, OmegaDotRef = self.ReferenceRates(TimeValue)
        AttitudeError = 0.5 * self.Vee(Rd.T @ R - R.T @ Rd)
        RelativeRotation = R.T @ Rd
        RateError = Omega - RelativeRotation @ OmegaRef
        Torque = (-self.AttitudeGain * AttitudeError - self.RateGain * RateError
                  + np.cross(Omega, self.ModelI @ Omega)
                  - self.ModelI @ (self.Hat(Omega) @ RelativeRotation @ OmegaRef
                                   - RelativeRotation @ OmegaDotRef))

        self.PositionIntegral = np.clip(self.PositionIntegral + self.ControlDt * PositionError,
                                        -self.IntegralLimit, self.IntegralLimit)
        return self.LimitControl(np.r_[Thrust, Torque])

    def ResetObserver(self):
        self.Rng = np.random.default_rng(7)
        self.Covariance = np.diag(self.InitialCovariance ** 2)

    def Measure(self, X):
        # Noisy position and Euler angle measurements
        return X[self.MeasurementIndex] + self.MeasurementNoise * self.Rng.standard_normal(6)

    def ModelJacobian(self, X, U):
        Nx = self.SystemOrder
        F = np.zeros((Nx, Nx))
        for i in range(Nx):
            dX = np.zeros(Nx)
            dX[i] = 1e-6
            F[:, i] = (self.ModelDyn(X + dX, U) - self.ModelDyn(X - dX, U)) / 2e-6
        return F

    def ObserverStep(self, X, Y, U, Dt):
        # Extended Kalman filter: correct with the measurement, then predict one step
        H = np.zeros((len(self.MeasurementIndex), self.SystemOrder))
        H[np.arange(len(self.MeasurementIndex)), self.MeasurementIndex] = 1.0
        R = np.diag(self.MeasurementNoise ** 2)
        P = self.Covariance

        Innovation = Y - X[self.MeasurementIndex]
        Innovation[3:6] = self.WrapAngle(Innovation[3:6])
        Gain = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
        X = X + Gain @ Innovation
        Joseph = np.eye(self.SystemOrder) - Gain @ H
        P = Joseph @ P @ Joseph.T + Gain @ R @ Gain.T

        F = np.eye(self.SystemOrder) + Dt * self.ModelJacobian(X, U)
        X = self.ModelStep(X, U, Dt)
        self.Covariance = F @ P @ F.T + np.diag(self.ProcessNoise ** 2) * Dt
        return X

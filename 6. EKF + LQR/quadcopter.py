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

        # Nominal model used by the EKF and the LQR design, with parameter errors
        self.ModelMass = 0.440
        self.ModelI = np.diag([0.0021, 0.0025, 0.0042])
        self.ModelInvI = np.linalg.inv(self.ModelI)

        # LQR Properties
        # Error states: [position, velocity, Euler angles, body rates] plus integrals of [x, y, z, psi]
        self.IntegralIndex = np.array([0, 1, 2, 8])
        self.StateWeight = np.array([100.0, 100.0, 100.0,
                                     6.0, 6.0, 6.0,
                                     25.0, 25.0, 1600.0,
                                     1.0, 1.0, 1.0])
        self.IntegralWeight = np.array([1600.0, 1600.0, 1600.0, 100.0])
        self.InputWeight = np.array([4.0, 1e4, 1e4, 1e4])
        self.IntegralLimit = np.array([0.5, 0.5, 0.5, 0.5])

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

    def ReferenceInput(self, TimeValue):
        # Feedforward thrust and torques of the nominal model along the reference
        Euler, EulerDot, EulerDDot, ForceNorm = self.FlatAttitudeDerivatives(TimeValue)
        E = self.EulerRateMatrix(Euler)
        Omega = E @ EulerDot
        OmegaDot = E @ EulerDDot + self.EulerRateMatrixDot(Euler, EulerDot) @ EulerDot
        Torque = self.ModelI @ OmegaDot + np.cross(Omega, self.ModelI @ Omega)
        return np.r_[self.ModelMass * ForceNorm, Torque]

    def Linearize(self, X, U):
        # Discrete-time Jacobians of one nominal control interval
        Nx = self.SystemOrder
        A = np.zeros((Nx, Nx))
        B = np.zeros((Nx, self.InputOrder))
        for i in range(Nx):
            dX = np.zeros(Nx)
            dX[i] = 1e-5
            A[:, i] = (self.ModelStep(X + dX, U, self.ControlDt) - self.ModelStep(X - dX, U, self.ControlDt)) / 2e-5
        for i in range(self.InputOrder):
            dU = np.zeros(self.InputOrder)
            dU[i] = 1e-5
            B[:, i] = (self.ModelStep(X, U + dU, self.ControlDt) - self.ModelStep(X, U - dU, self.ControlDt)) / 2e-5
        return A, B

    def LinearizeReference(self, SimTime):
        Steps = int(round(SimTime / self.ControlDt)) + 1
        Time = np.arange(Steps) * self.ControlDt
        self.ReferenceStates = np.array([self.Reference(t) for t in Time])
        self.ReferenceInputs = np.array([self.ReferenceInput(t) for t in Time])
        self.LinearModels = [self.Linearize(X, U) for X, U in zip(self.ReferenceStates, self.ReferenceInputs)]

    def DesignController(self):
        # Time-varying LQR on the linearized error dynamics with integral action
        Nx = self.SystemOrder
        Ni = len(self.IntegralIndex)
        Q = np.diag(np.r_[self.StateWeight, self.IntegralWeight])
        R = np.diag(self.InputWeight)

        C = np.zeros((Ni, Nx))
        C[np.arange(Ni), self.IntegralIndex] = 1.0

        def Augment(A, B):
            Aa = np.block([[A, np.zeros((Nx, Ni))], [self.ControlDt * C, np.eye(Ni)]])
            Ba = np.vstack((B, np.zeros((Ni, self.InputOrder))))
            return Aa, Ba

        def RiccatiStep(P, A, B):
            K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
            return Q + A.T @ P @ (A - B @ K), K

        # Terminal cost from the stationary Riccati solution at the final point
        A, B = Augment(*self.LinearModels[-1])
        P = Q.copy()
        for _ in range(3000):
            P, _ = RiccatiStep(P, A, B)

        self.Gains = np.zeros((len(self.LinearModels), self.InputOrder, Nx + Ni))
        for k in range(len(self.LinearModels) - 1, -1, -1):
            A, B = Augment(*self.LinearModels[k])
            P, self.Gains[k] = RiccatiStep(P, A, B)

    def ResetController(self):
        self.Integral = np.zeros(len(self.IntegralIndex))

    def ControlInput(self, X, TimeValue):
        Index = min(int(round(TimeValue / self.ControlDt)), len(self.Gains) - 1)
        Error = X - self.ReferenceStates[Index]
        Error[6:9] = self.WrapAngle(Error[6:9])

        U = self.ReferenceInputs[Index] - self.Gains[Index] @ np.r_[Error, self.Integral]
        self.Integral = np.clip(self.Integral + self.ControlDt * Error[self.IntegralIndex],
                                -self.IntegralLimit, self.IntegralLimit)
        return self.LimitControl(U)

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

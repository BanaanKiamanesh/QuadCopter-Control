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

        Lambda = 1.6
        self.PositionK3 = 4.0 * Lambda
        self.PositionK2 = 6.0 * Lambda ** 2
        self.PositionK1 = 4.0 * Lambda ** 3
        self.PositionK0 = Lambda ** 4
        self.YawKp = 9.0
        self.YawKd = 6.0
        self.MaxThrustAcceleration = 18.0

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

    def WrapAngle(self, Angle):
        return (Angle + np.pi) % (2.0 * np.pi) - np.pi

    def LimitControl(self, U):
        # Static actuator limits followed by nonnegative rotor thrust allocation.
        U = np.clip(U, self.MinInput, self.MaxInput)
        RotorThrusts = np.maximum(self.InvMixingMatrix @ U, 0.0)
        return self.MixingMatrix @ RotorThrusts


    def ReferenceDerivatives(self, TimeValue):
        # Position, velocity, acceleration, jerk, and snap.
        C, S = np.cos, np.sin
        Position = np.array([0.7 * C(0.6 * TimeValue),
                             0.7 * S(0.6 * TimeValue),
                             2.5 + 0.25 * S(0.5 * TimeValue)])
        Velocity = np.array([-0.42 * S(0.6 * TimeValue),
                              0.42 * C(0.6 * TimeValue),
                              0.125 * C(0.5 * TimeValue)])
        Acceleration = np.array([-0.252 * C(0.6 * TimeValue),
                                 -0.252 * S(0.6 * TimeValue),
                                 -0.0625 * S(0.5 * TimeValue)])
        Jerk = np.array([0.1512 * S(0.6 * TimeValue),
                         -0.1512 * C(0.6 * TimeValue),
                         -0.03125 * C(0.5 * TimeValue)])
        Snap = np.array([0.09072 * C(0.6 * TimeValue),
                         0.09072 * S(0.6 * TimeValue),
                         0.015625 * S(0.5 * TimeValue)])
        Yaw = 0.25 * S(0.45 * TimeValue)
        YawRate = 0.1125 * C(0.45 * TimeValue)
        YawAcceleration = -0.050625 * S(0.45 * TimeValue)
        return Position, Velocity, Acceleration, Jerk, Snap, Yaw, YawRate, YawAcceleration

    def Reference(self, TimeValue):
        Position, Velocity, _, _, _, Yaw, YawRate, _ = self.ReferenceDerivatives(TimeValue)
        Xref = np.zeros(self.SystemOrder)
        Xref[0:3] = Position
        Xref[3:6] = Velocity
        Xref[8] = Yaw
        Xref[11] = YawRate
        return Xref

    def CrossMatrix(self, Value):
        X, Y, Z = Value
        return np.array([[0, -Z, Y], [Z, 0, -X], [-Y, X, 0]])

    def ExtendedDynamics(self, X, U):
        # Extended states [original 12 states, thrust, thrust rate].
        T, Tdot = X[12:14]
        Torque = U[1:4]
        PhysicalInput = self.LimitControl(np.r_[T, Torque])
        dX = np.zeros(14)
        dX[0:12] = self.SysDyn(X[0:12], PhysicalInput)
        dX[12] = Tdot
        dX[13] = U[0]
        return dX

    def Step(self, X, U, Dt):
        K1 = self.ExtendedDynamics(X, U)
        K2 = self.ExtendedDynamics(X + 0.5 * Dt * K1, U)
        K3 = self.ExtendedDynamics(X + 0.5 * Dt * K2, U)
        K4 = self.ExtendedDynamics(X + Dt * K3, U)
        Xnew = X + Dt * (K1 + 2.0 * K2 + 2.0 * K3 + K4) / 6.0
        Xnew[12] = np.clip(Xnew[12], 1.5, 8.0)
        Xnew[13] = np.clip(Xnew[13], -5.0, 5.0)
        return Xnew

    def OutputDerivatives(self, X):
        # Compute y=[position,yaw] and its derivatives from the plant.
        Position = X[0:3]
        Velocity = X[3:6]
        Euler = X[6:9]
        Omega = X[9:12]
        T, Tdot = X[12:14]
        R = self.RotationMatrix(Euler)
        E = self.EulerRateMatrix(Euler)
        EulerDot = np.linalg.solve(E, Omega)
        Phi, Theta = Euler[:2]
        PhiDot, ThetaDot = EulerDot[:2]
        Edot = np.array([
            [0.0, 0.0, -np.cos(Theta) * ThetaDot],
            [0.0, -np.sin(Phi) * PhiDot,
             -np.sin(Theta) * ThetaDot * np.sin(Phi)
             + np.cos(Theta) * np.cos(Phi) * PhiDot],
            [0.0, -np.cos(Phi) * PhiDot,
             -np.sin(Theta) * ThetaDot * np.cos(Phi)
             - np.cos(Theta) * np.sin(Phi) * PhiDot]
        ])

        Axis = np.array([0.0, 0.0, 1.0])
        B = R[:, 2]
        Bdot = R @ np.cross(Omega, Axis)
        Acceleration = -self.g * Axis + B * T / self.Mass
        Jerk = (B * Tdot + Bdot * T) / self.Mass

        # p(4) = DriftP + DecouplingP @ [Tddot, tau]; psi(2) likewise.
        Gyro = np.cross(Omega, self.I @ Omega)
        OmegaDotFree = -self.InvI @ Gyro
        BddotFree = R @ (np.cross(Omega, np.cross(Omega, Axis))
                           + np.cross(OmegaDotFree, Axis))
        Btorque = -R @ self.CrossMatrix(Axis) @ self.InvI
        DriftP = (2.0 * Bdot * Tdot + BddotFree * T) / self.Mass
        DriftYaw = np.linalg.solve(E, OmegaDotFree - Edot @ EulerDot)[2]

        Decoupling = np.zeros((4, 4))
        Decoupling[0:3, 0] = B / self.Mass
        Decoupling[0:3, 1:4] = T * Btorque / self.Mass
        Decoupling[3, 1:4] = np.linalg.solve(E, self.InvI)[2]
        Drift = np.r_[DriftP, DriftYaw]
        return Position, Velocity, Acceleration, Jerk, Euler[2], EulerDot[2], Drift, Decoupling

    def InitialState(self):
        # Match reference position, velocity and attitude at t=0.
        Position, Velocity, Acceleration, Jerk, _, Yaw, YawRate, _ = self.ReferenceDerivatives(0.0)
        Force = Acceleration + np.array([0.0, 0.0, self.g])
        ForceNorm = np.linalg.norm(Force)
        Axis = Force / ForceNorm
        # Roll/pitch satisfy R[:,2] = Axis for the specified yaw.
        Phi = np.arcsin(Axis[0] * np.sin(Yaw) - Axis[1] * np.cos(Yaw))
        Theta = np.arctan2(Axis[0] * np.cos(Yaw) + Axis[1] * np.sin(Yaw), Axis[2])
        X = np.zeros(14)
        X[0:3] = Position
        X[3:6] = Velocity
        X[6:9] = [Phi, Theta, Yaw]
        X[12] = self.Mass * ForceNorm
        # Find thrust rate and body rates consistent with reference jerk and yaw rate.
        E = self.EulerRateMatrix(X[6:9])
        R = self.RotationMatrix(X[6:9])
        AxisBody = np.array([0.0, 0.0, 1.0])
        InitialMap = np.zeros((4, 4))
        InitialMap[0:3, 0] = R[:, 2] / self.Mass
        InitialMap[0:3, 1:4] = -X[12] * R @ self.CrossMatrix(AxisBody) / self.Mass
        InitialMap[3, 1:4] = np.linalg.inv(E)[2]
        InitialRates = np.linalg.solve(InitialMap, np.r_[Jerk, YawRate])
        X[13] = InitialRates[0]
        X[9:12] = InitialRates[1:4]
        return X

    def ControlInput(self, X, TimeValue):
        (PosRef, VelRef, AccRef, JerkRef, SnapRef,
         YawRef, YawRateRef, YawAccRef) = self.ReferenceDerivatives(TimeValue)
        Pos, Vel, Acc, Jerk, Yaw, YawRate, Drift, Decoupling = self.OutputDerivatives(X)

        # p(4) and psi(2) are independently assigned closed-loop poles.
        VirtualPosition = (SnapRef + self.PositionK3 * (JerkRef - Jerk)
                           + self.PositionK2 * (AccRef - Acc)
                           + self.PositionK1 * (VelRef - Vel)
                           + self.PositionK0 * (PosRef - Pos))
        YawError = self.WrapAngle(YawRef - Yaw)
        VirtualYaw = (YawAccRef + self.YawKd * (YawRateRef - YawRate)
                      + self.YawKp * YawError)
        Input = np.linalg.solve(Decoupling, np.r_[VirtualPosition, VirtualYaw] - Drift)
        Input[0] = np.clip(Input[0], -self.MaxThrustAcceleration, self.MaxThrustAcceleration)
        Input[1:4] = self.LimitControl(np.r_[X[12], Input[1:4]])[1:4]
        return Input

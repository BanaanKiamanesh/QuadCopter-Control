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

        self.I = np.diag([self.Ixx, self.Iyy, self.Izz])
        self.InvI = np.linalg.inv(self.I)

        # Nominal Prediction Model
        self.NominalMass = 0.440
        self.NominalI = np.diag([0.0020, 0.0026, 0.0041])
        self.NominalInvI = np.linalg.inv(self.NominalI)

        # Observer Properties
        self.SystemOrder = 12
        self.InputOrder = 4
        self.G = 90.0 * np.eye(self.SystemOrder)

        # Neural Network Properties
        self.NumNeurons = 24
        self.VSize = (self.SystemOrder + self.InputOrder, self.NumNeurons)
        self.WSize = (self.SystemOrder, self.NumNeurons)

        self.eta1 = 30.0
        self.rho1 = 0.03
        self.eta2 = 2.0
        self.rho2 = 0.02

        self.InputScale = np.array([
            3.0, 3.0, 4.0,
            3.0, 3.0, 3.0,
            0.7, 0.7, 1.5,
            3.0, 3.0, 3.0,
            7.0, 0.05, 0.05, 0.05
        ])

        rng = np.random.default_rng(123)
        self.W = np.zeros(self.WSize)
        self.V = 0.03 * rng.standard_normal(self.VSize)

        # MPC Properties
        self.ControlHorizon = 20
        self.ControlDt = 0.05
        self.IntegrationDt = 0.005

        self.StateWeight = np.array([
            120.0, 120.0, 220.0,
            18.0, 18.0, 32.0,
            8.0, 8.0, 55.0,
            1.0, 1.0, 5.0
        ])
        self.TerminalWeight = 4.0 * self.StateWeight
        self.InputWeight = np.array([0.12, 4.0, 4.0, 2.5])
        self.InputRateWeight = np.array([0.60, 8.0, 8.0, 6.0])

        self.MinInput = np.array([1.5, -0.040, -0.040, -0.028])
        self.MaxInput = np.array([8.0, 0.040, 0.040, 0.028])
        self.InputRateLimit = np.array([1.0, 0.025, 0.025, 0.012])

        self.StateMin = np.full(self.SystemOrder, -np.inf)
        self.StateMax = np.full(self.SystemOrder, np.inf)

        self.StateMin[2] = 0.50
        self.StateMax[2] = 4.50
        self.StateMin[3:6] = np.array([-2.5, -2.5, -2.0])
        self.StateMax[3:6] = np.array([2.5, 2.5, 2.0])
        self.StateMin[6:8] = np.deg2rad([-30.0, -30.0])
        self.StateMax[6:8] = np.deg2rad([30.0, 30.0])
        self.StateMin[9:12] = np.array([-3.0, -3.0, -2.2])
        self.StateMax[9:12] = np.array([3.0, 3.0, 2.2])

        self.StateConstraintMargin = np.zeros(self.SystemOrder)
        self.StateConstraintMargin[9:12] = np.array([0.25, 0.25, 0.10])

        self.HoverInput = np.array([self.NominalMass * self.g, 0.0, 0.0, 0.0])
        self.PreviousInput = np.array([self.Mass * self.g, 0.0, 0.0, 0.0])
        self.ControlSequence = np.tile(self.PreviousInput, (self.ControlHorizon, 1))

    def RotationMatrix(self, Euler):
        Phi = Euler[0]
        Theta = Euler[1]
        Psi = Euler[2]

        cPhi = np.cos(Phi)
        sPhi = np.sin(Phi)
        cThe = np.cos(Theta)
        sThe = np.sin(Theta)
        cPsi = np.cos(Psi)
        sPsi = np.sin(Psi)

        return np.array([
            [cThe * cPsi, sPhi * sThe * cPsi - cPhi * sPsi, cPhi * sThe * cPsi + sPhi * sPsi],
            [cThe * sPsi, sPhi * sThe * sPsi + cPhi * cPsi, cPhi * sThe * sPsi - sPhi * cPsi],
            [-sThe, cThe * sPhi, cThe * cPhi]
        ])

    def Dynamics(self, X, U, Mass, I, InvI):
        T = U[0]
        Tau = U[1:4]

        Vel = X[3:6]
        Orient = X[6:9]
        Omega = X[9:12]

        dX = np.zeros(self.SystemOrder)
        dX[0:3] = Vel

        R = self.RotationMatrix(Orient)
        dX[3:6] = np.array([0.0, 0.0, -self.g]) + R @ np.array([0.0, 0.0, T / Mass])

        Phi = Orient[0]
        Theta = Orient[1]

        E = np.array([
            [1.0, 0.0, -np.sin(Theta)],
            [0.0, np.cos(Phi), np.cos(Theta) * np.sin(Phi)],
            [0.0, -np.sin(Phi), np.cos(Theta) * np.cos(Phi)]
        ])

        dX[6:9] = np.linalg.solve(E, Omega)
        dX[9:12] = InvI @ (Tau - np.cross(Omega, I @ Omega))

        return dX

    def SysDyn(self, X, U):
        return self.Dynamics(X, U, self.Mass, self.I, self.InvI)

    def NominalDyn(self, X, U):
        return self.Dynamics(X, U, self.NominalMass, self.NominalI, self.NominalInvI)

    def NeuralModel(self, X, U):
        Xbar = np.concatenate((X, U)) / self.InputScale
        Sigma = np.tanh(self.V.T @ Xbar)
        return self.W @ Sigma

    def PredictionModel(self, X, U):
        return self.NominalDyn(X, U) + self.NeuralModel(X, U)

    def Observer(self, X_hat, Xerr, U):
        return self.PredictionModel(X_hat, U) + self.G @ Xerr

    def LearningRule(self, X_hat, Xerr, U):
        Xbar = np.concatenate((X_hat, U)) / self.InputScale
        Sigma = np.tanh(self.V.T @ Xbar)

        # Unexplained model error estimated from the observer: when the
        # estimation error settles, f(X) - PredictionModel(X_hat) = G @ Xerr.
        ModelError = self.G @ Xerr

        dW = self.eta1 * ModelError.reshape(-1, 1) @ Sigma.reshape(1, -1) \
            - self.rho1 * self.W

        BackError = (self.W.T @ ModelError) * (1.0 - Sigma ** 2)
        dV = self.eta2 * Xbar.reshape(-1, 1) @ BackError.reshape(1, -1) \
            - self.rho2 * self.V

        return dW, dV

    def Step(self, X, X_hat, U, dt):
        Xerr = X - X_hat

        dX = self.SysDyn(X, U)
        dX_hat = self.Observer(X_hat, Xerr, U)
        dW, dV = self.LearningRule(X_hat, Xerr, U)

        X = X + dt * dX
        X_hat = X_hat + dt * dX_hat
        self.W = self.W + dt * dW
        self.V = self.V + dt * dV

        return X, X_hat

    def ResetController(self):
        self.PreviousInput = np.array([self.Mass * self.g, 0.0, 0.0, 0.0])
        self.ControlSequence = np.tile(self.PreviousInput, (self.ControlHorizon, 1))

    def Reference(self, t):
        Xref = np.zeros(self.SystemOrder)
        Xref[0:3] = np.array([np.cos(t), np.sin(2.0 * t), 2.5 + np.sin(t)])
        Xref[3:6] = np.array([-np.sin(t), 2.0 * np.cos(2.0 * t), np.cos(t)])
        Xref[8] = np.sin(t)
        Xref[11] = np.cos(t)
        return Xref

    def PredictionLinearization(self, X, U):
        f0 = self.PredictionModel(X, U)
        Ac = np.zeros((self.SystemOrder, self.SystemOrder))
        Bc = np.zeros((self.SystemOrder, self.InputOrder))

        Xstep = 1e-5
        Ustep = 1e-5

        for i in range(self.SystemOrder):
            dX = np.zeros(self.SystemOrder)
            dX[i] = Xstep
            Ac[:, i] = (self.PredictionModel(X + dX, U) - self.PredictionModel(X - dX, U)) / (2.0 * Xstep)

        for i in range(self.InputOrder):
            dU = np.zeros(self.InputOrder)
            dU[i] = Ustep
            Bc[:, i] = (self.PredictionModel(X, U + dU) - self.PredictionModel(X, U - dU)) / (2.0 * Ustep)

        Ad = np.eye(self.SystemOrder) + self.ControlDt * Ac
        Bd = self.ControlDt * Bc
        cd = self.ControlDt * (f0 - Ac @ X - Bc @ U)

        return Ad, Bd, cd


    def PredictionMatrices(self, X0, Ad, Bd, cd):
        Nx = self.SystemOrder
        Nu = self.InputOrder
        N = self.ControlHorizon

        StateMap = np.zeros((N * Nx, N * Nu))
        StateOffset = np.zeros(N * Nx)

        XMap = np.zeros((Nx, N * Nu))
        XOffset = X0.copy()

        for k in range(N):
            XMap = Ad @ XMap
            XMap[:, k * Nu:(k + 1) * Nu] += Bd
            XOffset = Ad @ XOffset + cd

            StateMap[k * Nx:(k + 1) * Nx] = XMap
            StateOffset[k * Nx:(k + 1) * Nx] = XOffset

        return StateMap, StateOffset

    def QPProblem(self, X0, t0, Ad, Bd, cd):
        Nx = self.SystemOrder
        Nu = self.InputOrder
        N = self.ControlHorizon

        StateMap, StateOffset = self.PredictionMatrices(X0, Ad, Bd, cd)

        Reference = np.zeros(N * Nx)
        for k in range(N):
            Reference[k * Nx:(k + 1) * Nx] = self.Reference(t0 + (k + 1) * self.ControlDt)

        Qdiag = np.tile(self.StateWeight, N)
        Qdiag[-Nx:] += self.TerminalWeight
        Rdiag = np.tile(self.InputWeight, N)
        Sdiag = np.tile(self.InputRateWeight, N)

        D = np.zeros((N * Nu, N * Nu))
        for k in range(N):
            D[k * Nu:(k + 1) * Nu, k * Nu:(k + 1) * Nu] = np.eye(Nu)
            if k > 0:
                D[k * Nu:(k + 1) * Nu, (k - 1) * Nu:k * Nu] = -np.eye(Nu)

        Uhover = np.tile(self.HoverInput, N)
        DUref = np.zeros(N * Nu)
        DUref[0:Nu] = self.PreviousInput

        WeightedStateMap = Qdiag[:, None] * StateMap
        H = 2.0 * (
            StateMap.T @ WeightedStateMap
            + np.diag(Rdiag)
            + D.T @ (Sdiag[:, None] * D)
        )

        StateError = StateOffset - Reference
        f = 2.0 * (
            StateMap.T @ (Qdiag * StateError)
            - Rdiag * Uhover
            - D.T @ (Sdiag * DUref)
        )

        InputA = np.eye(N * Nu)
        InputLower = np.tile(self.MinInput, N)
        InputUpper = np.tile(self.MaxInput, N)

        RateA = D
        RateLower = np.tile(-self.InputRateLimit, N)
        RateUpper = np.tile(self.InputRateLimit, N)
        RateLower[0:Nu] += self.PreviousInput
        RateUpper[0:Nu] += self.PreviousInput

        FiniteState = np.where(np.isfinite(self.StateMin) | np.isfinite(self.StateMax))[0]
        StateRows = []
        StateLower = []
        StateUpper = []

        for k in range(N):
            for i in FiniteState:
                RowIndex = k * Nx + i
                StateRows.append(StateMap[RowIndex])
                StateLower.append(self.StateMin[i] + self.StateConstraintMargin[i] - StateOffset[RowIndex])
                StateUpper.append(self.StateMax[i] - self.StateConstraintMargin[i] - StateOffset[RowIndex])

        StateA = np.asarray(StateRows)
        A = np.vstack((InputA, RateA, StateA))
        Lower = np.concatenate((InputLower, RateLower, StateLower))
        Upper = np.concatenate((InputUpper, RateUpper, StateUpper))

        return H, f, A, Lower, Upper

    def SolveQP(self, H, f, A, Lower, Upper):
        N = self.ControlHorizon
        Scale = np.tile(np.array([5.0, 0.04, 0.04, 0.03]), N)

        Hn = Scale[:, None] * H * Scale[None, :]
        fn = Scale * f
        An = A * Scale[None, :]

        RowScale = np.linalg.norm(An, axis=1)
        RowScale[RowScale < 1e-12] = 1.0
        An = An / RowScale[:, None]
        Lowern = Lower / RowScale
        Uppern = Upper / RowScale

        Rho = 1000.0
        MaxIter = 300

        K = Hn + Rho * (An.T @ An) + 1e-7 * np.eye(Hn.shape[0])
        L = np.linalg.cholesky(K)

        V = self.ControlSequence.ravel() / Scale
        Z = np.clip(An @ V, Lowern, Uppern)
        Y = np.zeros_like(Z)

        for _ in range(MaxIter):
            RHS = -fn + Rho * An.T @ (Z - Y)
            V = np.linalg.solve(L.T, np.linalg.solve(L, RHS))

            AV = An @ V
            Zold = Z.copy()
            Z = np.clip(AV + Y, Lowern, Uppern)
            Y = Y + AV - Z

            Primal = np.max(np.abs(AV - Z))
            Dual = Rho * np.max(np.abs(An.T @ (Z - Zold)))

            if Primal < 1e-5 and Dual < 1e-5:
                break

        return Scale * V

    def ControlInput(self, X_hat, t):
        Ad, Bd, cd = self.PredictionLinearization(X_hat, self.PreviousInput)
        H, f, A, Lower, Upper = self.QPProblem(X_hat, t, Ad, Bd, cd)

        Solution = self.SolveQP(H, f, A, Lower, Upper)

        if np.all(np.isfinite(Solution)):
            self.ControlSequence = Solution.reshape(self.ControlHorizon, self.InputOrder)

        U = self.ControlSequence[0].copy()
        U = np.minimum(np.maximum(U, self.MinInput), self.MaxInput)
        U = np.minimum(np.maximum(U, self.PreviousInput - self.InputRateLimit), self.PreviousInput + self.InputRateLimit)

        self.PreviousInput = U.copy()
        self.ControlSequence[:-1] = self.ControlSequence[1:]
        self.ControlSequence[-1] = self.ControlSequence[-2]

        return U


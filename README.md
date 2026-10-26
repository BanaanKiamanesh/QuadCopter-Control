# QuadCopter-Control

Eight independent demonstrations of nonlinear quadcopter control:

1. [Attitude + Elevation Controller](1.%20Attitude%20%2B%20Elevation%20Controller/)
2. [Yaw Angle + Position Controller](2.%20Yaw%20Angle%20%2B%20Position%20Controller/)
3. [Output Feedback Control with Feedback Linearization](3.%20Output%20Feedback%20Control%20with%20Feedback%20Linearization/)
4. [Sliding-Mode Controller + Observer](4.%20Sliding-Mode%20Controller%20%2B%20Observer/)
5. [EKF + NN-Adaptive MPC](5.%20EKF%20%2B%20NN-Adaptive%20MPC/)
6. [EKF + LQR](6.%20EKF%20%2B%20LQR/)
7. [EKF + Backstepping](7.%20EKF%20%2B%20Backstepping/)
8. [EKF + Geometric Control](8.%20EKF%20%2B%20Geometric%20Control/)

Demos 1–3 control from the true state. Demo 4 measures only noisy position and Euler angles and estimates velocities and body rates with a sliding-mode observer. Demos 5–8 use the same sensors as demo 4 and an extended Kalman filter. Demo 5 runs a constrained MPC whose prediction model is corrected online by a neural network that learns from the filter's corrections. Demo 6 feeds a time-varying LQR, demo 7 an integral backstepping controller, and demo 8 a geometric tracking controller on SE(3).

All designs are evaluated on nonlinear rigid-body dynamics:

```math
\begin{aligned}
m\ddot{\mathbf p} &= T R(\boldsymbol\eta)\mathbf e_3 - mg\mathbf e_3, \\
\mathbf I\dot{\boldsymbol\omega} &= \boldsymbol\tau - \boldsymbol\omega\times(\mathbf I\boldsymbol\omega), \\
\boldsymbol\omega &= E(\boldsymbol\eta)\dot{\boldsymbol\eta}.
\end{aligned}
```

Here $`\boldsymbol\eta=[\phi,\theta,\psi]^\top`$ and $`\mathbf u=[T,\tau_\phi,\tau_\theta,\tau_\psi]^\top`$.

```math
m=0.468\ \mathrm{kg},\qquad
\mathbf I=\mathrm{diag}(0.0023,0.0023,0.0046)\ \mathrm{kg\,m^2},\qquad
g=9.81\ \mathrm{m/s^2}.
```

## Running

Install the dependencies, then run a demo from inside its own directory:

```bash
pip install -r requirements.txt
cd "2. Yaw Angle + Position Controller"
python main.py
```

Each run prints tracking metrics, opens the plots, and rewrites the files in that demo's `results/` folder. Set `MPLBACKEND=Agg` to run without opening windows.

## Results

| Demo | Position RMSE | Yaw RMSE |
|---|---|---|
| 1. Attitude + Elevation | 0.046 m (elevation only) | 0.81° |
| 2. Yaw + Position | 0.019 m | 0.022 rad |
| 3. Feedback Linearization | 0.038 m | 0.010 rad |
| 4. Sliding-Mode Controller + Observer | 0.032 m | 0.011 rad |
| 5. EKF + NN-Adaptive MPC | 0.040 m | 0.019 rad |
| 6. EKF + LQR | 0.013 m | 0.013 rad |
| 7. EKF + Backstepping | 0.021 m | 0.012 rad |
| 8. EKF + Geometric Control | 0.020 m | 0.012 rad |

All demos measure error over the whole 10 s run. Each demo's README has its plots and animation.

## License

[MIT](LICENSE)

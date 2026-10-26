# QuadCopter-Control

Five independent demonstrations of nonlinear quadcopter control:

1. [Attitude + Elevation Controller](1.%20Attitude%20%2B%20Elevation%20Controller/)
2. [Yaw Angle + Position Controller](2.%20Yaw%20Angle%20%2B%20Position%20Controller/)
3. [Output Feedback Control with Feedback Linearization](3.%20Output%20Feedback%20Control%20with%20Feedback%20Linearization/)
4. [Sliding-Mode Controller + Observer](4.%20Sliding-Mode%20Controller%20%2B%20Observer/)
5. [NN-Adaptive](5.%20NN-Adaptive/)

Demos 1–3 control from the true state. Demo 4 measures only noisy position and Euler angles and estimates velocities and body rates with a sliding-mode observer. Demo 5 runs its MPC on the estimate from a neural-network-augmented observer.

All designs are evaluated on nonlinear rigid-body dynamics:

$$
\begin{aligned}
m\ddot{\mathbf p} &= T R(\boldsymbol\eta)\mathbf e_3 - mg\mathbf e_3, \\
\mathbf I\dot{\boldsymbol\omega} &= \boldsymbol\tau - \boldsymbol\omega\times(\mathbf I\boldsymbol\omega), \\
\boldsymbol\omega &= E(\boldsymbol\eta)\dot{\boldsymbol\eta}.
\end{aligned}
$$

Here $\boldsymbol\eta=[\phi,\theta,\psi]^\top$ and $\mathbf u=[T,\tau_\phi,\tau_\theta,\tau_\psi]^\top$.

$$
m=0.468\ \mathrm{kg},\qquad
\mathbf I=\operatorname{diag}(0.0023,0.0023,0.0046)\ \mathrm{kg\,m^2},\qquad
g=9.81\ \mathrm{m/s^2}.
$$

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
| 5. NN-Adaptive MPC | 0.035 m | 0.010 rad |

Demos 1–4 measure error over the whole 10 s run. Demo 5 measures it from $t=2$ s onward, after the observer and network have settled. Each demo's README has its plots and animation.

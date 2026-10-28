# QuadCopter-Control

Nine independent demonstrations of nonlinear quadcopter control:

1. [Attitude + Elevation Controller](1.%20Attitude%20%2B%20Elevation%20Controller/)
2. [Yaw Angle + Position Controller](2.%20Yaw%20Angle%20%2B%20Position%20Controller/)
3. [Output Feedback Control with Feedback Linearization](3.%20Output%20Feedback%20Control%20with%20Feedback%20Linearization/)
4. [Sliding-Mode Controller + Observer](4.%20Sliding-Mode%20Controller%20%2B%20Observer/)
5. [EKF + MPC](5.%20EKF%20%2B%20MPC/)
6. [EKF + NN-Adaptive MPC](6.%20EKF%20%2B%20NN-Adaptive%20MPC/)
7. [EKF + LQR](7.%20EKF%20%2B%20LQR/)
8. [EKF + Backstepping](8.%20EKF%20%2B%20Backstepping/)
9. [EKF + Geometric Control](9.%20EKF%20%2B%20Geometric%20Control/)

Demos 1–3 control from the true state. Demo 4 measures only noisy position and Euler angles and estimates velocities and body rates with a sliding-mode observer. Demos 5–9 use the same sensors as demo 4 and an extended Kalman filter. Demo 5 runs a constrained offset-free MPC, with the filter also estimating a constant acceleration disturbance. Demo 6 runs the same MPC, but its prediction model is corrected online by a neural network that learns from the filter's corrections. Demo 7 feeds a time-varying LQR, demo 8 an integral backstepping controller, and demo 9 a geometric tracking controller on SE(3).

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

Each run prints tracking metrics, opens the plots, and rewrites the files in that demo's `results/` folder. That includes `simulation_data.npz`, which holds the time, true state, reference, inputs and, for demos 4–9, the state estimate. Set `MPLBACKEND=Agg` to run without opening windows.

## Results

| Demo | Position RMSE | Yaw RMSE |
|---|---|---|
| 1. Attitude + Elevation | 0.046 m (elevation only) | 0.81° |
| 2. Yaw + Position | 0.019 m | 0.022 rad |
| 3. Feedback Linearization | 0.024 m | 0.009 rad |
| 4. Sliding-Mode Controller + Observer | 0.032 m | 0.011 rad |
| 5. EKF + MPC | 0.026 m | 0.019 rad |
| 6. EKF + NN-Adaptive MPC | 0.040 m | 0.019 rad |
| 7. EKF + LQR | 0.013 m | 0.013 rad |
| 8. EKF + Backstepping | 0.021 m | 0.012 rad |
| 9. EKF + Geometric Control | 0.020 m | 0.012 rad |

All demos measure error over the whole 10 s run. Each demo's README has its plots and animation.

## Comparison

[`comparison.ipynb`](comparison.ipynb) loads the saved data of all nine demos and compares them on the same metrics: tracking error, error over time, flight paths, control effort and state estimation. It ends with a ranking of the observer-based demos 4–9, which share sensors, model error and trajectory. The notebook is saved with its outputs, so it can be read on GitHub without running it. To rerun it after regenerating the demos:

```bash
pip install notebook
jupyter notebook comparison.ipynb
```

## License

[MIT](LICENSE)

## References

Quadrotor modelling and flatness (all demos; flat reference in demos 7–9):

1. R. Mahony, V. Kumar and P. Corke, "Multirotor aerial vehicles: Modeling, estimation, and control of quadrotor," *IEEE Robotics & Automation Magazine*, vol. 19, no. 3, pp. 20–32, 2012. [doi:10.1109/MRA.2012.2206474](https://doi.org/10.1109/MRA.2012.2206474)
2. T. Luukkonen, "Modelling and control of quadcopter," independent research project in applied mathematics, Aalto University, 2011.
3. M. Fliess, J. Lévine, P. Martin and P. Rouchon, "Flatness and defect of non-linear systems: Introductory theory and examples," *International Journal of Control*, vol. 61, no. 6, pp. 1327–1361, 1995. [doi:10.1080/00207179508921959](https://doi.org/10.1080/00207179508921959)
4. D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," in *Proc. IEEE International Conference on Robotics and Automation (ICRA)*, 2011, pp. 2520–2525. [doi:10.1109/ICRA.2011.5980409](https://doi.org/10.1109/ICRA.2011.5980409)

Nonlinear control (demos 3, 4 and 8):

5. A. Isidori, *Nonlinear Control Systems*, 3rd ed. London: Springer, 1995.
6. H. K. Khalil, *Nonlinear Systems*, 3rd ed. Upper Saddle River, NJ: Prentice Hall, 2002.
7. J.-J. E. Slotine and W. Li, *Applied Nonlinear Control*. Englewood Cliffs, NJ: Prentice Hall, 1991.
8. J.-J. E. Slotine, J. K. Hedrick and E. A. Misawa, "On sliding observers for nonlinear systems," *Journal of Dynamic Systems, Measurement, and Control*, vol. 109, no. 3, pp. 245–252, 1987. [doi:10.1115/1.3143852](https://doi.org/10.1115/1.3143852)
9. S. Bouabdallah and R. Siegwart, "Backstepping and sliding-mode techniques applied to an indoor micro quadrotor," in *Proc. IEEE International Conference on Robotics and Automation (ICRA)*, 2005, pp. 2247–2252. [doi:10.1109/ROBOT.2005.1570447](https://doi.org/10.1109/ROBOT.2005.1570447)
10. M. Krstić, I. Kanellakopoulos and P. Kokotović, *Nonlinear and Adaptive Control Design*. New York: Wiley, 1995.

Geometric control (demo 9):

11. T. Lee, M. Leok and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3)," in *Proc. 49th IEEE Conference on Decision and Control (CDC)*, 2010, pp. 5420–5425. [doi:10.1109/CDC.2010.5717652](https://doi.org/10.1109/CDC.2010.5717652)

State estimation and LQR (demos 5–9):

12. D. Simon, *Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches*. Hoboken, NJ: Wiley, 2006.
13. B. D. O. Anderson and J. B. Moore, *Optimal Control: Linear Quadratic Methods*. Englewood Cliffs, NJ: Prentice Hall, 1990.

Model predictive control and QP solvers (demos 5 and 6):

14. J. B. Rawlings, D. Q. Mayne and M. M. Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed. Madison, WI: Nob Hill Publishing, 2017.
15. G. Pannocchia and J. B. Rawlings, "Disturbance models for offset-free model-predictive control," *AIChE Journal*, vol. 49, no. 2, pp. 426–437, 2003. [doi:10.1002/aic.690490213](https://doi.org/10.1002/aic.690490213)
16. S. Boyd, N. Parikh, E. Chu, B. Peleato and J. Eckstein, "Distributed optimization and statistical learning via the alternating direction method of multipliers," *Foundations and Trends in Machine Learning*, vol. 3, no. 1, pp. 1–122, 2011. [doi:10.1561/2200000016](https://doi.org/10.1561/2200000016)
17. B. Stellato, G. Banjac, P. Goulart, A. Bemporad and S. Boyd, "OSQP: An operator splitting solver for quadratic programs," *Mathematical Programming Computation*, vol. 12, no. 4, pp. 637–672, 2020. [doi:10.1007/s12532-020-00179-2](https://doi.org/10.1007/s12532-020-00179-2)

Neural-network adaptive control (demo 6):

18. H. A. Talebi, F. Abdollahi, R. V. Patel and K. Khorasani, *Neural Network-Based State Estimation of Nonlinear Systems: Application to Fault Detection and Isolation*, Lecture Notes in Control and Information Sciences, vol. 395. New York: Springer, 2010. [doi:10.1007/978-1-4419-1438-5](https://doi.org/10.1007/978-1-4419-1438-5)
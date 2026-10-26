# Output Feedback Control with Feedback Linearization

**Controlled outputs:** $\mathbf y=[x,y,z,\psi]^\top$.

Introduce the dynamic thrust extension $\dot T=s$ and $\dot s=u_T$. The position and yaw input-output dynamics satisfy

$$
\begin{bmatrix}\mathbf p^{(4)}\\\ddot\psi\end{bmatrix}
=\mathbf b(X)+A(X)\begin{bmatrix}u_T\\\boldsymbol\tau\end{bmatrix}.
$$

The virtual inputs are

$$
\begin{aligned}
\mathbf v_p={}&\mathbf p_d^{(4)}+K_3(\mathbf p_d^{(3)}-\mathbf p^{(3)})
+K_2(\ddot{\mathbf p}_d-\ddot{\mathbf p})\\
&+K_1(\dot{\mathbf p}_d-\dot{\mathbf p})+K_0(\mathbf p_d-\mathbf p),\\
v_\psi={}&\ddot\psi_d+k_{D,\psi}(\dot\psi_d-\dot\psi)
+k_{P,\psi}\operatorname{wrap}(\psi_d-\psi).
\end{aligned}
$$

Thus, for invertible $A(X)$,

$$
\begin{bmatrix}u_T\\\boldsymbol\tau\end{bmatrix}
=A(X)^{-1}\!\left(\begin{bmatrix}\mathbf v_p\\v_\psi\end{bmatrix}-\mathbf b(X)\right).
$$

The selected characteristic polynomials are $(s+1.6)^4$ for position and $(s+3)^2$ for yaw. Output derivatives are reconstructed from the full state and model.

## Running

From this directory:

```bash
python main.py
```

This prints the metrics below and rewrites the files in `results/`.

## Results

Over the 10 s simulation:

| Metric | Value |
|---|---|
| Position RMSE ($x$, $y$, $z$) | 0.026, 0.019, 0.021 m |
| Total position RMSE | 0.038 m |
| Yaw RMSE | 0.010 rad |

![Tracking plots](results/tracking_plots.svg)

![3D trajectory](results/trajectory_3d.svg)

![Animation](results/quadcopter_animation.gif)

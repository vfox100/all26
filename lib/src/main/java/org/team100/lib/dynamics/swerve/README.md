# Swerve Dynamics

Dynamics of the swerve drivetrain.

<img src="image_swerve.png" width="300" />

Divide the problem into three pieces:

* Determine the total rigid-body forces and torques for the desired rigid-body accelerations, using $F=ma$ and $\tau=I\alpha$.
* Find the set of drive forces (the "grasp") that sum to the total.
* Project those drive forces into the wheel axes.

See [WRENCH.md](../WRENCH.md) for background.

See [SE2](../se2/README.md) regarding the rigid-body effort.

## Grasp Forces

In the Mecanum drive, the contact points are fixed.  For example:

```math
\mathbf{r_1}
=
\begin{bmatrix}
1 \\
1
\end{bmatrix}
```

```math
\mathbf{r_2}
=
\begin{bmatrix}
1 \\
-1
\end{bmatrix}
```

```math
\mathbf{r_3}
=
\begin{bmatrix}
-1 \\
1
\end{bmatrix}
```

```math
\mathbf{r_4}
=
\begin{bmatrix}
-1 \\
-1
\end{bmatrix}
```

In the swerve drive, the contact directions (wheel angles) are variable.
How should we determine them?

Each wheel is capable of producing force perpendicular to its axle,
parallel to the velocity our kinematics assumes.
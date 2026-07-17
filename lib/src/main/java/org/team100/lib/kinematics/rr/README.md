# RR Kinematics

Revolute - Revolute

A 2-DOF arm, as in 2023.

<img src="../../dynamics/rr/image_rr.png">

Cartesian joint positions are $P_1$ (the elbow) and $P_2$ (the end-effector).

Joint configurations are $q_1$, relative to the x axis,
and $q_2$, measured relative to the proximal link.

Link lengths are $l_1$ and $l_2$.

## Forward kinematics

```math
P_1 = 
\begin{bmatrix}
x_1\\
y_1
\end{bmatrix}
=
\begin{bmatrix}
l_1 cos(q_1)\\
\\
l_1 sin(q_1)
\end{bmatrix}
```
```math
P_2 =
\begin{bmatrix}
x_2\\
y_2
\end{bmatrix}
=
\begin{bmatrix}
x_1 + l_2 cos(q_1 + q_2)\\
\\
y_1 + l_2 sin(q_1 + q_2)
\end{bmatrix}
=
\begin{bmatrix}
l_1 cos(q_1) + l_2 cos(q_1 + q_2)\\
\\
l_1 sin(q_1) + l_2 sin(q_1 + q_2)
\end{bmatrix}
```


## Inverse kinematics

```math
r = \|P2\|
```
```math
\gamma = atan2(y_2, x_2)
```
```math
\beta = acos\left(\frac{r^2 + l_1^2 - l_2^2}{2  r  l_1}\right)
```
```math
\alpha = acos\left(\frac{l_1^2  + l_2^2 - r^2} {2 * l_1 * l_2}\right)
```
```math
q_1 = \gamma + \beta
```

```math
q_2 = \alpha + \pi
```

## Velocity

the function for cartesian velocity, $\dot{x}$, can be written
in terms of the Jacobian, $J$:

```math
\begin{equation}
\dot{x} = J \dot{q}
\end{equation}
```

To obtain the Jacobian, differentiate the forward kinematics:

Using end-effector coordinates $(x,y)$:

```math
J = 
\begin{bmatrix}
\frac{\partial x}{\partial q_1} & \frac{\partial x}{\partial q_2} \\
\\
\frac{\partial y}{\partial q_1} & \frac{\partial y}{\partial q_2} \\
\end{bmatrix}
```

substituting:

```math
J =
\begin{bmatrix}
\frac{\partial (l_1 cos(q_1) + l_2 cos(q_1 + q_2))}{\partial q_1} & \frac{\partial (l_1 cos(q_1) + l_2 cos(q_1 + q_2))}{\partial q_2} \\
\\
\frac{\partial (l_1 sin(q_1) + l_2 sin(q_1 + q_2))}{\partial q_1} & \frac{\partial (l_1 sin(q_1) + l_2 sin(q_1 + q_2))}{\partial q_2} \\
\end{bmatrix}
```

and so:

```math
J =
\begin{bmatrix}
 -l_1 sin(q_1) - l_2 sin(q_1+q_2) & - l_2 sin(q_1+q_2) \\
 \\
l_1 cos(q_1) + l_2 cos(q_1+q_2) & l_2 cos(q_1+q_2) \\
\end{bmatrix}
```


## Acceleration

To find the acceleration, differentiate again and apply the product rule:

```math
\begin{equation}
\ddot{x} = \dot{J}\dot{q} + J\ddot{q}
\end{equation}
```
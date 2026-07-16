# RR Kinematics

Revolute - Revolute

A 2-DOF arm, as in 2023.

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
x_1 + l_2 cos(q_2 + q_1)\\
y_1 + l_2 sin(q_2 + q_1)
\end{bmatrix}
=
\begin{bmatrix}
l_1 cos(q_1) + l_2 cos(q_2 + q_1)\\
l_1 sin(q_1) + l_2 sin(q_2 + q_1)
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

## Jacobian

Differentiate the forward kinematics to get the Jacobian.

TODO: finish this part
# PR Kinematics

Prismatic - Revolute

Elevator - Arm

<img src="../../dynamics/pr/image_pr.png" width=300>

## Forward kinematics

Using end-effector coordinates $(x,y)$
and linkage configuration $(q_1, q_2)$:

```math
\begin{bmatrix}
x \\
y
\end{bmatrix}
=
\begin{bmatrix}
q_1  + l  cos(q_2) \\
\\
l sin(q_2)
\end{bmatrix}        
```

## Inverse kinematics


```math
\begin{bmatrix}
q_1 \\
q_2
\end{bmatrix}
=
\begin{bmatrix}
x -  \sqrt{\displaystyle l^2-y^2} \\
\\
arcsin \left(\displaystyle \frac{y}  {l} \right)
\end{bmatrix}
       

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
\frac{\partial q_1  + l  cos(q_2) }{\partial q_1} &
\frac{\partial q_1  + l  cos(q_2) }{\partial q_2} \\
\\
\frac{\partial l sin(q_2)}{\partial q_1} &
\frac{\partial l sin(q_2)}{\partial q_2} \\
\end{bmatrix}
```

and so:

```math
J =
\begin{bmatrix}
1 &
-l sin(q_2) \\
\\
0 &
l cos(q_2) \\
\end{bmatrix}
```

## Acceleration

To find the acceleration, differentiate again and apply the product rule:

```math
\begin{equation}
\ddot{x} = \dot{J}\dot{q} + J\ddot{q}
\end{equation}
```
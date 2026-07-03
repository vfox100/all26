# SE2 Dynamics

SE2 has three dimensions.

For this system, we define the coordinates:

```math
q =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
```

Dynamics of a rigid body in SE2 can be decomposed into the coordinates.

## Mass

The mass matrix is simply:

```math
M =
\begin{bmatrix}
m & 0 & 0 \\
0 & m & 0 \\
0 & 0 & I \\
\end{bmatrix}
```

## Centrifugal and Coriolis

For this system, there is no centrifugal/coriolis term.

## Gravity

For this system, there is no gravity term.

## Equation of Motion

Now we can write the equation of motion, starting
with the definition:

```math
\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q)
```

we obtain:

```math
\tau = 
\begin{bmatrix}
m & 0 & 0 \\
0 & m & 0 \\
0 & 0 & I \\
\end{bmatrix}
\begin{bmatrix}
\ddot{x} \\
\ddot{y} \\
\ddot{\theta}
\end{bmatrix}
```

which is simply the familiar expressions, $F=ma$ and $\tau = I \alpha$.

The configuration of the system (i.e. $q$) does not appear,
so the implementation has no `SE2Config` record.

The velocity (i.e. $\dot{q}$) doesn't appear, so it isn't in `SE2Dynamics` either.

Also, there's another acceleration class, `AccelerationSE2`,
so there's no separate dynamics record for that.


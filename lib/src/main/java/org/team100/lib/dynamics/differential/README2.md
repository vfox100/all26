# Differential Dynamics Addendum

Following the
[differential drive notes](https://robotics.caltech.edu/~me72/class/DiffDrive2.pdf)
from [CalTech ME72](https://me72.caltech.edu), a somewhat-famous
upper-division lab course.

## Kinematics

This reference begins with the kinematics, using a slightly different formulation
than we use.  For example, wheel rotation is measured positive-counterclockwise, 
looking at the wheel from the outside -- our usual coordinates treat the
"vehicle forward" direction as positive on both sides.  The kinematics are
also extrinsic (field-centric) rather than our usual intrinsics.

Accounting for these differences, the forward kinematics agrees with
the WPI formulation, where $r$ is wheel diameter and $W$ is half the track width:

```math
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix}
=
\frac{1}{2}
r
\begin{bmatrix}
\dot{q_1} +\dot{q_2} \\
0 \\
(\dot{q_2} - \dot{q_1})/W
\end{bmatrix}
```

## Dynamics

The dynamics problem relates rigid-body accelerations to
wheel forces.

Rewriting equation 45 a bit (massless wheels, coordinates, etc):

```math
\ddot{x}
\approx
\frac{1}{mr}
(\tau_1 + \tau_2)
```

We generally represent drive wheels as *linear* forces, to encapsulate
details about gear ratio and wheel size: $\tau = rF$ or
So in those terms, it is simply $F=ma$:

```math
\ddot{x}
\approx
\frac{1}{m}
(F_1 + F_2)
```

Rewriting equation 49 a bit:

```math
\ddot{\theta}
\approx
\frac{W}{rI}
(\tau_2 - \tau_1)
```

With the same force substitution:

```math
\ddot{\theta}
\approx
\frac{W}{I}
(F_2 - F_1)
```

Combining translation and rotation:

```math
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
\frac{1}{m} && \frac{1}{m} \\
\\
\frac{-W}{I} && \frac{W}{I}
\end{bmatrix}
\begin{bmatrix}
F_1 \\
F_2
\end{bmatrix}
```

2x2 matrices are easy to invert, so:

```math
\begin{bmatrix}
F_1 \\
F_2
\end{bmatrix}
=
\frac{mI}{2W}
\begin{bmatrix}
\frac{W}{I} && \frac{-1}{m} \\
\\
\frac{W}{I} && \frac{1}{m}
\end{bmatrix}
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
```
or
```math
\begin{bmatrix}
F_1 \\
F_2
\end{bmatrix}
=
\frac{1}{2}
\begin{bmatrix}
m && \frac{-I}{W} \\
\\
m && \frac{I}{W}
\end{bmatrix}
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
```

Note that these expressions don't depend on
configuration (i.e. value of $x$ or $\theta$, or
velocity (i.e. value of $\dot{x}$ or $\dot{\theta}$), so
there are no "Config" or "Velocity" records.
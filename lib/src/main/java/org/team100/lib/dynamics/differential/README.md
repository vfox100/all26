# Differential Dynamics

The dynamics of a differential drivetrain, aka "Tank Drive."

<img src="image_differential.png" width="300"/>


Divide the problem into two pieces.

* Determine the total rigid-body forces and torques for the desired rigid-body accelerations, using $F=ma$ and $\tau=I\alpha$.
* Find the set of drive forces that sum to the total.

See [WRENCH.md](../WRENCH.md) for background.



The contacts and normals from the diagram are:

```math
\bold{r_1}
=
\begin{bmatrix}
0 \\
1
\end{bmatrix}
```

```math
\bold{r_2}
=
\begin{bmatrix}
0 \\
-1
\end{bmatrix}
```

```math
\bold{n_1}
=
\bold{n_2}
=
\begin{bmatrix}
1 \\
0
\end{bmatrix}
```

So the forward dynamics are:


```math
\begin{bmatrix}
f_x \\
f_y \\
\tau
\end{bmatrix}
=
\begin{bmatrix}
1 && 1  \\
0 && 0  \\
-1 && 1 
\end{bmatrix}
\begin{bmatrix}
f_1 \\
f_2 \\
\end{bmatrix}
```

Using the
[Moore-Penrose pseudoinverse](https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse),
we obtain the inverse dynamics:

```math
\begin{bmatrix}
f_1 \\
f_2 \\
\end{bmatrix}
=
\begin{bmatrix}
0.5 && 0 && -0.5  \\
0.5 && 0 && 0.5  
\end{bmatrix}
\begin{bmatrix}
f_x \\
f_y \\
\tau
\end{bmatrix}
```

Which is the familiar result.
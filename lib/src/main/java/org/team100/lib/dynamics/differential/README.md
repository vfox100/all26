# Differential Dynamics

The dynamics of a differential drivetrain, aka "Tank Drive."

<img src="image_differential.png" width="300"/>


Divide the problem into two pieces.

* Determine the total rigid-body forces and torques for the desired rigid-body accelerations, using $F=ma$ and $\tau=I\alpha$.
* Find the set of drive forces that sum to the total.

See [WRENCH.md](../WRENCH.md) for background.

## Constrained Actuation

The contacts and normals from the diagram are:

```math
\mathbf{r_1}
=
\begin{bmatrix}
0 \\
1
\end{bmatrix}
\tag{1}
```

```math
\mathbf{r_2}
=
\begin{bmatrix}
0 \\
-1
\end{bmatrix}
\tag{2}
```

```math
\mathbf{n_1}
=
\mathbf{n_2}
=
\begin{bmatrix}
1 \\
0
\end{bmatrix}
\tag{3}
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
\tag{4}
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
\tag{5}
```

Which is the familiar result.


## Free Actuation

Using the $G$ matrix for each component force (see WRENCH.md):

```math
\mathbf{w}
=
\begin{bmatrix}
F_x \\
F_y \\
\tau
\end{bmatrix}
=
\begin{bmatrix}
1 && 0 \\
0 && 1 \\
- r_{y} && r_{x} 
\end{bmatrix}
\begin{bmatrix}
f_{x} \\
f_{y} 
\end{bmatrix}
\tag{6}
```

The whole system is


```math
\mathbf{W}
=
\begin{bmatrix}
F_x \\
F_y \\
\tau
\end{bmatrix}
=
\begin{bmatrix}
1 && 0 && 1 && 0 \\
0 && 1 && 0 && 1 \\
-r_{1y} && r_{1x} && -r_{2y} && r_{2x}
\end{bmatrix}
\begin{bmatrix}
f_{1x} \\
f_{1y} \\
f_{2x} \\
f_{2y} 
\end{bmatrix}
\tag{7}
```

Using the example above, $r_1 = (0,1)$ and $r_2 = (0,-1)$:


```math
\mathbf{W}
=
\begin{bmatrix}
F_x \\
F_y \\
\tau
\end{bmatrix}
=
\begin{bmatrix}
1 && 0 && 1 && 0 \\
0 && 1 && 0 && 1 \\
-1 && 0 && 1 && 0
\end{bmatrix}
\begin{bmatrix}
f_{1x} \\
f_{1y} \\
f_{2x} \\
f_{2y} 
\end{bmatrix}
\tag{8}
```

Using the pseudo-inverse:

```math
\begin{bmatrix}
f_{1x} \\
f_{1y} \\
f_{2x} \\
f_{2y} 
\end{bmatrix}
=
\begin{bmatrix}
0.5 && 0 && -0.5  \\
0  && 0.5 && 0 \\
0.5 && 0 && 0.5 \\
0 && 0.5 && 0
\end{bmatrix}
\begin{bmatrix}
f_x \\
f_y \\
\tau
\end{bmatrix}
\tag{9}
```
which is the same result, if $f_y$ is zero.  This would
be the centrifugal component, which is ignored for this drivetrain.
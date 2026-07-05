# Wrench

<img src="differential/image_differential.png" width="300" />

Some of the dynamics problems involve multiple contact points
and forces through them, for example the drivetrains.

This problem is similar to the "grasping" problem in robotics,
as if the robot were "grasping" the floor.

The general way to handle this situation is with the concept
of a "wrench", which is the generalization of "force" in SE2, i.e.
it represents both linear and rotational
forces on the rigid body.
It can be written with generality as the stacked vector:

```math
\mathbf{w}
=
\begin{bmatrix}
\mathbf{f} \\
\boldsymbol{\tau}
\end{bmatrix}
\tag{1}
```

where $\mathbf{f}$ is the total force vector
and $\boldsymbol{\tau}$ the total torque
around the center of mass:


In SE2 it is three dimensional:

```math
\mathbf{w}
=
\begin{bmatrix}
f_x \\
f_y \\
\tau
\end{bmatrix}
\tag{2}
```

Each component actuator (wheel) produces a
linear force $\mathbf{f_i}$
acting at a point $\mathbf{r_i}$,
which yields a wrench $\mathbf{w_i}$:

```math
\mathbf{w_i}
=
\begin{bmatrix}
\mathbf{f_i} \\
\mathbf{r_i} \times \mathbf{f_i}
\end{bmatrix}
\tag{3}
```

## Constrained Actuation

One way to simplify the problem is to imagine that the contacts
(wheels) can only produce force perpendicular to their axes. 
(This isn't realistic -- it implies the wheels are "omni" wheels --
see below to remove this constraint.)
In this version of the problem, the 
actuation vector directions are fixed by the drive geometry:
the vectors $\mathbf{n_i}$ in the diagram above.

So the component wrenches can be written:

```math
\mathbf{w_i}
=
\begin{bmatrix}
\mathbf{n_i} \\
\mathbf{r_i} \times \mathbf{n_i}
\end{bmatrix}
f_i
\tag{4}

```
In SE2, the sum can be written:

```math
\begin{bmatrix}
f_x \\
f_y \\
\tau
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{n_1} && \mathbf{n_2} && \dots && \mathbf{n_k} \\
\mathbf{r_1} \times \mathbf{n_1} && \mathbf{r_2} \times \mathbf{n_2} &&\dots && \mathbf{r_k} \times \mathbf{n_k}
\end{bmatrix}
\begin{bmatrix}
f_1 \\
f_2 \\
\vdots \\
f_k
\end{bmatrix}
\tag{5}
```

Our setup is in two dimensions, so instead of the cross product,
we use the two-dimensional equivalent, the vector determinant:

```math
det(a,b) = a_x b_y - a_y b_x
\tag{6}
```

thus:

```math
\begin{bmatrix}
f_x \\
f_y \\
\tau
\end{bmatrix}
=
\begin{bmatrix}
n_{1x} && n_{2x}  \\
n_{y1} && n_{2y}  \\
r_{1x}n_{1y} - r_{1y}n_{1x} && r_{2x}n_{2y} - r_{2y}n_{2x} 
\end{bmatrix}
\begin{bmatrix}
f_1 \\
f_2 \\
\end{bmatrix}
\tag{7}
```

## Free Actuation

If we remove the constraint that the force directions are fixed,
then we start again with the component wrench definition:

```math
\mathbf{w_i}
=
\begin{bmatrix}
\mathbf{f_i} \\
\mathbf{r_i} \times \mathbf{f_i}
\end{bmatrix}
\tag{8}
```

In SE2 this can be written componentwise (dropping the $i$ for now):

```math
\mathbf{w}
=
\begin{bmatrix}
f_{x} \\
f_{y} \\
r_{x}f_{y} - r_{y}f_{x}
\end{bmatrix}
\tag{9}
```

Which can be written in terms of the "grasp" matrix,
$G$:

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
\tag{10}
```

## References

These are mostly about the general grasping problem, which has
many aspects that our simpler problems do not.

* [Lin et al 2013](https://www.scirp.org/journal/paperinformation?paperid=31739)
* [Tlale et al 2008](https://researchspace.csir.co.za/server/api/core/bitstreams/07942b39-865d-4a47-8611-d4e1336b8bb3/content)
* [Zeidis et al 2019](https://onlinelibrary.wiley.com/doi/full/10.1002/zamm.201900173)
* [Muir 1987](https://publications.ri.cmu.edu/storage/publications/pub_files/1991/3/01087767-1.pdf)
* [Moreno-Caireta et al](https://www.iri.upc.edu/files/scidoc/2467-Model-Predictive-Control-for-a-Mecanum-wheeled-Robot-Navigating-among-Obstacles.pdf)
* [Tani 2019](https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/amod/Lecture_13/20191104%20-%20ETH%20-%2001%20-%20Modeling.pdf) (see Duckietown)
* [Robotics Unveiled on Wrenches](https://www.roboticsunveiled.com/robotics-wrenches/)
* [Haber 2023](https://aleksandarhaber.com/clear-and-detailed-explanation-of-kinematics-equations-and-geometry-of-motion-of-differential-wheeled-robot-differential-drive-robot/)
* [Notes from Stanford CS273b](https://web.stanford.edu/class/cs237b/pdfs/lecture/cs237b_lecture_7.pdf)
* [Notes from Cal C106b](https://pages.github.berkeley.edu/EECS-106/sp22-site/assets/scribe_notes/scribe_lec_9A.pdf)
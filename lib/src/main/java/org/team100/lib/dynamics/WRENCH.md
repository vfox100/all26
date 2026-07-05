# Wrench

Some of the dynamics problems involve multiple contact points
and forces through them, for example the drivetrains.

This problem is similar to the "grasping" problem in robotics,
as if the robot were "grasping" the floor.

The general way to handle this situation is with the concept
of a "wrench", which represents both linear and rotational
forces on the rigid body (like a twist but for force and torque).
It can be written with generality as the stacked vector:

```math
\bold{w}
=
\begin{bmatrix}
\bold{f} \\
\boldsymbol{\tau}
\end{bmatrix}
```

where $\bold{f}$ is the total force vector
and $\boldsymbol{\tau}$ the total torque
around the center of mass:


In SE2 it is three dimensional:

```math
\bold{w}
=
\begin{bmatrix}
f_x \\
f_y \\
\tau
\end{bmatrix}
```

Each component actuator (wheel) produces a
linear force $\bold{f_i}$
acting at a point $\bold{r_i}$,
which yields a wrench $\bold{w_i}$:

```math
\bold{w_i}
=
\begin{bmatrix}
\bold{f_i} \\
\bold{r_i} \times \bold{f_i}
\end{bmatrix}
```

Our problem involves a key constraint, which is that the
actuation vector directions are fixed by the drive geometry:
the vectors $\bold{n_i}$ in the diagram above.

So the component wrenches can be written:

```math
\bold{w_i}
=
\begin{bmatrix}
\bold{n_i} \\
\bold{r_i} \times \bold{n_i}
\end{bmatrix}
f_i
```
So the sum can be written:

```math
\begin{bmatrix}
f_x \\
f_y \\
\tau
\end{bmatrix}
=
\begin{bmatrix}
\bold{n_1} && \bold{n_2} && \dots && \bold{n_k} \\
\bold{r_1} \times \bold{n_1} && \bold{r_2} \times \bold{n_2} &&\dots && \bold{r_k} \times \bold{n_k}
\end{bmatrix}
\begin{bmatrix}
f_1 \\
f_2 \\
\vdots \\
f_k
\end{bmatrix}
```

Our setup is in two dimensions, so instead of the cross product,
we use the two-dimensional equivalent, the vector determinant:

```math
det(a,b) = a_x b_y - a_y b_x
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
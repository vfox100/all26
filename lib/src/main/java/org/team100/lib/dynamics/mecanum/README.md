# Mecanum Drive Dynamics

The dynamics of the Mecanum drivetrain.

There are many dynamics references; they all seem pretty
complicated.

Maybe instead, we could use the two-step method we initially
used for the differential drive.

* Determine SE2 components $F_x$, $F_y$, and $\tau$.
* Find the drive forces that yield this SE2 wrench.

The general way to handle this situation is with the concept
of a "wrench" (like a twist but for force and torque).
A force $f$ acting at a point $r$ produces a wrench $\mathcal{F}$,
using the cross-product of the point and the force.

The forward dynamics is simple: sum the wrenches.

What we want is inverse dynamics: given points $r_i$ and
a direction for each, determine the magnitude of force required
to produce the total wrench.

To understand multiple contact points and the wrenches they
produce, robotics folk have invented the "grasping matrix",
which relates the point forces to the total rigid-body wrench.
We'll do something similar, somewhat as if the robot were
"grasping" the floor, or, equivalently, the floor were "grasping"
the robot.

In the general case, there is a total wrench on the
robot, described by the stacked matrix,
where $\bold{f}$ is the total force vector
and $\boldsymbol{\tau}$ the total torque
around the center of mass:

```math
\bold{w}
=
\begin{bmatrix}
\bold{f} \\
\boldsymbol{\tau}
\end{bmatrix}
```

The component point forces $\bold{f_i}$,
each acting at a point $\bold{r_i}$,
define the component wrenches:

```math
\bold{w_i}
=
\begin{bmatrix}
\bold{f_i} \\
\bold{r_i} \times \bold{f_i}
\end{bmatrix}
```




## References
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
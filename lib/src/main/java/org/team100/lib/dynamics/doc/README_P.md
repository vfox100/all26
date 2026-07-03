# P Dynamics

Dynamics of a prismatic joint in one dimension.

* the position is $q$.
* the mass of the moving part is $m$.

## Mass

For a prismatic joint along X, the mass matrix is scalar and
independent of $q$.

```math
M = m
```

## Centrifugal and Coriolis

For this system, there is no centrifugal/coriolis term.

## Gravity

For now, let's say the joint is horizontal, so there is no gravity term.

TODO: add gravity.

## Equation of Motion

Start with the definition:

```math
\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q)
```

So we get the familiar expression:

```math
\tau = m \ddot{q}
```

better known as

```math
F = m a
```
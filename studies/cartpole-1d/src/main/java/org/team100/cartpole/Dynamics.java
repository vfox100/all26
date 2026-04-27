package org.team100.cartpole;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

/**
 * linear:
 * xdot = Ax + Bu
 * 
 * nonlinear:
 * xdot = f(x,u)
 */
public interface Dynamics {
    /**
     * @param x state (x, xdot, theta, thetadot)
     * @param u control force on cart
     */
    Vector<N4> xdot(Vector<N4> x, double u);
}
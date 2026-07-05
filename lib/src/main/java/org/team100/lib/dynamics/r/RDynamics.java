package org.team100.lib.dynamics.r;

/**
 * TODO: allow variable angle of gravity.
 * 
 * Alternatively, we could just have a convention such that the "zero"
 * of every coordinate system is "up".
 */
public class RDynamics {
    /** Gravity. */
    private static final double g = 9.8;
    /** Mass. */
    private final double m1;
    /** Distance from q to the link center of mass. */
    private final double lc1;
    /** Moment of inertia. */
    private final double izz1;

    /** Arm. */
    public RDynamics(double m1, double lc1, double izz1) {
        this.m1 = m1;
        this.lc1 = lc1;
        this.izz1 = izz1;
    }

    /**
     * Roller drum has only inertia.
     * 
     * @param I inertia kg m^2
     */
    public RDynamics(double I) {
        this(0, 0, I);
    }

    /**
     * Generalized force (torque or force) to achieve the required
     * acceleration, and also to oppose gravity.
     * 
     * Note there is no velocity term here because the dynamics don't
     * depend on velocity.
     */
    public REffort effort(RConfig q, RAcceleration a) {
        double s1 = Math.sin(q.q1());
        double m11 = m1 * lc1 * lc1 + izz1;
        double g1 = -m1 * g * lc1 * s1;
        double t1 = m11 * a.q1ddot() + g1;
        return new REffort(t1);
    }

}

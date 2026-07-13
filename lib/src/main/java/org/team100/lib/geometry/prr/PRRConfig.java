package org.team100.lib.geometry.prr;

/**
 * Joint configuration for the PRR example.
 * 
 * @param q1 extension of the P joint (+x)
 * @param q2 rotation of the first R joint (CCW from +x)
 * @param q3 rotation of the second R joint (CCW from the first)
 */
public record PRRConfig(double q1, double q2, double q3) {
        /**
     * Apply the velocity for period dt.
     * 
     * x = x0 + v dt
     */
    public PRRConfig integrate(PRRVelocity jv, double dt) {
        return new PRRConfig(
                q1 + jv.q1dot() * dt,
                q2 + jv.q2dot() * dt,
                q3 + jv.q3dot() * dt);
    }

    /**
     * v = (x - x0) / dt
     */
    public PRRVelocity diff(PRRConfig c, double dt) {
        return new PRRVelocity(
                (q1 - c.q1) / dt,
                (q2 - c.q2) / dt,
                (q3 - c.q3) / dt);
    }

    /** True if any of the axes are NaN */
    public boolean isNaN() {
        return Double.isNaN(q1())
                || Double.isNaN(q2())
                || Double.isNaN(q3());
    }

    @Override
    public String toString() {
        return String.format("%6.3f %6.3f %6.3f", q1, q2, q3);
    }

}

package org.team100.cartpole;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

/**
 * Includes the inertia of the pole (not a point mass) and friction in both
 * joints.
 *
 * This is the "hybrid" model where the inertia of the pole is taken into
 * account in a limited way: mass is treated as a point mass in the middle.
 * 
 * @see https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html
 */
public class CartPoleEquationsWithFrictionAndInertia implements Dynamics {
    /** m/s^2 */
    private static final double g = 9.8;
    /** pole mass (kg) */
    private static final double m_p = 0.1;
    /** Mass of the cart mass (kg) */
    private static final double m_c = 1.0;
    /** Half of the pole's length, for center of mass */
    private final double half_l;
    /** friction at the pivot joint. */
    private static final double mu_p = 0.001;
    /** friction between the cart and the track. */
    private static final double mu_c = 0.1;

    /**
     * @param l length (m)
     */
    public CartPoleEquationsWithFrictionAndInertia(double l) {
        half_l = l / 2.0;
    }

    @Override
    public Vector<N4> xdot(Vector<N4> x, double u) {
        double xdot = x.get(1);
        double theta = x.get(2);
        double thetadot = x.get(3);

        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        double cos2 = cos * cos;
        double thetadot2 = thetadot * thetadot;

        // Eq 56 in
        // https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html
        double xddot = (m_p * g * sin * cos - (7.0 / 3.0) * (u + m_p * half_l * thetadot2 * sin - mu_c * xdot)
                - ((mu_p * thetadot * cos) / half_l)) / (m_p * cos2 - (7.0 / 3.0) * (m_p + m_c));

        // Eq 57 in
        // https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html
        double thetaddot = (3.0 / (7.0 * half_l)) * (g * sin - xddot * cos - (mu_p * thetadot) / (m_p * half_l));

        return VecBuilder.fill(
                xdot,
                xddot,
                thetadot,
                thetaddot);
    }

}

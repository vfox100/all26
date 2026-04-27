package org.team100.cartpole;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

/**
 * Ignores friction, treats pole as massless with a mass on the end.
 * 
 * @see https://openmdao.github.io/dymos/examples/cart_pole/cart_pole.html
 * @see https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html
 */
public class CartPoleEquationsSimple implements Dynamics {
    /** m/s^2 */
    private static final double g = 9.8;
    /** pole mass (kg) */
    private static final double m_p = 0.1;
    /** Mass of the cart mass (kg) */
    private static final double m_c = 1.0;
    /** pole length */
    private final double l;

    /**
     * @param l length (m)
     */
    public CartPoleEquationsSimple(double l) {
        this.l = l;
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

        double xddot = (m_p * g * sin * cos - (u + m_p * l * thetadot2 * sin))
                / (m_p * cos2 - (m_p + m_c));
        double thetaddot = (g * sin - xddot * cos) / l;

        return VecBuilder.fill(
                xdot,
                xddot,
                thetadot,
                thetaddot);
    }
}

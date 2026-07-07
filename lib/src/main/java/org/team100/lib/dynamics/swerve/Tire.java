package org.team100.lib.dynamics.swerve;

/**
 * Tire model for slip angle calculation.
 * 
 * Cornering stiffness is the (linear, for small angle)
 * relation between slip angle and side force. For large
 * angles, the side force is just the normal force.
 * 
 * This assumes that stiffness is proportional to normal
 * force, which is only approximately true for "real" tires.
 * 
 * This assumes that the saturation angle is fixed, which
 * is only approximately true for "real" tires.
 */
public class Tire {
    /**
     * Normal force, Newtons.
     * 
     * For a 150-lb robot, a typical normal force is around 200N.
     * 
     * Note: in real cornering, the "inside" wheels receive less
     * normal force, and the "outside" wheels receive more normal
     * force, due to centrifugal force on the center of mass,
     * which is above the contact patches.
     */
    private final double m_normalN;
    /**
     * Maximum slip angle, radians.
     * 
     * I'm not sure what a good value to use is: try 0.05 rad.
     */
    private final double m_saturationRad;

    /**
     * @param normalN       corner weight, try 200 N
     * @param saturationRad max useful slip angle, try 0.05 rad
     */
    public Tire(double normalN, double saturationRad) {
        m_normalN = normalN;
        m_saturationRad = saturationRad;
    }

    /**
     * @param fN desired side force, Newtons (a positive number)
     * @return angle required (also positive)
     */
    public double angle(double fN) {
        if (fN < 0)
            throw new IllegalArgumentException();
        double f = Math.min(m_normalN, fN);
        double fraction = f / m_normalN;
        return fraction * m_saturationRad;
    }

}

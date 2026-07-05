package org.team100.lib.dynamics.swerve;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

/**
 * Here "torque "means "wheel force" in Newtons.
 * 
 * @param fl front left force N
 * @param fr front right force N
 * @param rl rear left force N
 * @param rr rear right force N
 */
public record SwerveEffort(
        double fl, double fr, double rl, double rr) {
    public static SwerveEffort fromtVector(Vector<N4> v) {
        return new SwerveEffort(
                v.get(0), v.get(1), v.get(2), v.get(3));
    }
}

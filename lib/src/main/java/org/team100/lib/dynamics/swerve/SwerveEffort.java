package org.team100.lib.dynamics.swerve;

import org.team100.lib.geometry.ForceR2;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N8;

/**
 * Forces at each corner of the swerve drive.
 * 
 * It is expected that the drive will project these
 * forces into whatever the steering orientation is.
 * 
 * It might be better for the steering projection to
 * be handled here?
 * 
 * @param fl front left
 * @param fr front right
 * @param rl rear left
 * @param rr rear right
 */
public record SwerveEffort(
        ForceR2 fl, ForceR2 fr, ForceR2 rl, ForceR2 rr) {
    /**
     * The argument is (f1x, f1y, f2x, f2y ...)
     * as specified in README.md.
     */
    public static SwerveEffort fromVector(Vector<N8> v) {
        return new SwerveEffort(
                new ForceR2(v.get(0), v.get(1)),
                new ForceR2(v.get(2), v.get(3)),
                new ForceR2(v.get(4), v.get(5)),
                new ForceR2(v.get(6), v.get(7)));
    }
}

package org.team100.lib.dynamics.swerve;

import org.team100.lib.dynamics.se2.SE2Dynamics;
import org.team100.lib.dynamics.se2.SE2Effort;
import org.team100.lib.geometry.AccelerationSE2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

/**
 * Implements the grasp matrix.  Note this is the inverse
 * of the transpose of the kinematics matrix, so maybe
 * implement it that way?
 */
public class SwerveDynamics {
    // This is unconstrained actuation so there are no
    // normal vectors here.
    /** Robot dynamics, to obtain the whole-robot wrench. */
    private final SE2Dynamics m_dyn;
    /** Inverse dynamics matrix. */
    private final Matrix<N8, N3> m_inv;

    public SwerveDynamics(
            double m, double I,
            Translation2d fl, Translation2d fr,
            Translation2d rl, Translation2d rr) {
        m_dyn = new SE2Dynamics(m, I);
        Matrix<N3, N8> fwd = MatBuilder.fill(Nat.N3(), Nat.N8(),
                1, 0, 1, 0, 1, 0, 1, 0, //
                0, 1, 0, 1, 0, 1, 0, 1, //
                -fl.getY(), fl.getX(), -fr.getY(), fr.getX(), //
                -rl.getY(), rl.getX(), -rr.getY(), rr.getX());
        m_inv = new Matrix<>(fwd.getStorage().pseudoInverse());
    }

    /**
     * Returns corner efforts for the given rigid body acceleration.
     * 
     * Acceleration here is extrinsic/inertial: no centrifugal force.
     */
    public SwerveEffort effort(AccelerationSE2 a) {
        // Compute rigid body wrench.
        SE2Effort se2Effort = m_dyn.effort(a);
        Vector<N3> w = se2Effort.vector();
        // Find contact forces.
        Vector<N8> f = new Vector<>(m_inv.times(w));
        return SwerveEffort.fromVector(f);
    }
}

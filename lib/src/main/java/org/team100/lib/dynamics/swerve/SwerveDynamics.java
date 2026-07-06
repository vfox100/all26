package org.team100.lib.dynamics.swerve;

import java.util.Optional;

import org.team100.lib.dynamics.se2.SE2Dynamics;
import org.team100.lib.dynamics.se2.SE2Effort;
import org.team100.lib.dynamics.swerve.SwerveConfig.ModuleConfig;
import org.team100.lib.geometry.AccelerationSE2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

/**
 * Implements the grasp matrix. Note this is the inverse
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
    public SwerveEffort effort(SwerveConfig q, AccelerationSE2 a) {
        // First find the corner forces
        Corners corners = corners(a);

        // Then decompose each corner into longitudinal and side forces
        ModuleConfig flc = q.fl();
        Corner flo = corners.fl();
        Optional<Rotation2d> fla = flc.angle();
        if (fla.isPresent()) {
            // if the angle exists, then there is velocity, thus a defined
            // longitudinal direction.
            Rotation2d flr = fla.get();
            // longitudinal normal
            Vector<N2> fln = VecBuilder.fill(flr.getCos(), flr.getSin());
            // total corner force, N
            Vector<N2> flf = flo.vector();
            // longitudinal component, N
            Vector<N2> fll = fln.projection(flf);
            // side component, N
            Vector<N2> fls = flf.minus(fll);
        } else {
            // if the angle does not exist, then the angle should be determined
            // by the required force, which is all longitudinal
        }

        ModuleConfig frc = q.fr();
        ModuleConfig rlc = q.rl();
        ModuleConfig rrc = q.rr();

        return null;
    }

    /**
     * Corner forces for the given rigid body acceleration.
     * 
     * Acceleration here is extrinsic/inertial: no centrifugal force.
     */
    public Corners corners(AccelerationSE2 a) {
        // Compute rigid body wrench.
        SE2Effort se2Effort = m_dyn.effort(a);
        Vector<N3> w = se2Effort.vector();
        // Find contact forces.
        Vector<N8> f = new Vector<>(m_inv.times(w));
        return Corners.fromVector(f);
    }

    /**
     * Planar (R2) force in Newtons.
     */
    record Corner(double x, double y) {
        Vector<N2> vector() {
            return VecBuilder.fill(x, y);
        }
    }

    record Corners(Corner fl, Corner fr, Corner rl, Corner rr) {
        /**
         * The argument is (f1x, f1y, f2x, f2y ...)
         * as specified in README.md.
         */
        public static Corners fromVector(Vector<N8> v) {
            return new Corners(
                    new Corner(v.get(0), v.get(1)),
                    new Corner(v.get(2), v.get(3)),
                    new Corner(v.get(4), v.get(5)),
                    new Corner(v.get(6), v.get(7)));
        }
    }
}

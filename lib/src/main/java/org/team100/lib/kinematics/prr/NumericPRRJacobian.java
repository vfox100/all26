package org.team100.lib.kinematics.prr;

import java.util.function.Function;

import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.optimization.NumericalJacobian100;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N3;

/**
 * The Jacobian describes how the kinematics affect velocities.
 * This version uses numerical differentiation, which isn't awesome.
 * 
 * Use AnalyticalJacobian instead.
 */
public class NumericPRRJacobian {
    private final PRRKinematics m_k;
    /** Vector version of the forward kinematics */
    private final Function<Vector<N3>, Vector<N3>> m_f;

    public NumericPRRJacobian(PRRKinematics k) {
        m_k = k;
        m_f = q -> pose(k.forward(config(q)));
    }

    /**
     * Given a joint configuration and velocities, what is the cartesian velocity?
     */
    public VelocitySE2 forward(PRRConfig c, PRRVelocity v) {
        Matrix<N3, N3> j = NumericalJacobian100.numericalJacobian(
                Nat.N3(), Nat.N3(), m_f, config(c));
        return VelocitySE2.fromVector(j.times(v.toVector()));
    }

    /**
     * Given a state (position and velocity), what are the joint velocities?
     * It's a bit weird to use "swerve" here. Maybe rename this SE2Model?
     */
    public PRRVelocity inverse(ModelSE2 swerveModel) {
        Pose2d p = swerveModel.pose();
        VelocitySE2 v = swerveModel.velocity();
        PRRConfig c = m_k.inverse(p);
        Matrix<N3, N3> j = NumericalJacobian100.numericalJacobian(
                Nat.N3(), Nat.N3(), m_f, config(c));
        if (Math.abs(j.det()) < 1e-3) {
            // Don't try to invert if it's not possible.
            // The safe thing is to stop.
            return new PRRVelocity(0, 0, 0);
        }
        return PRRVelocity.fromVector(j.inv().times(v.toVector()));
    }

    public static Vector<N3> config(PRRConfig c) {
        return VecBuilder.fill(c.q1(), c.q2(), c.q3());
    }

    public static PRRConfig config(Vector<N3> q) {
        return new PRRConfig(q.get(0), q.get(1), q.get(2));
    }

    public static Vector<N3> pose(Pose2d p) {
        return VecBuilder.fill(p.getX(), p.getY(), p.getRotation().getRadians());
    }

    public static Pose2d pose(Vector<N3> v) {
        return new Pose2d(v.get(0), v.get(1), new Rotation2d(v.get(2)));
    }

    public static Vector<N3> transform(Transform2d t) {
        return VecBuilder.fill(t.getX(), t.getY(), t.getRotation().getRadians());
    }
}

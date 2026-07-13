package org.team100.lib.kinematics.prr;

import org.team100.lib.geometry.prr.PRRAcceleration;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

/**
 * The PRR end-effector Jacobian is simple enough to write out. It's almost
 * identical to the center-of-mass Jacobian for the last link, used for
 * dynamics.
 * 
 * See doc/README.md
 */
public class AnalyticalPRRJacobian {
    private final PRRKinematics m_k;
    // notation from PRRDynamics.
    private final double l2;
    private final double l3;

    public AnalyticalPRRJacobian(PRRKinematics k) {
        m_k = k;
        l2 = k.getArmLength();
        l3 = k.getManipulatorLength();
    }

    /**
     * Forward velocity kinematics.
     * 
     * \dot{x} = J\dot{q}
     * 
     * See doc/README.md equation 4
     */
    public VelocitySE2 forward(PRRConfig q, PRRVelocity qdot) {
        Matrix<N3, N3> j = getJ(q);
        return VelocitySE2.fromVector(j.times(qdot.toVector()));
    }

    /**
     * Inverse velocity kinematics.
     * 
     * \dot{q} = J^{-1}\dot{x}
     * 
     * See README.md equation 5
     */
    public PRRVelocity inverse(ModelSE2 m) {
        Pose2d x = m.pose();
        VelocitySE2 xdot = m.velocity();
        PRRConfig q = m_k.inverse(x);
        Matrix<N3, N3> Jinv = getJinv(q);
        return PRRVelocity.fromVector(Jinv.times(xdot.toVector()));
    }

    /**
     * Forward acceleration kinematics.
     * 
     * \ddot{x} = \dot{J}\dot{q} + J\ddot{q}
     * 
     * See doc/README.md equation 6
     */
    public AccelerationSE2 forwardA(
            PRRConfig q, PRRVelocity qdot, PRRAcceleration qddot) {
        Matrix<N3, N3> J = getJ(q);
        Matrix<N3, N3> Jdot = getJdot(q, qdot);
        return AccelerationSE2.fromVector(
                new Vector<N3>(Jdot.times(qdot.toVector()).plus(J.times(qddot.toVector()))));
    }

    /**
     * Inverse acceleration kinematics.
     * 
     * \ddot{q} = J^{-1}(\ddot{x} - \dot{J}J^{-1}\dot{x})
     * 
     * See doc/README.md equation 9
     */
    public PRRAcceleration inverseA(ControlSE2 m) {
        Pose2d x = m.pose();
        VelocitySE2 xdot = m.velocity();
        AccelerationSE2 xddot = m.acceleration();
        PRRConfig q = m_k.inverse(x);
        Matrix<N3, N3> Jinv = getJinv(q);
        PRRVelocity qdot = PRRVelocity.fromVector(Jinv.times(xdot.toVector()));
        Matrix<N3, N3> Jdot = getJdot(q, qdot);
        return PRRAcceleration.fromVector(
                Jinv.times(
                        xddot.toVector().minus(
                                Jdot.times(Jinv.times(xdot.toVector())))));
    }

    /////////////////////////////////////////////////

    /**
     * End-effector Jacobian.
     * 
     * See doc/README.md equation 3
     */
    private Matrix<N3, N3> getJ(PRRConfig q) {
        double q2 = q.q2();
        double q3 = q.q3();
        double s2 = Math.sin(q2);
        double c2 = Math.cos(q2);
        double s23 = Math.sin(q2 + q3);
        double c23 = Math.cos(q2 + q3);
        Matrix<N3, N3> J = new Matrix<>(Nat.N3(), Nat.N3());
        J.set(0, 0, 1);
        J.set(0, 1, -l2 * s2 - l3 * s23);
        J.set(0, 2, -l3 * s23);
        J.set(1, 0, 0);
        J.set(1, 1, l2 * c2 + l3 * c23);
        J.set(1, 2, l3 * c23);
        J.set(2, 0, 0);
        J.set(2, 1, 1);
        J.set(2, 2, 1);
        return J;
    }

    /**
     * Time-derivative of the end-effector Jacobian.
     * 
     * See doc/README.md equation 7
     */
    private Matrix<N3, N3> getJdot(PRRConfig q, PRRVelocity qdot) {
        double q2 = q.q2();
        double q3 = q.q3();
        double s2 = Math.sin(q2);
        double c2 = Math.cos(q2);
        double s23 = Math.sin(q2 + q3);
        double c23 = Math.cos(q2 + q3);
        double q2dot = qdot.q2dot();
        double q3dot = qdot.q3dot();
        Matrix<N3, N3> Jdot = new Matrix<>(Nat.N3(), Nat.N3());
        Jdot.set(0, 0, 0);
        Jdot.set(0, 1, -l2 * c2 * q2dot - l3 * c23 * (q2dot + q3dot));
        Jdot.set(0, 2, -l3 * c23 * (q2dot + q3dot));
        Jdot.set(1, 0, 0);
        Jdot.set(1, 1, -l2 * s2 * q2dot - l3 * s23 * (q2dot + q3dot));
        Jdot.set(1, 2, -l3 * s23 * (q2dot + q3dot));
        Jdot.set(2, 0, 0);
        Jdot.set(2, 1, 0);
        Jdot.set(2, 2, 0);
        return Jdot;
    }

    /**
     * Inverse Jacobian, or zero if singular.
     */
    private Matrix<N3, N3> getJinv(PRRConfig q) {
        Matrix<N3, N3> J = getJ(q);
        if (Math.abs(J.det()) < 1e-3) {
            // Don't try to invert if it's not possible.
            // a zero inverse determinant will result in zero speed,
            // which is the safe thing.
            System.out.printf("WARNING: zero jacobian for config %s\n", q.toString());
            return new Matrix<>(Nat.N3(), Nat.N3());
        }
        return J.inv();
    }
}

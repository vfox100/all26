package org.team100.lib.kinematics.pr;

import org.team100.lib.geometry.pr.PRConfig;
import org.team100.lib.geometry.pr.PRVelocity;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

/** Jacobian for the PR apparatus. */
public class PRJacobian {
    private final PRKinematics m_k;
    private final double l;

    public PRJacobian(PRKinematics k) {
        m_k = k;
        l = k.l();
    }

    public VelocityR2 forward(PRConfig q, PRVelocity qdot) {
        Matrix<N2, N2> J = J(q);
        return VelocityR2.fromVector2(J.times(qdot.toVector()));
    }

    public PRVelocity inverse(Translation2d x, VelocityR2 xdot) {
        PRConfig q = m_k.inverse(x);
        Matrix<N2, N2> Jinv = Jinv(q);
        return PRVelocity.fromVector(Jinv.times(xdot.toVector()));

    }

    /////////////////////////////////

    private Matrix<N2, N2> J(PRConfig q) {
        double s2 = Math.sin(q.q2());
        double c2 = Math.cos(q.q2());
        return MatBuilder.fill(Nat.N2(), Nat.N2(),
                1, -l * s2, //
                0, l * c2);
    }

    private Matrix<N2, N2> Jdot(PRConfig q, PRVelocity qdot) {
        return null;
    }

    private Matrix<N2, N2> Jinv(PRConfig q) {
        Matrix<N2, N2> J = J(q);
        if (Math.abs(J.det()) < 1e-3) {
            // not invertible
            System.out.printf("WARNING: zero jacobian for config %s\n", q.toString());
            return new Matrix<>(Nat.N2(), Nat.N2());
        }
        return J.inv();
    }

}

package org.team100.lib.kinematics.pr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.pr.PRConfig;
import org.team100.lib.geometry.pr.PRVelocity;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;

import edu.wpi.first.math.geometry.Translation2d;

public class PRJacobianTest {

    public void verify(double x, double y, VelocityR2 v) {
        assertEquals(x, v.x(), 1e-3);
        assertEquals(y, v.y(), 1e-3);
    }

    public void verify(double q1dot, double q2dot, PRVelocity v) {
        assertEquals(q1dot, v.q1dot(), 1e-3);
        assertEquals(q2dot, v.q2dot(), 1e03);
    }

    @Test
    void testForward() {
        PRKinematics k = new PRKinematics(1);
        PRJacobian j = new PRJacobian(k);
        VelocityR2 xdot = j.forward(new PRConfig(0, 0), new PRVelocity(0, 0));
        verify(0, 0, xdot);
        xdot = j.forward(new PRConfig(0, 0), new PRVelocity(1, 0));
        verify(1, 0, xdot);
        xdot = j.forward(new PRConfig(0, 0), new PRVelocity(0, 1));
        verify(0, 1, xdot);
        xdot = j.forward(new PRConfig(1, Math.PI / 2), new PRVelocity(0, 0));
        verify(0, 0, xdot);
        xdot = j.forward(new PRConfig(1, Math.PI / 2), new PRVelocity(1, 0));
        verify(1, 0, xdot);
        xdot = j.forward(new PRConfig(1, Math.PI / 2), new PRVelocity(0, 1));
        verify(-1, 0, xdot);
    }

    @Test
    void testInverse() {
        PRKinematics k = new PRKinematics(1);
        PRJacobian j = new PRJacobian(k);
        PRVelocity qdot = j.inverse(new Translation2d(2, 0), new VelocityR2(0, 0));
        verify(0, 0, qdot);
        qdot = j.inverse(new Translation2d(2, 0), new VelocityR2(1, 0));
        verify(1, 0, qdot);
        qdot = j.inverse(new Translation2d(2, 0), new VelocityR2(0, 1));
        verify(0, 1, qdot);
        qdot = j.inverse(new Translation2d(1, 1), new VelocityR2(0, 0));
        verify(0, 0, qdot);
        qdot = j.inverse(new Translation2d(1, 1), new VelocityR2(1, 0));
        verify(0, -1, qdot);
        qdot = j.inverse(new Translation2d(1, 1), new VelocityR2(0, 1));
        verify(0, 1, qdot);
        qdot = j.inverse(new Translation2d(1, 1), new VelocityR2(-1, 1));
        verify(0, 0, qdot);
    }

}

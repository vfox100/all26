package org.team100.lib.kinematics.prr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.prr.PRRAcceleration;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.trajectory.TrajectorySE2;
import org.team100.lib.trajectory.TrajectorySE2Entry;
import org.team100.lib.trajectory.TrajectorySE2Factory;
import org.team100.lib.trajectory.TrajectorySE2Planner;
import org.team100.lib.trajectory.constraint.ConstantConstraint;
import org.team100.lib.trajectory.constraint.TimingConstraint;
import org.team100.lib.trajectory.examples.TrajectoryExamples;
import org.team100.lib.trajectory.path.PathSE2Factory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Some tests from JacobianTest to verify correctness. */
public class AnalyticalPRRJacobianTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void testForward() {
        final PRRKinematics k = new PRRKinematics(2, 1);
        AnalyticalPRRJacobian j = new AnalyticalPRRJacobian(k);
        PRRConfig q = new PRRConfig(0, 0, 0);
        // extended and motionless
        PRRVelocity jv = new PRRVelocity(0, 0, 0);
        VelocitySE2 v = j.forward(q, jv);
        assertEquals(0, v.x(), DELTA);
        assertEquals(0, v.y(), DELTA);
        assertEquals(0, v.theta(), DELTA);

        // +shoulder => +y and +theta
        jv = new PRRVelocity(0, 1, 0);
        v = j.forward(q, jv);
        assertEquals(0, v.x(), DELTA);
        assertEquals(3, v.y(), DELTA);
        assertEquals(1, v.theta(), DELTA);

        // +wrist => +y and +theta
        jv = new PRRVelocity(0, 0, 1);
        v = j.forward(q, jv);
        assertEquals(0, v.x(), DELTA);
        assertEquals(1, v.y(), DELTA);
        assertEquals(1, v.theta(), DELTA);

        // bent at shoulder, +shoulder => -x, +theta
        q = new PRRConfig(0, Math.PI / 2, 0);
        jv = new PRRVelocity(0, 1, 0);
        v = j.forward(q, jv);
        assertEquals(-3, v.x(), DELTA);
        assertEquals(0, v.y(), DELTA);
        assertEquals(1, v.theta(), DELTA);

        // bent at shoulder and wrist, +wrist => -y, +theta
        q = new PRRConfig(0, Math.PI / 2, Math.PI / 2);
        jv = new PRRVelocity(0, 0, 1);
        v = j.forward(q, jv);
        assertEquals(0, v.x(), DELTA);
        assertEquals(-1, v.y(), DELTA);
        assertEquals(1, v.theta(), DELTA);
    }

    @Test
    void testInverse() {
        final PRRKinematics k = new PRRKinematics(2, 1);
        AnalyticalPRRJacobian j = new AnalyticalPRRJacobian(k);

        PRRConfig c = new PRRConfig(1, 0, 0);
        Pose2d p = k.forward(c);

        // some example velocities
        // zero velocity
        ModelSE2 v = new ModelSE2(p);

        PRRVelocity jv = j.inverse(v);
        assertEquals(0, jv.q1dot(), DELTA);
        assertEquals(0, jv.q2dot(), DELTA);
        assertEquals(0, jv.q3dot(), DELTA);

        // +x
        v = new ModelSE2(p, new VelocitySE2(1, 0, 0));
        jv = j.inverse(v);
        assertEquals(1, jv.q1dot(), DELTA);
        assertEquals(0, jv.q2dot(), DELTA);
        assertEquals(0, jv.q3dot(), DELTA);

        // +y
        v = new ModelSE2(p, new VelocitySE2(0, 1, 0));
        jv = j.inverse(v);
        assertEquals(0, jv.q1dot(), DELTA);
        assertEquals(0.5, jv.q2dot(), DELTA);
        assertEquals(-0.5, jv.q3dot(), DELTA);

        // +theta
        v = new ModelSE2(p, new VelocitySE2(0, 0, 1));
        jv = j.inverse(v);
        assertEquals(0, jv.q1dot(), DELTA);
        assertEquals(-0.5, jv.q2dot(), DELTA);
        assertEquals(1.5, jv.q3dot(), DELTA);
    }

    @Test
    void testForwardA() {
        final PRRKinematics k = new PRRKinematics(2, 1);
        AnalyticalPRRJacobian j = new AnalyticalPRRJacobian(k);
        PRRConfig q = new PRRConfig(0, 0, 0);
        // extended, motionless
        PRRVelocity qdot = new PRRVelocity(0, 0, 0);
        PRRAcceleration qddot = new PRRAcceleration(0, 0, 0);
        AccelerationSE2 a = j.forwardA(q, qdot, qddot);
        assertEquals(0, a.x(), DELTA);
        assertEquals(0, a.y(), DELTA);
        assertEquals(0, a.theta(), DELTA);

        // +elevator => +x
        q = new PRRConfig(0, 0, 0);
        qdot = new PRRVelocity(0, 0, 0);
        qddot = new PRRAcceleration(1, 0, 0);
        a = j.forwardA(q, qdot, qddot);
        assertEquals(1, a.x(), DELTA);
        assertEquals(0, a.y(), DELTA);
        assertEquals(0, a.theta(), DELTA);

        // +shoulder => +y, +theta
        q = new PRRConfig(0, 0, 0);
        qdot = new PRRVelocity(0, 0, 0);
        qddot = new PRRAcceleration(0, 1, 0);
        a = j.forwardA(q, qdot, qddot);
        assertEquals(0, a.x(), DELTA);
        assertEquals(3, a.y(), DELTA);
        assertEquals(1, a.theta(), DELTA);

        // +wrist => +y, +theta
        q = new PRRConfig(0, 0, 0);
        qdot = new PRRVelocity(0, 0, 0);
        qddot = new PRRAcceleration(0, 0, 1);
        a = j.forwardA(q, qdot, qddot);
        assertEquals(0, a.x(), DELTA);
        assertEquals(1, a.y(), DELTA);
        assertEquals(1, a.theta(), DELTA);

        // shoulder bent, +shoulder => -x, +theta
        q = new PRRConfig(0, Math.PI / 2, 0);
        qdot = new PRRVelocity(0, 0, 0);
        qddot = new PRRAcceleration(0, 1, 0);
        a = j.forwardA(q, qdot, qddot);
        assertEquals(-3, a.x(), DELTA);
        assertEquals(0, a.y(), DELTA);
        assertEquals(1, a.theta(), DELTA);

        // shoulder and wrist bent, +wrist => -y, +theta
        q = new PRRConfig(0, Math.PI / 2, Math.PI / 2);
        qdot = new PRRVelocity(0, 0, 0);
        qddot = new PRRAcceleration(0, 0, 1);
        a = j.forwardA(q, qdot, qddot);
        assertEquals(0, a.x(), DELTA);
        assertEquals(-1, a.y(), DELTA);
        assertEquals(1, a.theta(), DELTA);
    }

    @Test
    void testInverseA() {
        final PRRKinematics k = new PRRKinematics(2, 1);
        AnalyticalPRRJacobian j = new AnalyticalPRRJacobian(k);

        // extended, motionless
        PRRConfig c = new PRRConfig(0, 0, 0);
        Pose2d p = k.forward(c);
        VelocitySE2 v = new VelocitySE2(0, 0, 0);
        ControlSE2 m = new ControlSE2(p, v, new AccelerationSE2(0, 0, 0));
        PRRAcceleration ja = j.inverseA(m);
        assertEquals(0, ja.q1ddot(), DELTA);
        assertEquals(0, ja.q2ddot(), DELTA);
        assertEquals(0, ja.q3ddot(), DELTA);

        // +x => +elevator
        c = new PRRConfig(0, 0, 0);
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(1, 0, 0));
        ja = j.inverseA(m);
        assertEquals(1, ja.q1ddot(), DELTA);
        assertEquals(0, ja.q2ddot(), DELTA);
        assertEquals(0, ja.q3ddot(), DELTA);

        // +y => +shoulder and -wrist (because zero theta)
        c = new PRRConfig(0, 0, 0);
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(0, 1, 0));
        ja = j.inverseA(m);
        assertEquals(0, ja.q1ddot(), DELTA);
        assertEquals(0.5, ja.q2ddot(), DELTA);
        assertEquals(-0.5, ja.q3ddot(), DELTA);

        // +theta => -shoulder, +wrist (because no translation)
        c = new PRRConfig(0, 0, 0);
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(0, 0, 1));
        ja = j.inverseA(m);
        assertEquals(0, ja.q1ddot(), DELTA);
        assertEquals(-0.5, ja.q2ddot(), DELTA);
        assertEquals(1.5, ja.q3ddot(), DELTA);

        // bent shoulder, +x => +elevator only
        // using 45 deg because of singularity at 90
        c = new PRRConfig(0, Math.PI / 4, 0);
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(1, 0, 0));
        ja = j.inverseA(m);
        assertEquals(1, ja.q1ddot(), DELTA);
        assertEquals(0, ja.q2ddot(), DELTA);
        assertEquals(0, ja.q3ddot(), DELTA);

        // bent shoulder, +y => +elevator, +shoulder, -wrist (because no theta)
        c = new PRRConfig(0, Math.PI / 4, 0);
        // using 45 deg because of singularity at 90
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(0, 1, 0));
        ja = j.inverseA(m);
        assertEquals(1, ja.q1ddot(), DELTA);
        assertEquals(0.707, ja.q2ddot(), DELTA);
        assertEquals(-0.707, ja.q3ddot(), DELTA);

        // bent shoulder and wrist, +y => +elevator, +shoulder, -wrist
        c = new PRRConfig(0, Math.PI / 4, Math.PI / 4);
        // using 45 deg because of singularity at 90
        p = k.forward(c);
        v = new VelocitySE2(0, 0, 0);
        m = new ControlSE2(p, v, new AccelerationSE2(0, 1, 0));
        ja = j.inverseA(m);
        assertEquals(1, ja.q1ddot(), DELTA);
        assertEquals(0.707, ja.q2ddot(), DELTA);
        assertEquals(-0.707, ja.q3ddot(), DELTA);
    }

    @Test
    void testTrajectory() {
        final PRRKinematics k = new PRRKinematics(2, 1);
        AnalyticalPRRJacobian j = new AnalyticalPRRJacobian(k);

        List<TimingConstraint> constraints = List.of(new ConstantConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        Pose2d start = new Pose2d(1, -1, Rotation2d.kZero);
        Pose2d end = new Pose2d(2, 1, Rotation2d.k180deg);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 t = ex.restToRest(start, end);
        double d = t.duration();
        double dt = d / 20;
        for (double time = 0; time < d; time += dt) {
            TrajectorySE2Entry tp = t.sample(time);
            ModelSE2 sm = ModelSE2.fromMovingPathPointSE2(tp.point().point(), tp.point().velocity());
            Pose2d p = sm.pose();
            VelocitySE2 v = sm.velocity();
            PRRConfig c = k.inverse(p);
            PRRVelocity jv = j.inverse(sm);
            if (DEBUG)
                System.out.printf(
                        "s (%5.2f) pose(%5.2f %5.2f %5.2f) conf(%5.2f %5.2f %5.2f) tv(%5.2f %5.2f %5.2f) jv(%5.2f %5.2f %5.2f)\n",
                        time,
                        p.getX(), p.getY(), p.getRotation().getRadians(),
                        c.q1(), c.q2(), c.q3(),
                        v.x(), v.y(), v.theta(),
                        jv.q1dot(), jv.q2dot(), jv.q3dot());
        }
    }
}

package org.team100.lib.kinematics.prr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.prr.PRRConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PRRKinematicsTest {
    private static final boolean DEBUG = false;
    // one micrometer tolerance since all the math here is exact
    private static final double DELTA = 0.000001;

    @Test
    void testArmHeightComp() {
        PRRKinematics k = new PRRKinematics(5, 1);
        Translation2d wristPosition = new Translation2d(3, 3);
        double h = k.armX(wristPosition);
        if (DEBUG)
            System.out.println(wristPosition.getY());
        assertEquals(4, h);
    }

    @Test
    void testForward0() {
        PRRKinematics k = new PRRKinematics(0.3, 0.1);
        PRRConfig c = new PRRConfig(1, Math.toRadians(60), Math.toRadians(0));
        Pose2d p = k.forward(c);
        // 60 degrees so x is half the total length
        assertEquals(1.2, p.getX(), DELTA);
        // 30/60/90 triangle, this side is sqrt(3)/2
        assertEquals(0.4 * Math.sqrt(3) / 2, p.getY(), DELTA);
        // should be the same as the input
        assertEquals(Math.toRadians(60), p.getRotation().getRadians(), DELTA);
    }

    @Test
    void testForward1() {
        PRRKinematics k = new PRRKinematics(2, 1);
        // one meter high, zero shoulder (so to the right along x), zero wrist (also
        // along x)
        PRRConfig c = new PRRConfig(1, 0, 0);
        Pose2d p = k.forward(c);
        // should be the height plus the sum of the link lengths
        assertEquals(4.0, p.getX(), DELTA);
        // straight up
        assertEquals(0.0, p.getY(), DELTA);
        // relative angle should be zero
        assertEquals(Math.toRadians(0), p.getRotation().getRadians(), DELTA);
    }

    @Test
    void testInverse0() {
        // should be straight up
        PRRKinematics k = new PRRKinematics(2, 1);
        Pose2d p = new Pose2d(4, 0, Rotation2d.kZero);
        PRRConfig c = k.inverse(p);
        // pose at 4, total is 3 long, so shoulder at 1
        assertEquals(1, c.q1(), DELTA);
        assertEquals(0, c.q2(), DELTA);
        assertEquals(0, c.q3(), DELTA);
    }

    @Test
    void testInverseDownArm45Triangle() {
        // built for a 45 45 90 triangle for
        PRRKinematics k = new PRRKinematics((2 * Math.sqrt(2)), 1);
        Pose2d p = new Pose2d(0.1, 3, Rotation2d.kCCW_90deg);
        PRRConfig c = k.inverse(p);

        assertEquals(-1.9, c.q1(), 0.001);
        assertEquals(Math.toRadians(45), c.q2(), 0.001);
        assertEquals(Math.toRadians(45), c.q3(), 0.001);
    }

    @Test
    void testInverseDownArm() {
        PRRKinematics k = new PRRKinematics(2, 1);
        Pose2d p = new Pose2d(0.1, 2, Rotation2d.kCCW_90deg);
        PRRConfig c = k.inverse(p);

        assertEquals(0.1 - Math.sqrt(3), c.q1(), 0.001);
        assertEquals(Math.toRadians(30), c.q2(), 0.001);
        assertEquals(Math.toRadians(60), c.q3(), 0.001);
    }

    @Test
    void testInverse1() {
        // arm up, wrist to the side
        PRRKinematics k = new PRRKinematics(2, 1);
        Pose2d p = new Pose2d(3, 1, Rotation2d.kCCW_90deg);
        PRRConfig c = k.inverse(p);
        // arm length is 2, wrist location is at 3
        assertEquals(1, c.q1(), DELTA);
        assertEquals(0, c.q2(), DELTA);
        assertEquals(Math.PI / 2, c.q3(), DELTA);
    }

    @Test
    void testInverse2() {
        // arm to the side, wrist down
        PRRKinematics k = new PRRKinematics(2, 1);
        Pose2d p = new Pose2d(0, 2, Rotation2d.k180deg);
        PRRConfig c = k.inverse(p);
        assertEquals(1, c.q1(), DELTA);
        assertEquals(Math.PI / 2, c.q2(), DELTA);
        assertEquals(Math.PI / 2, c.q3(), DELTA);
    }

    @Test
    void testRoundTripInverseFirst() {
        PRRKinematics k = new PRRKinematics(0.3, 0.1);
        Pose2d p = new Pose2d(1.178, 0.207, new Rotation2d(Math.toRadians(55)));

        PRRConfig c2 = k.inverse(p);
        Pose2d p2 = k.forward(c2);

        assertEquals(p.getX(), p2.getX(), DELTA);
        assertEquals(p.getY(), p2.getY(), DELTA);
        assertEquals(p.getRotation().getRadians(), p2.getRotation().getRadians(), DELTA);

    }

    @Test
    void testRoundTripForwardFirst() {
        PRRKinematics k = new PRRKinematics(0.3, 0.1);
        PRRConfig c = new PRRConfig(1, Math.toRadians(60), Math.toRadians(60));

        Pose2d p2 = k.forward(c);
        assertEquals(1.1, p2.getX(), DELTA);
        assertEquals(0.3 * Math.sqrt(3) / 2 + 0.1 * Math.sqrt(3) / 2, p2.getY(), DELTA);
        assertEquals(Math.toRadians(120), p2.getRotation().getRadians(), DELTA);

        PRRConfig c2 = k.inverse(p2);
        assertEquals(c.q1(), c2.q1(), DELTA);
        assertEquals(c.q2(), c2.q2(), DELTA);
        assertEquals(c.q3(), c2.q3(), DELTA);
    }

}

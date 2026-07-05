package org.team100.lib.dynamics.mecanum;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.AccelerationSE2;

import edu.wpi.first.math.geometry.Translation2d;

public class MecanumDynamicsTest {
    @Test
    void test0() {
        MecanumDynamics d = new MecanumDynamics(
                1,
                1,
                new Translation2d(0.25, 0.25),
                new Translation2d(0.25, -0.25),
                new Translation2d(-0.25, 0.25),
                new Translation2d(-0.25, -0.25));
        AccelerationSE2 a = new AccelerationSE2(1, 0, 0);
        MecanumEffort e = d.effort(a);
        // TODO: this seems like double what it should be.
        assertEquals(0.5, e.fl(), 0.001);
        assertEquals(0.5, e.fr(), 0.001);
        assertEquals(0.5, e.rl(), 0.001);
        assertEquals(0.5, e.rr(), 0.001);
    }

    @Test
    void test1() {
        MecanumDynamics d = new MecanumDynamics(
                1,
                1,
                new Translation2d(0.25, 0.25),
                new Translation2d(0.25, -0.25),
                new Translation2d(-0.25, 0.25),
                new Translation2d(-0.25, -0.25));
        AccelerationSE2 a = new AccelerationSE2(0, 1, 0);
        MecanumEffort e = d.effort(a);
        // TODO: this seems like double what it should be.
        assertEquals(-0.5, e.fl(), 0.001);
        assertEquals(0.5, e.fr(), 0.001);
        assertEquals(0.5, e.rl(), 0.001);
        assertEquals(-0.5, e.rr(), 0.001);
    }

    @Test
    void test2() {
        MecanumDynamics d = new MecanumDynamics(
                1,
                1,
                new Translation2d(0.25, 0.25),
                new Translation2d(0.25, -0.25),
                new Translation2d(-0.25, 0.25),
                new Translation2d(-0.25, -0.25));
        AccelerationSE2 a = new AccelerationSE2(0, 0, 1);
        MecanumEffort e = d.effort(a);
        // TODO: this seems like double what it should be.
        assertEquals(-1, e.fl(), 0.001);
        assertEquals(1, e.fr(), 0.001);
        assertEquals(-1, e.rl(), 0.001);
        assertEquals(1, e.rr(), 0.001);
    }

    @Test
    void test3() {
        MecanumDynamics d = new MecanumDynamics(
                1,
                1,
                new Translation2d(0.25, 0.25),
                new Translation2d(0.25, -0.25),
                new Translation2d(-0.25, 0.25),
                new Translation2d(-0.25, -0.25));
        AccelerationSE2 a = new AccelerationSE2(1, 1, 0);
        MecanumEffort e = d.effort(a);
        assertEquals(0, e.fl(), 0.001);
        assertEquals(1, e.fr(), 0.001);
        assertEquals(1, e.rl(), 0.001);
        assertEquals(0, e.rr(), 0.001);
    }

}

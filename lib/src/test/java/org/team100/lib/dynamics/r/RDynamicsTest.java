package org.team100.lib.dynamics.r;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class RDynamicsTest {
    private static final double DELTA = 1e-3;

    @Test
    void test0() {
        RDynamics d = new RDynamics(1, 0.5, 1);
        // straight up
        REffort t = d.effort(
                new RConfig(0),
                new RAcceleration(0));
        // no torque
        assertEquals(0, t.t(), DELTA);
    }

    @Test
    void test1() {
        RDynamics d = new RDynamics(1, 0.5, 1);
        // to the side
        REffort t = d.effort(
                new RConfig(Math.PI / 2),
                new RAcceleration(0));
        // 1 kg is 0.5 m away, so 5Nm
        assertEquals(-4.9, t.t(), DELTA);
    }

    @Test
    void test4() {
        RDynamics d = new RDynamics(1, 0.5, 1);
        // accelerating
        REffort t = d.effort(
                new RConfig(0),
                new RAcceleration(1));
        // mass torque 0.25 + inertia 1 = 1.25
        assertEquals(1.25, t.t(), DELTA);

    }

    @Test
    void test5() {
        RDynamics d = new RDynamics(1, 0.5, 1);
        // slowing down
        REffort t = d.effort(
                new RConfig(0),
                new RAcceleration(-1));
        // slowing is the same
        assertEquals(-1.25, t.t(), DELTA);
    }

    @Test
    void testFlywheel() {
        // flywheel is an arm whose center of mass is at the pivot
        RDynamics d = new RDynamics(1, 0, 2);
        // speeding up
        REffort t = d.effort(
                new RConfig(0), // doesn't matter
                new RAcceleration(3));
        // t = I alpha
        assertEquals(6, t.t(), DELTA);
    }

}

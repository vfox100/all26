package org.team100.lib.dynamics.rr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;

public class RRDynamicsTest {
    private static final double DELTA = 1e-3;

    @Test
    void test0() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // straight up
        RREffort t = d.effort(
                new RRConfig(0, 0),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        // no torques
        assertEquals(0, t.t1(), DELTA);
        assertEquals(0, t.t2(), DELTA);
    }

    @Test
    void test1() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // to the side
        RREffort t = d.effort(
                new RRConfig(Math.PI / 2, 0),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        // 1 kg is 0.5 m away, so 5Nm, 1 kg 1.5 m away so 15Nm
        assertEquals(-19.6, t.t1(), DELTA);
        // 1 kg 0.5 m away
        assertEquals(-4.9, t.t2(), DELTA);
    }

    @Test
    void test2() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // wrist only to the side (bent arm)
        RREffort t = d.effort(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        // 1 kg 0.5 m away so 5Nm
        assertEquals(-4.9, t.t1(), DELTA);
        // 1 kg 0.5 m away (same as above)
        assertEquals(-4.9, t.t2(), DELTA);
    }

    @Test
    void test3() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // bent arm moving at the root
        RREffort t = d.effort(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        // 1 kg 0.5 m away so 5Nm
        assertEquals(-4.9, t.t1(), DELTA);
        // 1 kg 0.5 m away (same as above), minus centrifugal force
        assertEquals(-4.4, t.t2(), DELTA);
    }

    @Test
    void test4() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // bent arm accelerating at the root
        RREffort t = d.effort(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0),
                new RRAcceleration(1, 0));
        // 1 kg 0.5 m away so 5Nm, minus centrifugal force
        assertEquals(-1.4, t.t1(), DELTA);
        // 1 kg 0.5 m away (same as above), minus centrifugal force
        assertEquals(-3.65, t.t2(), DELTA);
    }

    @Test
    void test5() {
        RRDynamics d = new RRDynamics(1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // like a whip: extended, moving, slowing down at the root
        RREffort t = d.effort(
                new RRConfig(0, 0),
                new RRVelocity(1, 0),
                new RRAcceleration(-1, 0));
        // elbow tries to keep going, so push back
        assertEquals(-4.5, t.t1(), DELTA);
        // trying to slow down
        assertEquals(-1.75, t.t2(), DELTA);
    }

}

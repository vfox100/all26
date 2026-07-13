package org.team100.lib.dynamics.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.AccelerationSE2;

public class SE2DynamicsTest {
    private static final double DELTA = 1e-3;

    @Test
    void test0() {
        SE2Dynamics d = new SE2Dynamics(1, 2);
        // +ax
        SE2Effort t = d.effort(new AccelerationSE2(3, 0, 0));
        assertEquals(3, t.fx(), DELTA);
        assertEquals(0, t.fy(), DELTA);
        assertEquals(0, t.t(), DELTA);
    }

    @Test
    void test1() {
        SE2Dynamics d = new SE2Dynamics(1, 2);
        // +ay
        SE2Effort t = d.effort(new AccelerationSE2(0, 3, 0));
        assertEquals(0, t.fx(), DELTA);
        assertEquals(3, t.fy(), DELTA);
        assertEquals(0, t.t(), DELTA);
    }

    @Test
    void test2() {
        SE2Dynamics d = new SE2Dynamics(1, 2);
        // +alpha
        SE2Effort t = d.effort(new AccelerationSE2(0, 0, 3));
        assertEquals(0, t.fx(), DELTA);
        assertEquals(0, t.fy(), DELTA);
        assertEquals(6, t.t(), DELTA);
    }

}

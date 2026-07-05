package org.team100.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class AccelerationSE2Test {
    /** Accel in Y while moving in X. */
    @Test
    void test0() {
        VelocitySE2 v0 = new VelocitySE2(1, 0, 0);
        VelocitySE2 v1 = new VelocitySE2(1, 0.1, 0);
        AccelerationSE2 a = AccelerationSE2.diff(v0, v1, 0.02);
        assertEquals(0, a.x(), 0.001);
        assertEquals(5, a.y(), 0.001);
        assertEquals(0, a.theta(), 0.001);
    }

    /** No centrifugal force here. */
    @Test
    void test1() {
        VelocitySE2 v0 = new VelocitySE2(1, 0, 1);
        VelocitySE2 v1 = new VelocitySE2(1, 0, 1);
        AccelerationSE2 a = AccelerationSE2.diff(v0, v1, 0.02);
        assertEquals(0, a.x(), 0.001);
        assertEquals(0, a.y(), 0.001);
        assertEquals(0, a.theta(), 0.001);
    }
}

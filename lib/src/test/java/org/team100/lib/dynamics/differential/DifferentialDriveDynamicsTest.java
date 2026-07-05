package org.team100.lib.dynamics.differential;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.AccelerationSE2;

public class DifferentialDriveDynamicsTest {
    @Test
    void test0() {
        DifferentialDriveDynamics d = new DifferentialDriveDynamics(1, 1, 0.5);
        AccelerationSE2 a = new AccelerationSE2(1, 0, 0);
        DifferentialDriveEffort t = d.effort(a);
        assertEquals(0.5, t.F1(), 0.001);
        assertEquals(0.5, t.F2(), 0.001);
    }

    @Test
    void test1() {
        // very wide; radius is 1m
        DifferentialDriveDynamics d = new DifferentialDriveDynamics(1, 1, 2);
        AccelerationSE2 a = new AccelerationSE2(1, 0, 1);
        DifferentialDriveEffort t = d.effort(a);
        // f=ma=1*1, so fleft=0.5
        // t=Ialpha=1*1, f=t/r=1, fleft=-0.5
        assertEquals(0, t.F1(), 0.001);
        assertEquals(1, t.F2(), 0.001);
    }
}

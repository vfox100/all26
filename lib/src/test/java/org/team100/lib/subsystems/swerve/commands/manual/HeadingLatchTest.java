package org.team100.lib.subsystems.swerve.commands.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelR1;

import edu.wpi.first.math.geometry.Rotation2d;

class HeadingLatchTest {
    private static final double DELTA = 0.001;

    @Test
    void testInit() {
        HeadingLatch l = new HeadingLatch();
        ModelR1 s = new ModelR1();
        Rotation2d pov = null;
        Velocity input = new Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);
    }

    @Test
    void testLatch() {
        HeadingLatch l = new HeadingLatch();
        ModelR1 s = new ModelR1();
        Rotation2d pov = Rotation2d.kCCW_Pi_2;
        Velocity input = new Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
        pov = null;
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
    }

    @Test
    void testUnLatch() {
        HeadingLatch l = new HeadingLatch();
        ModelR1 s = new ModelR1();
        Rotation2d pov = Rotation2d.kCCW_Pi_2;
        Velocity input = new Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
        pov = null;
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
        input = new Velocity(0, 0, 1);
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);
    }

    @Test
    void testExplicitUnLatch() {
        HeadingLatch l = new HeadingLatch();
        ModelR1 s = new ModelR1();
        Rotation2d pov = Rotation2d.kCCW_Pi_2;
        Velocity input = new Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
        pov = null;
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
        l.unlatch();
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);
    }
}

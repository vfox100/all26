package org.team100.lib.controller.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.tuning.Mutable;

import edu.wpi.first.math.geometry.Pose2d;

public class LeadingAimTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() {
        Mutable.unpublishAll();
        FeedbackR1 feedback = new FullStateFeedback(log, 1, 0.01, false, 1, 1);
        LeadingAim aim = new LeadingAim(log, () -> 5.0, feedback);
        aim.reset();
        assertEquals(0, aim.getOmega(
                new ModelSE2(),
                new ModelR1(0, 0)), DELTA);

        aim.reset();
        assertEquals(Math.PI / 2, aim.getOmega(
                new ModelSE2(),
                new ModelR1(Math.PI / 2, 0)), DELTA);

        // strafing
        aim.reset();
        assertEquals(-1.01, aim.getOmega(
                new ModelSE2(new Pose2d(), new VelocitySE2(0, 1, 0)),
                new ModelR1(0, -1)), DELTA);
    }

}

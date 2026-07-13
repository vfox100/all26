package org.team100.lib.controller.se2;

import org.team100.lib.geometry.se2.DeltaSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.VelocityControlSE2Logger;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.VelocityControlSE2;

/**
 * A controller that doesn't do anything except return the "next" setpoint. This
 * is appropriate if feedback control is outboard.
 */
public class FeedforwardControllerSE2 extends ControllerSE2Base {
    private final VelocityControlSE2Logger m_log_u_FF;

    public FeedforwardControllerSE2(
            LoggerFactory parent,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        super(parent, xTolerance, thetaTolerance, xDotTolerance, omegaTolerance);
        LoggerFactory log = parent.type(this);
        m_log_u_FF = log.velocityControlSE2Logger(Level.TRACE, "feedforward");

    }

    @Override
    public VelocityControlSE2 calculate100(
            DeltaSE2 positionError,
            VelocitySE2 velocityError,
            ControlSE2 nextReference) {
        m_log_u_FF.log(() -> nextReference.velocityControl());
        return nextReference.velocityControl();
    }
}

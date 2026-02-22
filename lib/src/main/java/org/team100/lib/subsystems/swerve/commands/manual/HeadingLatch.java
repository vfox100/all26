package org.team100.lib.subsystems.swerve.commands.manual;

import org.team100.lib.state.ModelR1;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Remembers the most recent desired heading, substituting null if there's any
 * dtheta input.
 */
public class HeadingLatch {
    private static final double unlatch = 0.01;

    private Rotation2d m_desiredRotation = null;

    /**
     * @param maxARad_S2 supply an acceleration that matches whatever profile or
     *                   expectations you have for angular acceleration.
     */
    public Rotation2d latchedRotation(
            double maxARad_S2,
            ModelR1 state,
            Rotation2d pov,
            double inputOmega) {
        if (Math.abs(inputOmega) > unlatch) {
            // if the driver is trying to drive, then let them
            m_desiredRotation = null;
        } else if (pov != null) {
            // if the driver is trying to snap, then let them
            m_desiredRotation = pov;
        }
        return m_desiredRotation;
    }

    public void unlatch() {
        m_desiredRotation = null;
    }
}

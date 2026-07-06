package org.team100.lib.dynamics.swerve;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Just the aspects of swerve config that matter to the dynamics
 */
public record SwerveConfig(
        ModuleConfig fl, ModuleConfig fr, ModuleConfig rl, ModuleConfig rr) {

    /**
     * @param angle the *input* to the dynamics computation, i.e. angle in the
     *              direction of motion, if any.
     */
    public record ModuleConfig(Optional<Rotation2d> angle) {
    }

}

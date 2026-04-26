package org.team100.lib.subsystems.tank.commands;

import java.util.function.DoubleSupplier;

import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.VelocitySE2Logger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.tank.TankDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual tank-drive control using a single joystick (if using an
 * xbox style control, this will be the right-hand stick).
 */
public class TankManual extends Command {
    private final DoubleSupplier m_translation;
    private final DoubleSupplier m_rotation;
    private final double m_maxV;
    private final double m_maxOmega;
    private final TankDrive m_drive;
    private final DoubleLogger m_logTranslation;
    private final DoubleLogger m_logRotation;
    private final VelocitySE2Logger m_logVelocity;
    private final ChassisSpeedsLogger m_logChassisSpeeds;

    public TankManual(
            LoggerFactory parent,
            DoubleSupplier translation,
            DoubleSupplier rotation,
            double maxV,
            double maxOmega,
            TankDrive robotDrive) {
        LoggerFactory log = parent.type(this);
        m_logTranslation = log.doubleLogger(Level.TRACE, "translation");
        m_logRotation = log.doubleLogger(Level.TRACE, "rotation");
        m_logVelocity = log.VelocitySE2Logger(Level.TRACE, "velocity");
        m_logChassisSpeeds = log.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
        m_translation = translation;
        m_rotation = rotation;
        m_maxV = maxV;
        m_maxOmega = maxOmega;
        m_drive = robotDrive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        // double rotscale = 1 - 0.5 * Math.abs(m_translation.getAsDouble());
        double translationM_S = MathUtil.applyDeadband(m_translation.getAsDouble(), 0.1) * m_maxV;
        double rotationRad_S = MathUtil.applyDeadband(m_rotation.getAsDouble(), 0.1) * m_maxOmega;
        m_logTranslation.log(() -> translationM_S);
        m_logRotation.log(() -> rotationRad_S);
        Rotation2d currentRotation = m_drive.getPose().getRotation();
        VelocitySE2 v = SwerveKinodynamics.fromInstantaneousChassisSpeeds(
                new ChassisSpeeds(translationM_S, 0, rotationRad_S), currentRotation);
        m_logVelocity.log(() -> v);
        ChassisSpeeds s = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, currentRotation);
        m_logChassisSpeeds.log(() -> s);
        m_drive.setVelocity(s.vxMetersPerSecond, s.omegaRadiansPerSecond);
    }
}

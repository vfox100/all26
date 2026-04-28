package org.team100.lib.subsystems.shooter;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.rev.MinionSparkMotor;
import org.team100.lib.motor.rev.MinionSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

/** Configuration of motors on the demobot shooter. */
public class DrumShooterFactory {

    public static DualDrumShooter make(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CurrentLimit limit,
            CanId canL,
            CanId canR,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("shooter");
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");

        SimpleDynamics ff = new SimpleDynamics(log, 0, 0);
        Friction friction = new Friction(log, 0.07, 0.07, 0.01, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.02);

        // for simulation
        double maxSpeedM_S = 10;
        double freeSpeedRad_S = maxSpeedM_S * gearRatio / (0.5 * wheelDiaM);

        BareMotor motorL = getMotor(
                limit, logL, currentLog, freeSpeedRad_S, canL,
                MotorPhase.FORWARD, ff, friction, pid);
        BareMotor motorR = getMotor(
                limit, logR, currentLog, freeSpeedRad_S, canR,
                MotorPhase.REVERSE, ff, friction, pid);

        LinearMechanism mechL = new LinearMechanism(
                logL, motorL, motorL.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        LinearMechanism mechR = new LinearMechanism(
                logR, motorR, motorR.encoder(), gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(10);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);

        return new DualDrumShooter(parent,
                new OutboardLinearVelocityServo(logL, mechL, ref, 1),
                new OutboardLinearVelocityServo(logR, mechR, ref, 1));
    }

    private static BareMotor getMotor(
            CurrentLimit limit,
            LoggerFactory log,
            TotalCurrentLog currentLog,
            double freeSpeedRad_S,
            CanId canId,
            MotorPhase phase,
            SimpleDynamics ff,
            Friction friction,
            PIDConstants pid) {
        return switch (Identity.instance) {
            case BLANK ->
                new SimulatedBareMotor(log, freeSpeedRad_S);
            default -> new MinionSparkMotor(
                    log, currentLog, canId, NeutralMode100.BRAKE, phase,
                    limit, ff, friction, pid);
        };
    }

}

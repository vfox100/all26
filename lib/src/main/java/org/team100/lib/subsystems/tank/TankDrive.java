package org.team100.lib.subsystems.tank;

import org.team100.lib.dynamics.differential.DifferentialDriveDynamics;
import org.team100.lib.dynamics.differential.DifferentialDriveEffort;
import org.team100.lib.dynamics.p.PEffort;
import org.team100.lib.dynamics.se2.SE2Dynamics;
import org.team100.lib.dynamics.se2.SE2Effort;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.AccelerationSE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.visualization.VizUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Tank drive that uses two linear mechanisms and provides a pose estimate using
 * odometry only.
 */
public class TankDrive extends SubsystemBase {
    private final DoubleArrayLogger m_log_field_robot;
    private final DifferentialDriveDynamics m_dynamics;
    private final double m_trackWidthM;
    private final double m_maxSpeedM_S;
    private final LinearMechanism m_left;
    private final LinearMechanism m_right;
    private final DifferentialDriveKinematics m_kinematics;

    private final ChassisSpeedsLogger m_logChassisSpeeds;
    private final DoubleLogger m_logLeft;
    private final DoubleLogger m_logRight;

    private ChassisSpeeds m_speed;
    private DifferentialDriveWheelPositions m_positions;
    private Pose2d m_pose;

    public TankDrive(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            DifferentialDriveDynamics dynamics,
            double trackWidthM,
            double maxSpeedM_S,
            LinearMechanism left,
            LinearMechanism right) {
        LoggerFactory log = parent.type(this);
        m_dynamics = dynamics;
        m_logChassisSpeeds = log.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
        m_logLeft = log.doubleLogger(Level.TRACE, "left");
        m_logRight = log.doubleLogger(Level.TRACE, "right");
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, "robot");
        m_trackWidthM = trackWidthM;
        m_maxSpeedM_S = maxSpeedM_S;
        m_left = left;
        m_right = right;
        m_kinematics = new DifferentialDriveKinematics(m_trackWidthM);
        m_positions = new DifferentialDriveWheelPositions(0, 0);
        m_pose = new Pose2d();
    }

    /** Use arcade drive to set duty cycle directly. */
    public void setDutyCycle(double translationSpeed, double rotSpeed) {
        WheelSpeeds s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed, false);
        m_left.setDutyCycle(s.left);
        m_right.setDutyCycle(s.right);
    }

    /**
     * Use inverse kinematics to set wheel speeds.
     * 
     * New! Uses dynamics to compute motor forces.
     */
    public void setVelocity(double translationM_S, double rotationRad_S) {
        ChassisSpeeds speed = new ChassisSpeeds(translationM_S, 0, rotationRad_S);
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speed);
        wheelSpeeds.desaturate(m_maxSpeedM_S);
        ChassisSpeeds actual = m_kinematics.toChassisSpeeds(wheelSpeeds);
      
        AccelerationSE2 accel = accel(actual);
        DifferentialDriveEffort t = m_dynamics.effort(accel);

        double left = wheelSpeeds.leftMetersPerSecond;
        double right = wheelSpeeds.rightMetersPerSecond;

        m_left.setVelocity(left, t.F1());
        m_right.setVelocity(right, t.F2());

        m_logChassisSpeeds.log(() -> actual);
        m_logLeft.log(() -> left);
        m_logRight.log(() -> right);
    }

    public void stop() {
        m_left.stop();
        m_right.stop();
    }

    @Override
    public void periodic() {
        updatePose();
        m_log_field_robot.log(this::poseArray);
        m_left.periodic();
        m_right.periodic();
    }

    public void setPose(Pose2d p) {
        m_pose = p;
    }

    public Pose2d getPose() {
        return m_pose;
    }

    /** Set the drive velocity. */
    public Command driveWithVelocity(double translationM_S, double rotationRad_s) {
        return run(() -> setVelocity(translationM_S, rotationRad_s))
                .withName("drive with velocity");
    }

    private void updatePose() {
        // This twist is relative to the center of rotation, which is near the midpoint
        // of the drive wheel axis, not the center of the robot, unless the drive wheels
        // happen to be in the center.
        Twist2d twist = twist();
        m_pose = m_pose.exp(twist);
    }

    private Twist2d twist() {
        DifferentialDriveWheelPositions newPositions = new DifferentialDriveWheelPositions(
                m_left.getPositionM(),
                m_right.getPositionM());
        Twist2d twist = m_kinematics.toTwist2d(m_positions, newPositions);
        m_positions = newPositions;
        return twist;
    }

    private double[] poseArray() {
        return VizUtil.poseToArray(m_pose);
    }

    /**
     * Compute acceleration using backwards finite difference
     * on chassis speed, using a constant DT.
     */
    private AccelerationSE2 accel(ChassisSpeeds speed) {
        ChassisSpeeds dv = speed.minus(m_speed);
        m_speed = speed;
        ChassisSpeeds a = dv.div(TimedRobot100.LOOP_PERIOD_S);
        return new AccelerationSE2(
                a.vxMetersPerSecond, a.vyMetersPerSecond, a.omegaRadiansPerSecond);
    }
}

package org.team100.lib.dynamics.mecanum;

import org.team100.lib.geometry.AccelerationSE2;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Maps desired acceleration in SE2 (in the robot frame)
 * to linear forces produced at each wheel.
 * 
 * Ignores "slip". TODO: add slip.
 */
public class MecanumDynamics {
    /** Mass, kg */
    private final double m_m;
    /** Inertia, kg m^2 */
    private final double m_I;
    // wheels
    private final Translation2d m_fl;
    private final Translation2d m_fr;
    private final Translation2d m_rl;
    private final Translation2d m_rr;

    public MecanumDynamics(
            double m, double I,
            Translation2d fl, Translation2d fr,
            Translation2d rl, Translation2d rr) {
        m_m = m;
        m_I = I;
        m_fl = fl;
        m_fr = fr;
        m_rl = rl;
        m_rr = rr;
    }

    /**
     * Here "torque" is actually linear force in Newtons.
     * TODO: some of these calculations can be done in advance.
     */
    public MecanumEffort effort(AccelerationSE2 a) {
        // First compute the total force and torque given the
        // rigid body mass and inertia.
        //
        // total linear force, N
        double f = m_m * a.x();
        // total torque, Nm
        double t = m_I * a.theta();
        //
        // Project the linear force into the wheel rollers

        return null;
    }

}

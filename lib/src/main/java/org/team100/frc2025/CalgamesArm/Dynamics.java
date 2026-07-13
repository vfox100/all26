package org.team100.frc2025.CalgamesArm;

import org.team100.lib.dynamics.prr.PRRDynamics;
import org.team100.lib.dynamics.prr.PRREffort;
import org.team100.lib.geometry.prr.PRRAcceleration;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;

/**
 * Dynamics for 2025 Calgames arm. Units are meters, kilograms, Newton-meters,
 * etc.
 */
public class Dynamics {
    private static final double HAND_MASS = 6;
    private static final double ARM_MASS = 1;
    private static final double CARRIAGE_MASS = 13;

    /** From shoulder to arm center of mass */
    private static final double ARM_COM_LENGTH = 0.25;
    private static final double ARM_LENGTH = 0.5;
    private static final double ARM_INERTIA = ARM_MASS * ARM_LENGTH * ARM_LENGTH / 3;

    /** From wrist to hand center of mass */
    private static final double HAND_COM_LENGTH = 0.14;
    private static final double HAND_LENGTH = 0.343;
    private static final double HAND_INERTIA = HAND_MASS * HAND_LENGTH * HAND_LENGTH / 3;

    private final PRRDynamics m_dynamics;

    public Dynamics() {
        m_dynamics = new PRRDynamics(
                CARRIAGE_MASS,
                ARM_MASS,
                HAND_MASS,
                ARM_LENGTH,
                ARM_COM_LENGTH,
                HAND_COM_LENGTH,
                ARM_INERTIA,
                HAND_INERTIA);
    }

    public PRREffort forward(PRRConfig c, PRRVelocity jv, PRRAcceleration ja) {
        PRRConfig q = new PRRConfig(c.q1(), c.q2(), c.q3());
        PRRVelocity v = new PRRVelocity(jv.q1dot(), jv.q2dot(), jv.q3dot());
        PRRAcceleration a = new PRRAcceleration(ja.q1ddot(), ja.q2ddot(), ja.q3ddot());
        PRREffort t = m_dynamics.effort(q, v, a);
        return new PRREffort(t.f1(), t.t2(), t.t3());
    }

}

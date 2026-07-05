package org.team100.lib.dynamics.mecanum;

/**
 * Here "torque "means "wheel force" in Newtons.
 * 
 * @param fl front left force N
 * @param fr front right force N
 * @param rl rear left force N
 * @param rr rear right force N
 */
public record MecanumEffort(
        double fl, double fr, double rl, double rr) {
}

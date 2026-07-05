package org.team100.lib.dynamics.prr;

/**
 * Effort for the PR example.
 * 
 * @param f1 force on the P joint in N
 * @param t2 torque on the first R joint in Nm
 * @param t3 torque on the second R joint in Nm
 */
public record PRREffort(double f1, double t2, double t3) {

}

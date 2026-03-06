package org.team100.frc2026;

import java.util.List;
import java.util.OptionalDouble;
import java.util.function.DoubleFunction;

import org.team100.lib.util.Relation;

/**
 * Interpolates shooting parameters, given range in meters.
 */
public class ShooterTable {
    /**
     * @param range     (m) to target center, in 2d, measured on the floor
     * @param speed     (m/s) shooter drum speed
     * @param elevation (rad) of the hood, not the ball path
     * @param tof       (sec) measured with a stopwatch
     */
    record Row(double range, double speed, double elevation, double tof) {
    }

    private final Relation<Row> m_table;
    private final DoubleFunction<Double> m_rangeToSpeed;
    private final DoubleFunction<Double> m_rangeToElevation;
    private final DoubleFunction<Double> m_rangeToTof;

    /**
     * Returns empty for out-of-range, so if you want to be sure to return
     * something, include "far away" points in the list.
     */
    public ShooterTable(List<Row> rows) {
        m_table = new Relation<>();
        m_table.addAll(rows);
        m_rangeToSpeed = m_table.function(Row::range, Row::speed);
        m_rangeToElevation = m_table.function(Row::range, Row::elevation);
        m_rangeToTof = m_table.function(Row::range, Row::tof);
    }

    public OptionalDouble angle(double rangeM) {
        return emptyIfNull(m_rangeToElevation.apply(rangeM));
    }

    public OptionalDouble speed(double rangeM) {
        return emptyIfNull(m_rangeToSpeed.apply(rangeM));
    }

    public OptionalDouble tof(double rangeM) {
        return emptyIfNull(m_rangeToTof.apply(rangeM));
    }

    /////////////////////////////////////////////////

    private OptionalDouble emptyIfNull(Double x) {
        if (x == null)
            return OptionalDouble.empty();
        return OptionalDouble.of(x);
    }
}
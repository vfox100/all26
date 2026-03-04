package org.team100.lib.util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/** Samples from a continuous function, with a payload for each point. */
public class DiscreteFunction<V> {
    public record Point<V>(double x, double y, V p) {
    }

    private final List<Point<V>> m_points;

    public DiscreteFunction() {
        m_points = new ArrayList<>();
    }

    public void put(double x, double y, V value) {
        m_points.add(new Point<>(x, y, value));
    }

    public List<Point<V>> points() {
        return m_points;
    }

    // /** Requires monotonicity. */
    public DiscreteFunction<V> inverse() {
        checkMonotonicity();
        DiscreteFunction<V> inverse = new DiscreteFunction<>();
        for (Point<V> p : m_points) {
            inverse.put(p.y, p.x, p.p());
        }
        return inverse;
    }

    /** Throw if the function is not monotonic */
    private void checkMonotonicity() {
        // sort by x
        m_points.sort(Comparator.comparing(Point::x));
        boolean increasing = false;
        boolean decreasing = false;
        for (int i = 0; i < m_points.size() - 1; ++i) {
            Point<V> p0 = m_points.get(i);
            Point<V> p1 = m_points.get(i + 1);
            if (p0.y >= p1.y)
                increasing = true;
            if (p0.y <= p1.y)
                decreasing = true;
        }
        if (increasing && decreasing) {
            throw new IllegalStateException("non-monotonic function is not invertible");
        }
    }

}

package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;

/** Samples from a continuous function, with a payload for each point. */
public class DiscreteFunction<V> {
    private record Point<V>(double x, double y, V p) {
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
    // public DiscreteFunction<V> invert() {
        

    // }

}

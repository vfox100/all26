package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.DiscreteFunction.Point;

public class DiscreteFunctionTest {
    private static final boolean DEBUG = true;

    @Test
    void testMonotonic() {
        DiscreteFunction<String> f = new DiscreteFunction<>();
        f.put(0.0, 0.0, "a");
        f.put(1.0, 10.0, "b");
        f.put(2.0, 20.0, "c");
        if (DEBUG)
            System.out.println("F");
        dump(f);
        DiscreteFunction<String> finv = f.inverse();
        if (DEBUG)
            System.out.println("Finv");
        dump(finv);
    }

    @Test
    void testNonMonotonic() {
        DiscreteFunction<String> f = new DiscreteFunction<>();
        f.put(0.0, 0.0, "a");
        f.put(1.0, 10.0, "b");
        f.put(2.0, 0.0, "c");
        if (DEBUG)
            System.out.println("F");
        dump(f);
        assertThrows(IllegalStateException.class, f::inverse);
    }

    @Test
    void testZeroDerivative() {
        DiscreteFunction<String> f = new DiscreteFunction<>();
        f.put(0.0, 0.0, "a");
        f.put(1.0, 10.0, "b");
        f.put(2.0, 10.0, "c");
        if (DEBUG)
            System.out.println("F");
        dump(f);
        assertThrows(IllegalStateException.class, f::inverse);
    }

    private <V> void dump(DiscreteFunction<V> f) {
        for (Point<V> p : f.points()) {
            if (DEBUG)
                System.out.printf("%s\n", p);
        }
    }

}

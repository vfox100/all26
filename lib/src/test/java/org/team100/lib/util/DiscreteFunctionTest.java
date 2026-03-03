package org.team100.lib.util;

import org.junit.jupiter.api.Test;

public class DiscreteFunctionTest {
    @Test
    void testMonotonic() {
        DiscreteFunction<String> f = new DiscreteFunction<>();
        f.put(0.0, 0.0, "a");
        f.put(1.0, 1.0, "b");
        f.put(2.0, 2.0, "c");
    }

}

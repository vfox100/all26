package org.team100.lib.util;

import java.util.Optional;
import java.util.OptionalDouble;

public class OptUtil {

    public static OptionalDouble emptyIfNull(Double x) {
        if (x == null)
            return OptionalDouble.empty();
        return OptionalDouble.of(x);
    }

    public static <V> Optional<V> emptyIfNull(V v) {
        if (v == null)
            return Optional.empty();
        return Optional.of(v);
    }

}

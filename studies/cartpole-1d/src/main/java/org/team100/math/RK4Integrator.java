package org.team100.math;

import java.util.function.BiFunction;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

public class RK4Integrator implements Integrator {

    @Override
    public Vector<N4> integrate(
            BiFunction<Vector<N4>, Double, Vector<N4>> f,
            Vector<N4> x,
            double u,
            double dt) {
        Vector<N4> k1 = f.apply(x, u);
        Vector<N4> k2 = f.apply(x.plus(k1.times(dt / 2.0)), u);
        Vector<N4> k3 = f.apply(x.plus(k2.times(dt / 2.0)), u);
        Vector<N4> k4 = f.apply(x.plus(k3.times(dt)), u);
        Vector<N4> dx = k1.plus(k2.times(2.0)).plus(k3.times(2.0)).plus(k4).times(dt / 6.0);
        return x.plus(dx);
    }
}
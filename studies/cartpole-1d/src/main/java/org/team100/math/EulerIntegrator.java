package org.team100.math;

import java.util.function.BiFunction;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

public class EulerIntegrator implements Integrator {

    @Override
    public Vector<N4> integrate(
            BiFunction<Vector<N4>, Double, Vector<N4>> f,
            Vector<N4> x,
            double u,
            double dt) {
        Vector<N4> xdot = f.apply(x, u);
        Vector<N4> dx = xdot.times(dt);
        return x.plus(dx);
    }
}
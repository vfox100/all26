package org.team100.math;

import java.util.function.BiFunction;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

public interface Integrator {

    /**
     * @param f  function
     * @param x  initial state
     * @param u  output
     * @param dt time step
     * @return new state
     */
    Vector<N4> integrate(
            BiFunction<Vector<N4>, Double, Vector<N4>> f,
            Vector<N4> x,
            double u,
            double dt);

}
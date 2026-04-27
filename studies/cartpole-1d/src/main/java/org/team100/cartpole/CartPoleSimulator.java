package org.team100.cartpole;

import java.util.function.BiFunction;
import java.util.function.ToDoubleFunction;

import org.team100.math.Integrator;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

public class CartPoleSimulator {
    private final ToDoubleFunction<Vector<N4>> control;
    private final double dt;
    private final BiFunction<Vector<N4>, Double, Vector<N4>> f;
    private final Integrator integrator;

    public final double[] t;
    public final double[] x;
    public final double[] xdot;
    public final double[] theta;

    public CartPoleSimulator(
            double dt,
            double durationSecs,
            BiFunction<Vector<N4>, Double, Vector<N4>> f,
            Integrator integrator,
            ToDoubleFunction<Vector<N4>> control) {
        this.control = control;
        this.dt = dt;
        this.f = f;
        this.integrator = integrator;
        int steps = (int) (durationSecs / dt);
        t = new double[steps];
        x = new double[steps];
        xdot = new double[steps];
        theta = new double[steps];
    }

    public void run(Vector<N4> initialState) {
        double time = 0.0;
        Vector<N4> state = initialState;
        for (int timestep = 0; timestep < t.length; timestep++) {
            time += dt;
            t[timestep] = time;
            x[timestep] = state.get(0);
            xdot[timestep] = state.get(1);
            theta[timestep] = state.get(2);

            // the target state is zero, so the error is just the state.
            Vector<N4> error = state;
            double u = control.applyAsDouble(error);

            state = integrator.integrate(f, state, u, dt);
        }
    }
}

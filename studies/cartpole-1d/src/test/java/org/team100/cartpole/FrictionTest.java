package org.team100.cartpole;

import java.util.List;
import java.util.function.ToDoubleFunction;

import org.jfree.data.xy.XYDataset;
import org.junit.jupiter.api.Test;
import org.team100.lib.util.ChartUtil;
import org.team100.math.RK4Integrator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

/** Turn on ChartUtil.SHOW to see anything. */
public class FrictionTest {
    @Test
    void testFreeSwinging() {
        RK4Integrator phys = new RK4Integrator();
        ToDoubleFunction<Vector<N4>> control = (x) -> 0.0;
        double l = 1.0;
        Dynamics equations = new CartPoleEquationsWithFrictionAndInertia(l);
        CartPoleSimulator sim = new CartPoleSimulator(0.01, 15.0, equations::xdot, phys, control);
        Vector<N4> state = VecBuilder.fill(0, 0, 0.1, 0);
        sim.run(state);
        XYDataset d1 = ChartUtil.xy("x", sim.t, sim.x);
        XYDataset d2 = ChartUtil.xy("theta", sim.t, sim.theta);
        ChartUtil.plotStacked(d1, d2);
        ChartUtil.plotOverlay(List.of(CartPoleChart.make(l, sim)), 500);
    }

    @Test
    void testWithFeedback() {
        RK4Integrator phys = new RK4Integrator();
        // trial and error
        Vector<N4> K = VecBuilder.fill(-2, -2, -30, -7);
        ToDoubleFunction<Vector<N4>> control = (x) -> -1 * K.dot(x);
        double l = 1.0;
        Dynamics equations = new CartPoleEquationsWithFrictionAndInertia(l);
        CartPoleSimulator sim = new CartPoleSimulator(0.01, 15.0, equations::xdot, phys, control);
        Vector<N4> state = VecBuilder.fill(0, 0, 0.1, 0);
        sim.run(state);
        XYDataset d1 = ChartUtil.xy("x", sim.t, sim.x);
        XYDataset d2 = ChartUtil.xy("theta", sim.t, sim.theta);
        ChartUtil.plotStacked(d1, d2);
        ChartUtil.plotOverlay(List.of(CartPoleChart.make(l, sim)), 500);
    }
}

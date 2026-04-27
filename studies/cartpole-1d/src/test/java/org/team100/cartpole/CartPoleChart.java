package org.team100.cartpole;

import org.jfree.data.xy.VectorSeries;

public class CartPoleChart {
    public static VectorSeries make(double l, CartPoleSimulator sim) {
        int step = 10;
        VectorSeries s = new VectorSeries("sim");
        for (int i = 0; i < sim.t.length; i += step) {
            double x = sim.x[i];
            double theta = sim.theta[i];
            double dx = l * Math.sin(theta);
            double dy = l * Math.cos(theta);
            s.add(x, 0, dx, dy);
        }

        return s;
    }
}

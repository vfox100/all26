package org.team100.lib.dynamics.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.AccelerationSE2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

public class SwerveDynamicsTest {
    // Confirm the pseudoinverse of the example.
    @Test
    void test0() {
        Matrix<N3, N8> m = MatBuilder.fill(Nat.N3(), Nat.N8(),
                1, 0, 1, 0, 1, 0, 1, 0, //
                0, 1, 0, 1, 0, 1, 0, 1, //
                -1, 1, 1, 1, -1, -1, 1, -1);
        Matrix<N8, N3> minv = new Matrix<>(m.getStorage().pseudoInverse());
        System.out.println(minv);
    }

    /** Simple accel: all same, 1/4 each. */
    @Test
    void test1() {
        SwerveDynamics d = new SwerveDynamics(
                1,
                1,
                new Translation2d(0.25, 0.25),
                new Translation2d(0.25, -0.25),
                new Translation2d(-0.25, 0.25),
                new Translation2d(-0.25, -0.25));
        AccelerationSE2 a = new AccelerationSE2(1, 0, 0);
        SwerveEffort e = d.effort(a);
        assertEquals(0.25, e.fl().x(), 0.001);
        assertEquals(0, e.fl().y(), 0.001);

        assertEquals(0.25, e.fr().x(), 0.001);
        assertEquals(0, e.fr().y(), 0.001);

        assertEquals(0.25, e.rl().x(), 0.001);
        assertEquals(0, e.rl().y(), 0.001);

        assertEquals(0.25, e.rr().x(), 0.001);
        assertEquals(0, e.rr().y(), 0.001);
    }

}

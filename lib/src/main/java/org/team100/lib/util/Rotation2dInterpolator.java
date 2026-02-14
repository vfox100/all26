package org.team100.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolator;

public class Rotation2dInterpolator implements Interpolator<Rotation2d> {
    @Override
    public Rotation2d interpolate(Rotation2d startValue, Rotation2d endValue, double t) {
        return startValue.interpolate(endValue, t);
    }
}

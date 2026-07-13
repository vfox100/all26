package org.team100.lib.trajectory;

import java.util.List;

import org.team100.lib.geometry.se3.WaypointSE3;
import org.team100.lib.trajectory.path.PathSE3;
import org.team100.lib.trajectory.path.PathSE3Factory;
import org.team100.lib.trajectory.spline.SplineSE3;
import org.team100.lib.trajectory.spline.SplineSE3Factory;

public class TrajectorySE3Planner {
    private static final boolean DEBUG = false;

    private final PathSE3Factory m_pathFactory;
    private final TrajectorySE3Factory m_trajectoryFactory;

    public TrajectorySE3Planner(PathSE3Factory pathFactory, TrajectorySE3Factory trajectoryFactory) {
        m_pathFactory = pathFactory;
        m_trajectoryFactory = trajectoryFactory;
    }

    public TrajectorySE3 restToRest(List<WaypointSE3> waypoints) {
        return generateTrajectory(waypoints, 0.0, 0.0);
    }

    public TrajectorySE3 generateTrajectory(
            List<WaypointSE3> waypoints, double start_vel, double end_vel) {
        try {
            // Create a path from splines.
            List<SplineSE3> splines = SplineSE3Factory.splinesFromWaypoints(waypoints);
            PathSE3 path = m_pathFactory.fromWaypoints(splines);
            if (DEBUG)
                System.out.printf("PATH\n%s\n", path);
            // Generate the timed trajectory.
            TrajectorySE3 result = m_trajectoryFactory.fromPath(path, start_vel, end_vel);
            if (DEBUG)
                System.out.printf("TRAJECTORY\n%s\n", result);
            return result;
        } catch (IllegalArgumentException e) {
            // catches various kinds of malformed input, returns a no-op.
            // this should never actually happen.
            System.out.println("WARNING: Bad trajectory input!!");
            // print the stack trace if you want to know who is calling
            // e.printStackTrace();
            return new TrajectorySE3();
        }
    }

}

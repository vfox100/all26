package org.team100.frc2026.auton;

import java.lang.reflect.Array;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GraphAvoid {

        private final Translation2d point;

        public GraphAvoid(Translation2d point) {
            this.point = point;
        }

        public Translation2d getPoint () {
            return point;

        }
    }

    public class ManeuverableGraph {
        
    public static Map<GraphAvoid, List<GraphAvoid>> build(
        Translation2d start,
        Translation2d goal,
        List<Obstacle> obstacles
    ) {

    List<GraphAvoid> nodes = new ArrayList<>();
    nodes.add(new GraphAvoid(start));
    nodes.add(new GraphAvoid(goal));    

    for (Obstacle obstacle : obstacles) {
        for (Translation2d corner : obstacle.getCorners()) {
            nodes.add(new GraphAvoid(corner));
        }
    }
    
}
    Map<GraphA
}

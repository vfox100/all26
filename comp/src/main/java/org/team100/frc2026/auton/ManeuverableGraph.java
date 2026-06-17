package org.team100.frc2026.auton;

import java.lang.reflect.Array;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.team100.frc2026.auton.Obstacle;

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
  {
        
    }
    Map<GraphAvoid,List<GraphAvoid>> graph = new HashMap<>();
    

    for (GraphAvoid a : nodes) {
        graph.put(a, new ArrayList<>());

    for (GraphAvoid b : nodes) {
        if (a.equals(b))

            continue;

      if (intersectsObstacle(
        a.getPoint(),
        b.getPoint(),
        obstacles)); {
            graph.get(a).add(b);
private static boolean intersectsObstacle(
        Translation2d a,
        Translation2d b,
        List<Obstacle> obstacles
    ) { 
        for (Obstacle obstacle : obstacles) {
            if (segmentIntersectsRectangle(
                a,
                b,
                obstacle.getRectangle()
            )) {
                return true;
            }
        } return false;
    }
         private static boolean segmentIntersectsRectangle(
        Translation2d a,
        Translation2d b,
         Rectangle2d rect
    ) { Translation2d[] c = getCorners(rect);

        return lineIntersects(a,b,c[0], c[1]) ||
            lineIntersects(a,b,c[1], c[2]) ||
            lineIntersects(a,b,c[2], c[3]) ||
            lineIntersects(a,b,c[3], c[0]);
    }

    private static boolean lineIntersects(
        Translation2d p1, Translation2d p2,
        Translation2d p3, Translation2d p4
    ) {
        return segmentsIntersect(p1,p2,p3,p4);
    }
     private static boolean ccw (
        Translation2d a,
        Translation2d b, 
        Translation2d c
     ){
        return (c.getY() - a.getY()) * (b.getX() - a.getX()) >
            (b.getY() - a.getY()) * (c.getX() - a.getX());
     }

     }

}
     private static Translation2d[] getCorners(Rectangle2d rect) {

        Pose2d center = rectangle.getCenter();

        double halfX = rectangle.getXWidth() / 2.0 ;
        double halfY = rectangle.getYWidth() / 2.0 ;

    List<Translation2d> corners = List.of(
        new Translation2d(-halfX, halfY),
        new Translation2d(halfX, halfY),
        new Translation2d(halfX, -halfY),
        new Translation2d(-halfX, -halfY)
    );
}
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
import org.team100.frc2026.auton.ManeuverableGraph;


public class GraphAvoid {
    private final Translation2d point;

        public GraphAvoid(Translation2d point) {
            this.point = point;
        }

        public Translation2d getPoint () {
            return point;

    
        }
    }

  
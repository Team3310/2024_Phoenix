package frc.robot.util.PathFollowing;

import java.util.ArrayList;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Trajectories {
    private static PathConstraints normalConstraints = new PathConstraints(5.0, 2.5, Math.PI, Math.PI/2.0);

    public static PathPlannerPath getTestPath(){
        ArrayList<PathPoint> points = new ArrayList<>();
        GoalEndState end = new GoalEndState(0.0, Rotation2d.fromDegrees(180.0));

        points.add(new PathPoint(new Translation2d(0.0, 0.0), new RotationTarget(0.0, Rotation2d.fromDegrees(180.0))));
        points.add(new PathPoint(new Translation2d(1.0, 0.0), new RotationTarget(0.0, Rotation2d.fromDegrees(180.0))));

        return PathPlannerPath.fromPathPoints(points, normalConstraints, end);
    }
}

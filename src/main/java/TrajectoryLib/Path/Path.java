package TrajectoryLib.path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import TrajectoryLib.Splines.Spline2d;
import TrajectoryLib.geometry.GeometryUtil;

public class Path {
    public static final int NumOfSamplePoints = 20;

    private List<PathPoint> points;
    private GoalEndState goalEndState;
    private PathConstraints globalConstraints;

    public Path(Spline2d[] splines, RotationTarget[] rotationTargets) {
        this.points = buildPoints(splines, rotationTargets);
        this.globalConstraints = new PathConstraints(5.0, 4.0, 2.0 * Math.PI, 2.0 * Math.PI);
        this.goalEndState = new GoalEndState(points.get(numPoints() - 1).position.getVelocities(),
                points.get(numPoints() - 1).position.getRotation());

        precalcValues();
    }

    public Path(PathConstraints constraints, Spline2d[] splines, RotationTarget[] rotationTargets) {
        this.points = buildPoints(splines, rotationTargets);
        this.globalConstraints = constraints;

        precalcValues();
    }

    public int numPoints() {
        return points.size();
    }

    public List<PathPoint> getAllPathPoints(){
        return points;
    }

    public PathPoint getPoint(int index) {
        if (index < 0 || index > numPoints() - 1) {
            throw new IllegalArgumentException("invailid index");
        }

        return points.get(index);
    }

    public PathConstraints getGlobalConstraints() {
        return globalConstraints;
    }

    public List<PathPoint> buildPoints(Spline2d[] splines, RotationTarget[] rotationTargets) {
        List<PathPoint> points = new ArrayList<>();
        List<RotationTarget> rotTargets = new LinkedList<RotationTarget>(Arrays.asList(rotationTargets));

        double timeStep = 1.0 / ((double) NumOfSamplePoints);

        System.out.println(rotTargets);

        for (int i = 0; i < splines.length; i++) {
            for (int j = i == 0 ? 0 : 1; j < NumOfSamplePoints; j++) {
                double t = j * timeStep;
                RotationTarget target = null;
                if (!rotTargets.isEmpty()) {
                    if (Math.abs(rotTargets.get(0).getPosition() - t) <= Math.abs(
                            rotTargets.get(0).getPosition() - Math.min(t + timeStep, 1.0))) {
                        target = rotTargets.get(0);
                        rotTargets.remove(0);
                    }
                }
                points.add(new PathPoint(splines[i].getPoseWithMotion(j * timeStep), target));
            }
        }

        return points;
    }

    public GoalEndState getGoalEndState() {
        return goalEndState;
    }

    private void precalcValues() {
        if (numPoints() > 0) {
            for (int i = 0; i < points.size(); i++) {
                PathPoint point = points.get(i);
                if (point.constraints == null) {
                    point.constraints = globalConstraints;
                }

                point.curveRadius = getCurveRadiusAtPoint(i, points);

                if (Double.isFinite(point.curveRadius)) {
                    point.maxV = Math.min(
                            Math.sqrt(
                                    point.constraints.getMaxAccelerationMpsSq() * Math.abs(point.curveRadius)),
                            point.constraints.getMaxVelocityMps());
                } else {
                    point.maxV = point.constraints.getMaxVelocityMps();
                }

                if (i != 0) {
                    point.distanceAlongPath = points.get(i - 1).distanceAlongPath
                            + (points.get(i - 1).position.getPose().getTranslation().getDistance(point.position.getPose().getTranslation()));
                }
            }

            points.get(points.size() - 1).rotationTarget = new RotationTarget(-1, goalEndState.getRotation(),
                    goalEndState.shouldRotateFast());
            points.get(points.size() - 1).maxV = goalEndState.getVelocity().getMagnitude();
        }
    }

    private static double getCurveRadiusAtPoint(int index, List<PathPoint> points) {
        if (points.size() < 3) {
            return Double.POSITIVE_INFINITY;
        }

        if (index == 0) {
            return GeometryUtil.calculateRadius(
                    points.get(index).position.getPose(),
                    points.get(index + 1).position.getPose(),
                    points.get(index + 2).position.getPose());
        } else if (index == points.size() - 1) {
            return GeometryUtil.calculateRadius(
                    points.get(index - 2).position.getPose(),
                    points.get(index - 1).position.getPose(),
                    points.get(index).position.getPose());
        } else {
            return GeometryUtil.calculateRadius(
                    points.get(index - 1).position.getPose(),
                    points.get(index).position.getPose(),
                    points.get(index + 1).position.getPose());
        }
    }
}

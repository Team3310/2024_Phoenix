package TrajectoryLib.path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import TrajectoryLib.geometry.GeometryUtil;
import TrajectoryLib.geometry.Pose2dWithMotion;
import TrajectoryLib.path.Trajectory.State;
import TrajectoryLib.splines.Spline;
import TrajectoryLib.splines.Spline2d;

public class Path {
    public static final int NumOfSamplePoints = 100;

    private List<PathPoint> points;
    private List<RotationTarget> rotationTargets;
    private GoalEndState goalEndState;
    private PathConstraints globalConstraints;

    public Path(Spline2d[] splines, RotationTarget[] rotationTargets) {
        for (Spline2d spline : splines) {
            spline.printCoefficients();
        }

        this.rotationTargets = Arrays.asList(rotationTargets);
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

        // System.out.println(rotTargets);

        for (int i = 0; i < splines.length; i++) {
            for (int j = i == 0 ? 0 : 1; j < NumOfSamplePoints; j++) {
                double t = j * timeStep;
                RotationTarget target = null;
                if (!rotTargets.isEmpty()) {
                    if (Math.abs(rotTargets.get(0).forSegmentIndex(i).getPosition() - t) <= Math.abs(
                            rotTargets.get(0).forSegmentIndex(i).getPosition() - Math.min(t + timeStep, 1.0))) {
                        target = rotTargets.get(0);
                        rotTargets.remove(0);
                    }
                }
                points.add(new PathPoint(splines[i].getPoseWithMotion(j * timeStep), target, i+(j*timeStep)));
            }
        }

        return points;
    }

    public Path replan(Pose2dWithMotion currentState){
        //TODO tests how this affects multi-waypoint paths
        //may have to use the PathPoint relative distance to maintain
        //the desired path shape when replanning

        final double MaxMainPointDistance = 0.25; //meters
        final double ShortestDistSkip = 0.05; //meters
        //save current end point
        PathPoint endPoint = points.get(numPoints()-1);
        PathPoint joinPoint;
        Spline2d join, main;
        List<RotationTarget> rotationTargets = new ArrayList<>();

        //checks where to join the path
        if(currentState.getPose().getTranslation().getDistance(endPoint.position.getPose().getTranslation()) <= MaxMainPointDistance){
            //if we are close enough to the end just return a path that goes to the end
            //note we don't care about rotation targets here becuase we're at the end
            return new Path(
                new Spline2d[]{
                    new Spline2d(currentState, endPoint.position)
                },
                new RotationTarget[]{}
            );
        }else if(currentState.getPose().getTranslation().getDistance(points.get(0).position.getPose().getTranslation()) <= MaxMainPointDistance){
            //if we are close enough to the start just append a path that goes from current 
            //to start then starts path, here we care about rotation targets
            //loop through the rotation targets and add 1 to their waypoint distance
            //since we are adding a waypoint before all of them
            for (RotationTarget target : this.rotationTargets) {
                rotationTargets.add(new RotationTarget(target.getPosition()+1.0, target.getTarget()));
            }
            //returns conjoined path
            return new Path(
                new Spline2d[]{
                    new Spline2d(currentState, points.get(0).position),
                    new Spline2d(points.get(0).position, endPoint.position),
                },
                rotationTargets.toArray(new RotationTarget[0])
            );
        }else{
            //loop through all points to find the closest point to join to
            int joinIndex = 0;
            double shorestDist = Double.MAX_VALUE;

            for (int i = 0; i < points.size(); i++) {
                Spline2d possible = new Spline2d(currentState, points.get(i).position);
                // System.out.println(String.format("%.2f<%.2f", shorestDist, possible.getTotalDistance()));
                if(possible.getTotalDistance()<=shorestDist){
                    shorestDist = possible.getTotalDistance();
                    joinIndex = i;
                    // System.out.println("new join index: "+joinIndex);
                    // if(shorestDist <= ShortestDistSkip){
                    //     //we have a break point that if we are within some small value
                    //     //we just choose that point and move on to save time
                    //     break;
                    // }
                }
            }

            joinPoint = points.get(joinIndex);

            //find the rotation start index
            double rotationStart = joinPoint.waypointRelativeDistance;
            double totalRelativeDistance = endPoint.waypointRelativeDistance-rotationStart;
            //sort through rotation targets to add them in correct spots
            for (RotationTarget target : this.rotationTargets) {
                if(target.getPosition() > rotationStart){
                    rotationTargets.add(
                        new RotationTarget(
                            //shift rotation distance and normalize
                            (target.getPosition()-rotationStart)/totalRelativeDistance - 1.0, 
                            target.getTarget()
                        )
                    );
                }
            }

            System.out.println(currentState.toString());
            System.out.println(joinPoint.position.toString());
            System.out.println(endPoint.position.toString());

            return new Path(
                new Spline2d[]{
                    new Spline2d(currentState, joinPoint.position),
                    new Spline2d(joinPoint.position, endPoint.position), 
                },
                rotationTargets.toArray(new RotationTarget[0])
            );
        }
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

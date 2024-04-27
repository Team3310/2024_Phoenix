package TrajectoryLib.Path;

import TrajectoryLib.Geometry.Pose2dWithMotion;

/** A point along a pathplanner path */
public class PathPoint {
    /** The position of this point */
    public final Pose2dWithMotion position;

    /** The distance of this point along the path, in meters */
    public double distanceAlongPath = 0.0;
    /** The curve radius at this point */
    public double curveRadius = 0.0;
    /** The max velocity at this point */
    public double maxV = Double.POSITIVE_INFINITY;
    /** The target rotation at this point */
    public RotationTarget rotationTarget = null;
    /** The constraints applied to this point */
    public PathConstraints constraints = null;

    /**
     * Create a path point
     *
     * @param position       Position of the point
     * @param rotationTarget Rotation target at this point
     * @param constraints    The constraints at this point
     */
    public PathPoint(
            Pose2dWithMotion position, RotationTarget rotationTarget, PathConstraints constraints) {
        this.position = position;
        this.rotationTarget = rotationTarget;
        this.constraints = constraints;
    }

    /**
     * Create a path point
     *
     * @param position       Position of the point
     * @param rotationTarget Rotation target at this point
     */
    public PathPoint(Pose2dWithMotion position, RotationTarget rotationTarget) {
        this.position = position;
        this.rotationTarget = rotationTarget;
    }

    /**
     * Create a path point
     *
     * @param position Position of the point
     */
    public PathPoint(Pose2dWithMotion position) {
        this.position = position;
    }
}

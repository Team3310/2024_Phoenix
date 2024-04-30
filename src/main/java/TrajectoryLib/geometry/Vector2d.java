package TrajectoryLib.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Vector2d implements Interpolatable<Vector2d> {
    private double magnitude;
    private Rotation2d heading;

    public Vector2d(double dx, double dy) {
        this.magnitude = Math.hypot(dx, dy);
        this.heading = new Rotation2d(dx, dy);
    }

    public Vector2d(ChassisSpeeds speeds) {
        this.magnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        this.heading = new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public Vector2d(double magnitude, Rotation2d heading) {
        this.magnitude = magnitude;
        this.heading = heading;
    }

    public double getX() {
        return magnitude * Math.cos(heading.getRadians());
    }

    public double getY() {
        return magnitude * Math.sin(heading.getRadians());
    }

    public double getMagnitude() {
        return magnitude;
    }

    public Rotation2d getHeading() {
        return heading;
    }

    public Vector2d adjustMagnitude(double newMagnitude) {
        this.magnitude = newMagnitude;
        return this;
    }

    @Override
    public Vector2d interpolate(Vector2d other, double t) {
        return new Vector2d(GeometryUtil.doubleLerp(magnitude, other.magnitude, t),
                heading.interpolate(other.heading, t));
    }
}
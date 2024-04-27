package TrajectoryLib.Geometry;

public class Pose2dWithMotion implements interpolable<Pose2dWithMotion> {
    private Vector2d velocity;
    private Pose2d pose;

    public Pose2dWithMotion(double x, double y, double dx, double dy) {
        this(x, y, Rotation2d.ZERO, dx, dy);
    }

    public Pose2dWithMotion(double x, double y, Rotation2d theta, double dx, double dy) {
        this.pose = new Pose2d(x, y, theta);
        this.velocity = new Vector2d(dx, dy);
    }

    public Pose2dWithMotion(double x, double y, double velocity, Rotation2d heading) {
        this(x, y, Rotation2d.ZERO, velocity, heading);
    }

    public Pose2dWithMotion(double x, double y, Rotation2d theta, double velocity, Rotation2d heading) {
        this.pose = new Pose2d(x, y, theta);
        this.velocity = new Vector2d(velocity, heading);
    }

    public Pose2dWithMotion(double x, double y, Vector2d velocity) {
        this(x, y, Rotation2d.ZERO, velocity);
    }

    public Pose2dWithMotion(double x, double y, Rotation2d theta, Vector2d velocity) {
        this.pose = new Pose2d(x, y, theta);
        this.velocity = velocity;
    }

    public Pose2dWithMotion(Pose2d pose, Vector2d velocity) {
        this.pose = new Pose2d(pose);
        this.velocity = velocity;
    }

    public Vector2d getVelocity() {
        return velocity;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getX() {
        return pose.getX();
    }

    public double getY() {
        return pose.getY();
    }

    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    @Override
    public Pose2dWithMotion interpolate(Pose2dWithMotion other, double t) {
        return new Pose2dWithMotion(
                pose.interpolate(other.pose, t),
                velocity.interpolate(other.velocity, t));
    }
}
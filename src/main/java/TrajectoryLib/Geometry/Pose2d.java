package TrajectoryLib.Geometry;

public class Pose2d implements interpolable<Pose2d> {
    private double x, y;
    private Rotation2d theta;

    public Pose2d(double x, double y) {
        this(x, y, Rotation2d.ZERO);
    }

    public Pose2d(double x, double y, Rotation2d theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Pose2d(Pose2d pose) {
        this(pose.getX(), pose.getY(), pose.getRotation());
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Rotation2d getRotation() {
        return theta;
    }

    /**
     * adds the translation of one pose to another pose
     * 
     * @param other the pose to translate by
     * @return new translated pose
     */
    public Pose2d plus(Pose2d other) {
        return new Pose2d(x + other.x, y + other.y, theta);
    }

    /**
     * subtracts the translation of one pose to another pose
     * 
     * @param other the pose to translate by
     * @return new translated pose
     */
    public Pose2d minus(Pose2d other) {
        return new Pose2d(x - other.x, y - other.y, theta);
    }

    /**
     * Returns the translation multiplied by a scalar.
     *
     * <p>
     * For example, Pose2d(2.0, 2.5) * 2 = Pose2d(4.0, 5.0).
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled translation.
     */
    public Pose2d times(double scalar) {
        return new Pose2d(x * scalar, x * scalar, theta);
    }

    /**
     * Calculates the distance between two translations in 2D space.
     *
     * <p>
     * The distance between translations is defined as √((x₂−x₁)²+(y₂−y₁)²).
     *
     * @param other The translation to compute the distance to.
     * @return The distance between the two translations.
     */
    public double getDistance(Pose2d other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    public Pose2d adjustRotation(Rotation2d newTheta) {
        this.theta = newTheta;
        return this;
    }

    @Override
    public Pose2d interpolate(Pose2d other, double t) {
        return new Pose2d(
                GeometryUtil.doubleLerp(x, other.x, t),
                GeometryUtil.doubleLerp(y, other.y, t),
                theta.interpolate(other.theta, t));
    }
}
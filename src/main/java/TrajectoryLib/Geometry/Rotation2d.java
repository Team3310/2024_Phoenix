package TrajectoryLib.Geometry;

import TrajectoryLib.util.MathUtil;

public class Rotation2d implements interpolable<Rotation2d> {
    public static Rotation2d ZERO = new Rotation2d(0);

    private double theta;

    private Rotation2d(double radians) {
        this.theta = MathUtil.inputModulus(radians, -Math.PI, Math.PI);
    }

    public Rotation2d(double x, double y) {
        double magnitude = Math.hypot(x, y);
        double m_sin, m_cos = 0.0;

        if (magnitude > 1e-6) {
            m_sin = y / magnitude;
            m_cos = x / magnitude;
        } else {
            m_sin = 0.0;
            m_cos = 1.0;
        }

        this.theta = Math.atan2(m_sin, m_cos);
    }

    public static Rotation2d fromRadians(double theta) {
        return new Rotation2d(theta);
    }

    public static Rotation2d fromDegrees(double theta) {
        return new Rotation2d(Math.toRadians(theta));
    }

    public double getDegrees() {
        return Math.toDegrees(theta);
    }

    public double getRadians() {
        return theta;
    }

    public double cos() {
        return Math.cos(theta);
    }

    public double sin() {
        return Math.sin(theta);
    }

    public Rotation2d unaryMinus() {
        return new Rotation2d(-theta);
    }

    public Rotation2d minus(Rotation2d other) {
        return rotateBy(other.unaryMinus());
    }

    public Rotation2d plus(Rotation2d other) {
        return rotateBy(other);
    }

    /**
     * Multiplies the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Rotation2d.
     */
    public Rotation2d times(double scalar) {
        return new Rotation2d(theta * scalar);
    }

    public Rotation2d rotateBy(Rotation2d other) {
        return new Rotation2d(
                cos() * other.cos() - sin() * other.sin(), cos() * other.sin() + sin() * other.cos());
    }

    @Override
    public Rotation2d interpolate(Rotation2d endValue, double t) {
        return plus(endValue.minus(this).times(MathUtil.clamp(t, 0, 1)));
    }

    /**
     * Checks equality between this Rotation2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Rotation2d) {
            var other = (Rotation2d) obj;
            return Math.hypot(cos() - other.cos(), sin() - other.sin()) < 1E-9;
        }
        return false;
    }

    @Override
    public String toString() {
        return String.format("Rotation2d(Rads: %.2f, Deg: %.2f)", theta, Math.toDegrees(theta));
    }
}

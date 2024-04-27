package TrajectoryLib.Geometry;

public class GeometryUtil {

    /**
     * Interpolate between two doubles
     *
     * @param startVal Start value
     * @param endVal   End value
     * @param t        Interpolation factor (0.0-1.0)
     * @return Interpolated value
     */
    public static double doubleLerp(double startVal, double endVal, double t) {
        return startVal + (endVal - startVal) * t;
    }

    /**
     * Calculate the curve radius given 3 points on the curve
     *
     * @param a Point A
     * @param b Point B
     * @param c Point C
     * @return Curve radius
     */
    public static double calculateRadius(Pose2d a, Pose2d b, Pose2d c) {
        Pose2d vba = a.minus(b);
        Pose2d vbc = c.minus(b);
        double cross_z = (vba.getX() * vbc.getY()) - (vba.getY() * vbc.getX());
        int sign = (cross_z < 0) ? 1 : -1;

        double ab = a.getDistance(b);
        double bc = b.getDistance(c);
        double ac = a.getDistance(c);

        double p = (ab + bc + ac) / 2;
        double area = Math.sqrt(Math.abs(p * (p - ab) * (p - bc) * (p - ac)));
        return sign * (ab * bc * ac) / (4 * area);
    }
}

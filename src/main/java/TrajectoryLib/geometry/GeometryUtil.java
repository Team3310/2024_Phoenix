package TrajectoryLib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

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
        Translation2d vba = a.getTranslation().minus(b.getTranslation());
        Translation2d vbc = c.getTranslation().minus(b.getTranslation());
        double cross_z = (vba.getX() * vbc.getY()) - (vba.getY() * vbc.getX());
        int sign = (cross_z < 0) ? 1 : -1;

        double ab = a.getTranslation().getDistance(b.getTranslation());
        double bc = b.getTranslation().getDistance(c.getTranslation());
        double ac = a.getTranslation().getDistance(c.getTranslation());

        double p = (ab + bc + ac) / 2;
        double area = Math.sqrt(Math.abs(p * (p - ab) * (p - bc) * (p - ac)));
        return sign * (ab * bc * ac) / (4 * area);
    }
}

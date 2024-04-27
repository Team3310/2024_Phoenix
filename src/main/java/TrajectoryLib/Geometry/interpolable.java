package TrajectoryLib.Geometry;

public interface interpolable<T> {
    T interpolate(T other, double t);
}

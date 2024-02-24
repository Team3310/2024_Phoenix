package frc.robot.util.Interpolable;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}

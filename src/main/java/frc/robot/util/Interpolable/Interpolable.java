package frc.robot.util.Interpolable;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}

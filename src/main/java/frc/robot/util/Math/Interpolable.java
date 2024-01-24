package frc.robot.util.Math;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}

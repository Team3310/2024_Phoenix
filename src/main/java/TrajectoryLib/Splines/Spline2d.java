package TrajectoryLib.Splines;

import java.util.ArrayList;
import java.util.List;

import TrajectoryLib.Geometry.Pose2d;
import TrajectoryLib.Geometry.Pose2dWithMotion;
import TrajectoryLib.Geometry.Rotation2d;
import TrajectoryLib.Geometry.Vector2d;

public class Spline2d{
    private Spline x, y;
    private Pose2dWithMotion start, end;

    public Spline2d(Pose2dWithMotion start, Pose2dWithMotion end){
        this.x = new Spline(start.getX(), start.getVelocity().getX(), end.getX(), end.getVelocity().getX());
        this.y = new Spline(start.getY(), start.getVelocity().getY(), end.getY(), end.getVelocity().getY());

        this.start = start;
        this.end = end;
    }

    public List<Pose2dWithMotion> getSamplePoints(int numPoints){
        List<Pose2dWithMotion> points = new ArrayList<>();

        double timeStep = 1.0/((double)numPoints);

        for (int i = 0; i < numPoints; i++) {
            points.add(getPoseWithMotion(i*timeStep));
        }

        return points;
    }

    public Pose2dWithMotion getPoseWithMotion(double t){
        return new Pose2dWithMotion(getPose(t), getVelocity(t));
    }

    public Pose2d getPose(double t){
        return new Pose2d(x.getPosition(t), y.getPosition(t), getRotation(t));
    }

    public Vector2d getVelocity(double t){
        return new Vector2d(x.getVelocity(t), y.getVelocity(t));
    }

    public Rotation2d getRotation(double t){
        return start.getRotation().interpolate(end.getRotation(), t);
    }

    public void printCoefficients(){
        printXCoefficients();
        printYCoefficients();
    }

    public void printXCoefficients(){
        x.printCoefficients();
    }

    public void printYCoefficients(){
        y.printCoefficients();
    }
}
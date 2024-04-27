package TrajectoryLib.Splines;

import java.util.ArrayList;
import java.util.List;

import TrajectoryLib.geometry.Pose2dWithMotion;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Spline2d{
    private Spline x, y;
    private Pose2dWithMotion start, end;

    public Spline2d(Pose2dWithMotion start, Pose2dWithMotion end){
        this.x = new Spline(start.getX(), start.getVelocities().vxMetersPerSecond, end.getX(), end.getVelocities().vxMetersPerSecond);
        this.y = new Spline(start.getY(), start.getVelocities().vyMetersPerSecond, end.getY(), end.getVelocities().vyMetersPerSecond);

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

    public ChassisSpeeds getVelocity(double t){
        return new ChassisSpeeds(x.getVelocity(t), y.getVelocity(t), 0.0);
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
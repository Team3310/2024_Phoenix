package TrajectoryLib.util;

import com.pathplanner.lib.util.PPLibTelemetry;

import TrajectoryLib.geometry.Pose2dWithMotion;
import TrajectoryLib.path.Path;
import TrajectoryLib.path.RotationTarget;
import TrajectoryLib.path.Trajectory;
import TrajectoryLib.path.Trajectory.State;
import TrajectoryLib.splines.Spline2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class TrajectoryTests {
    public static void main(String[] args) {
        try {
            // splineDistanceTest();
            trajectoryReplanTest(0.05);
            // OptimizingTest();
        } catch (Exception e) {
            e.printStackTrace();
        }
        
    }

    public static void OptimizingTest(){
        Pose2dWithMotion start = new Pose2dWithMotion(5.14, 3.77, Rotation2d.fromDegrees(0.0), 0.0, 0.0, 0.0);
        Pose2dWithMotion end = new Pose2dWithMotion(8.9, 2.64, Rotation2d.fromDegrees(0.0), 1, -1.8, 0.0);

        Spline2d[] splines = {
            new Spline2d(start, end)
        };
        RotationTarget[] rotationTargets = {
            new RotationTarget(0.25, Rotation2d.fromDegrees(45.0)),
            new RotationTarget(0.75, Rotation2d.fromDegrees(135)),
        };

        Path path = new Path(splines, rotationTargets);
        path = path.replan(new 
            Pose2dWithMotion(
                new Pose2d(7.5, 3.0, Rotation2d.fromDegrees(0.0)), 
                new ChassisSpeeds(0.0, 0.0, 0)
            )
        );
        Trajectory traj = new Trajectory(path, start.getVelocities(), start.getRotation());

        splines[0].printCoefficients();

        for(int i=0; i<5; i++){
            start = traj.getInitialState().getState();
            end = traj.getEndState().getState();
            
            splines[0] = new Spline2d(start, end);

            // splines[0].printCoefficients();

            path = new Path(splines, rotationTargets);
            traj = new Trajectory(path, start.getVelocities(), start.getRotation());
        }
    }

    public static void splineDistanceTest(){
        Pose2dWithMotion start = new Pose2dWithMotion(5.14, 3.77, Rotation2d.fromDegrees(0.0), 0.0, 0.0, 0.0);
        Pose2dWithMotion end = new Pose2dWithMotion(8.9, 2.64, Rotation2d.fromDegrees(0.0), 1, -1.8, 0.0);
        Pose2dWithMotion end2 = new Pose2dWithMotion(0.0, 3.77, Rotation2d.fromDegrees(0.0), 0, 0, 0.0);

        Spline2d[] splines = {
            new Spline2d(start, end),
            new Spline2d(start, end2)
        };
        

        System.out.println(splines[0].getTotalDistance());
        System.out.println(splines[1].getTotalDistance());
    }

    public static void trajectoryReplanTest(double resolution){
        Pose2dWithMotion start = new Pose2dWithMotion(5.14, 3.77, Rotation2d.fromDegrees(0.0), 0.0, 0.0, 0.0);
        Pose2dWithMotion end = new Pose2dWithMotion(8.9, 2.64, Rotation2d.fromDegrees(0.0), 1, -1.8, 0.0);

        Spline2d[] splines = {
            new Spline2d(start, end)
        };
        RotationTarget[] rotationTargets = {
            new RotationTarget(0.25, Rotation2d.fromDegrees(45.0)),
            new RotationTarget(0.75, Rotation2d.fromDegrees(135)),
        };

        Path path = new Path(splines, rotationTargets);
        System.out.println(path.getPoint(path.numPoints()-1).position.toString());
        path = path.replan(new 
            Pose2dWithMotion(
                new Pose2d(7.5, 3.0, Rotation2d.fromDegrees(0.0)), 
                new ChassisSpeeds(0.0, 0.0, 0)
            )
        );
        Trajectory traj = new Trajectory(path, path.getPoint(0).position.getVelocities(), Rotation2d.fromDegrees(0.0));

        String x = "", y = "", dx = "", dy = "";

        for (double t = 0; t < traj.getTotaltime(); t+=resolution) {
            State state = traj.sample(t);

            // System.out.println(state);

            x += Double.toString(state.getTargetPose().getX())+", ";
            y += Double.toString(state.getTargetPose().getY())+", ";
            dx += Double.toString(state.getTargetVectorVelocity().getX()+state.getTargetPose().getX())+", ";
            dy += Double.toString(state.getTargetVectorVelocity().getY()+state.getTargetPose().getY())+", ";
        }

        System.out.println("Total Path Time: "+traj.getTotaltime());
        System.out.println("X :\n"+x);
        System.out.println("Y :\n"+y);
        System.out.println("dX :\n"+dx);
        System.out.println("dY :\n"+dy);
    }

    public static void trajectoryPPLibLogTest(){
        Pose2dWithMotion start = new Pose2dWithMotion(5.14, 3.77, Rotation2d.fromDegrees(0.0), 0.0, 0.0, 0.0);
        Pose2dWithMotion end = new Pose2dWithMotion(8.9, 2.64, Rotation2d.fromDegrees(0.0), 1, -1.8, 0.0);

        Spline2d[] splines = {
            new Spline2d(start, end)
        };
        RotationTarget[] rotationTargets = {
            new RotationTarget(0.25, Rotation2d.fromDegrees(45.0)),
            new RotationTarget(0.75, Rotation2d.fromDegrees(135)),
        };

        Path path = new Path(splines, rotationTargets);
        Trajectory traj = new Trajectory(path, start.getVelocities(), start.getRotation());

        PPLibTelemetry.setCurrentPath(path);

        Timer timer = new Timer();
        timer.start();

        while (!timer.hasElapsed(traj.getTotaltime())) {
            System.out.println(timer.get());
            PPLibTelemetry.setVelocities(0.0, traj.sample(timer.get()).getTargetVectorVelocity().getMagnitude(), 0.0, traj.sample(timer.get()).getTargetVelocity().omegaRadiansPerSecond);
            PPLibTelemetry.setTargetPose(traj.sample(timer.get()).getTargetPose());
        }
    }

    public static void trajectoryGenerationTest(double resolution){
        //logs and records the (x, y) and (dx, dy) of the generated trajectory
        Pose2dWithMotion start = new Pose2dWithMotion(5.14, 3.77, Rotation2d.fromDegrees(0.0), 0.0, 0.0, 0.0);
        Pose2dWithMotion end = new Pose2dWithMotion(8.9, 2.64, Rotation2d.fromDegrees(0.0), 1, -1.8, 0.0);
        Spline2d[] splines = {
            new Spline2d(start, end)
        };
        RotationTarget[] rotationTargets = {
            new RotationTarget(0.25, Rotation2d.fromDegrees(45.0)),
            new RotationTarget(0.75, Rotation2d.fromDegrees(135)),
        };

        Path path = new Path(splines, rotationTargets);

        Trajectory traj = new Trajectory(path, start.getVelocities(), start.getRotation());

        String x = "", y = "", dx = "", dy = "";

        for (double t = 0; t < traj.getTotaltime(); t+=resolution) {
            State state = traj.sample(t);

            System.out.println(state);

            x += Double.toString(state.getTargetPose().getX())+", ";
            y += Double.toString(state.getTargetPose().getY())+", ";
            dx += Double.toString(state.getTargetVectorVelocity().getX()+state.getTargetPose().getX())+", ";
            dy += Double.toString(state.getTargetVectorVelocity().getY()+state.getTargetPose().getY())+", ";
        }

        System.out.println("Total Path Time: "+traj.getTotaltime());
        System.out.println("X :\n"+x);
        System.out.println("Y :\n"+y);
        System.out.println("dX :\n"+dx);
        System.out.println("dY :\n"+dy);
    }

    public static void splineGenerationTest(){
        //these come from a predefined path that has been calculated outside the code
        //the intent is to test that the math in the code matches expectations
        Pose2dWithMotion start = new Pose2dWithMotion(5.14, 3.77, Rotation2d.fromDegrees(0.0), 0.0, 0.0, 0.0);
        Pose2dWithMotion end = new Pose2dWithMotion(8.9, 2.64, Rotation2d.fromDegrees(0.0), 1, -1.8, 0.0);
        Spline2d spline = new Spline2d(start, end);

        spline.printXCoefficients();
        System.out.println(String.format("c0: %.2f, c1: %.2f, c2: %.2f, c3: %.2f, c4: %.2f, c5: %.2f", 5.14, 0.0, 0.0, 33.6, -49.4, 19.65));
        System.out.println();
        spline.printYCoefficients();
        System.out.println(String.format("c0: %.2f, c1: %.2f, c2: %.2f, c3: %.2f, c4: %.2f, c5: %.2f", 3.77, 0.0, 0.0, -4.1, 4.35, -1.38));
    }

    public static void rotationInterpTest(){
        Rotation2d start = Rotation2d.fromDegrees(Math.random()*180*(Math.random()>0.5?-1:1));
        Rotation2d end = Rotation2d.fromDegrees(Math.random()*180*(Math.random()>0.5?-1:1));

        System.out.println(start);
        System.out.println(end);
        System.out.println();

        for(double t = 0.0; t<=1.0; t+=0.2){
            System.out.println(start.interpolate(end, t));
        }

        end = end.unaryMinus();
        System.out.println();

        for(double t = 0.0; t<=1.0; t+=0.2){
            System.out.println(start.interpolate(end, t));
        }

        System.out.println("\nend\n");
    }
}

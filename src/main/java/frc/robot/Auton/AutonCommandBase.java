package frc.robot.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Drive.WaitUntilAtXBoundary;
import frc.robot.Commands.Intake.IntakeAmp;
import frc.robot.Commands.Intake.IntakeShooter;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Choosers.SideChooser.SideMode;

public class AutonCommandBase extends SequentialCommandGroup {
    protected RobotContainer robotContainer;
    private final double NOTE_DECISION_LINE = 7.25; //meters on blue side

    protected AutonCommandBase(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
    }

    protected Command Follow(PathPlannerPath path) {
        return new FollowPathCommand(path);
    }

    protected Command follow(String pathName) {
        return new FollowPathCommand(getPath(pathName));
    }

    protected void resetRobotPose(PathPlannerPath path) {
        path.getTrajectory(new ChassisSpeeds(0, 0, 0), path.getStartingDifferentialPose().getRotation()).getInitialDifferentialPose();
        Pose2d start = path.getPreviewStartingHolonomicPose();
        
        TunerConstants.DriveTrain.seedFieldRelative(start);   
        TunerConstants.DriveTrain.seedFieldRelativeWithOffset(start.getRotation());  
    }

    protected PathPlannerPath getPath(String pathName){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        if(robotContainer.getDrivetrain().getSideMode() == SideMode.RED){
            path.preventFlipping = false;
            return path.flipPath();
        }
        return path;
    }

    protected Double getPathTime(String pathName){
        return getPath(pathName).getTrajectory(new ChassisSpeeds(0, 0, 0), Rotation2d.fromDegrees(0)).getTotalTimeSeconds();
    }

    protected ParallelDeadlineGroup FollowToIntake(PathPlannerPath path){
        return FollowToIntake(path, false);
    }

    protected ParallelDeadlineGroup FollowToIntake(PathPlannerPath path, boolean track){
        return new ParallelDeadlineGroup(Follow(path).andThen(new WaitUntilCommand(()->Shooter.getInstance().hasNote()||!TunerConstants.DriveTrain.isTrackingNote).withTimeout(0.15)), new IntakeShooter(track));
    }

    protected ParallelDeadlineGroup FollowToAmpIntake(PathPlannerPath path){
        return new ParallelDeadlineGroup(Follow(path), new IntakeAmp());

    }

    protected Command GoToShoot(RobotContainer container, PathPlannerPath path){
        return
            new ParallelRaceGroup( 
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        Follow(path),
                        // new SetLiftAngle(Lift.getInstance(), Constants.LIFT_MIN_DEGREES)
                        new WaitUntilCommand(()->!container.shooter.hasNote()).andThen(new IntakeShooter())
                    ),
                    AimAndShoot(robotContainer)
                ),
                new SequentialCommandGroup(
                    new WaitUntilAtXBoundary(NOTE_DECISION_LINE, container),
                    new WaitUntilCommand(()->!container.shooter.hasNote())
                )
            );
    }

    protected Command GoToShoot(RobotContainer container, PathPlannerPath path, boolean end){
        return
            new ParallelRaceGroup( 
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        Follow(path),
                        // new SetLiftAngle(Lift.getInstance(), Constants.LIFT_MIN_DEGREES)
                        new WaitUntilCommand(()->!container.shooter.hasNote()).andThen(new IntakeShooter())
                    ),
                    AimAndShoot(robotContainer)
                ),
                new SequentialCommandGroup(
                    new WaitUntilAtXBoundary(NOTE_DECISION_LINE, container),
                    new WaitUntilCommand(()->(!container.shooter.hasNote() && end))
                )
            );
    }

    protected Command AimAndShoot(RobotContainer container){
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new AimLiftWithOdometryAuton().withTimeout(0.25),
                    new SetDriveMode(DriveMode.AIMATTARGET).andThen(new WaitUntilCommand(()->container.getDrivetrain().snapComplete()))
                ),
                new WaitCommand(0.1),
                new FeederShootCommandAuton(container.shooter).withTimeout(0.15)
            );
    }
}

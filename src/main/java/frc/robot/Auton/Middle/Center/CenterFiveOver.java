package frc.robot.Auton.Middle.Center;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Drive.WaitUntilAtXBoundary;
import frc.robot.Commands.Intake.IntakeShooter;
import frc.robot.Commands.Intake.IntakeThrough;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;

public class CenterFiveOver extends AutonCommandBase{

    public CenterFiveOver(RobotContainer robotContainer) {
        super(robotContainer);

        resetRobotPose(Paths.getInstance().C_N);
        
        this.addCommands(
            new OneAuton(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().CENTER_THROUGH), 
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitUntilAtXBoundary(4.93, robotContainer),
                        new SetLiftAngle(Lift.getInstance(), Constants.LIFT_MIN_DEGREES+1.0),
                        new SetLeftShooterRPM(Shooter.getInstance(), 3275.0-200.0),
                        new SetRightShooterRPM(Shooter.getInstance(), 3400.0-200.0),
                        new IntakeThrough(robotContainer.intake)
                    ),
                    new IntakeShooter()
                )
            ),
            GoToShoot(robotContainer, Paths.getInstance().CCN_CS, false),
            FollowToIntake(Paths.getInstance().CS_CENTER),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().CENTER_AMP),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().CENTER_PODIUM),
            AimAndShoot(robotContainer)
        );
    }
    
}

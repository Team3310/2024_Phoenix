package frc.robot.Auton.Middle.Center.over;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Drive.WaitUntilAtXBoundaryClose;
import frc.robot.Commands.Intake.IntakeShooter;
import frc.robot.Commands.Intake.IntakeThrough;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;

public class CenterOverStartCenter extends AutonCommandBase{

    public CenterOverStartCenter(RobotContainer robotContainer) {
        super(robotContainer);

        resetRobotPose(Paths.getInstance().C_N);
        
        this.addCommands(
            new OneAuton(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().CENTER_THROUGH), 
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitUntilAtXBoundaryClose(6.41, robotContainer),
                        new IntakeThrough(robotContainer.intake),
                        new SetLiftAngle(Lift.getInstance(), Constants.LIFT_MIN_DEGREES+1.0),
                        new SetLeftShooterRPM(Shooter.getInstance(), 3275.0-200.0),
                       new SetRightShooterRPM(Shooter.getInstance(), 3400.0-200.0)
                    ),
                    new IntakeShooter(true)
                )
            ),
            GoToShoot(robotContainer, Paths.getInstance().CCN_CS, false),
            FollowToIntake(Paths.getInstance().CS_CENTER),
            AimAndShoot(robotContainer)
        );
    }
    
}

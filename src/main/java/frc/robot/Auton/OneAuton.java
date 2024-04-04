package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;

public class OneAuton extends AutonCommandBase{
    public OneAuton(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(path);

        this.addCommands(
            new ParallelDeadlineGroup(
                new SetLiftAngle(robotContainer.lift, 60.0)
                    .andThen(new WaitUntilCommand(()->robotContainer.lift.isFinished()).withTimeout(1.0)), 
                new SetLeftShooterRPM(robotContainer.shooter, 2700),
                new SetRightShooterRPM(robotContainer.shooter, 2200)
            ),
            new WaitCommand(0.1),
            new ParallelDeadlineGroup(
                new WaitCommand(0.35), 
                new FeederShootCommandAuton(robotContainer.shooter)
            )
        );
    }
}

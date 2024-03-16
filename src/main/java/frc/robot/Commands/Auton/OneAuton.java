package frc.robot.Commands.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
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
                    .andThen(new WaitUntilCommand(()->robotContainer.lift.isFinished())), 
                new SetLeftShooterRPM(robotContainer.shooter, 3000),
                new SetRightShooterRPM(robotContainer.shooter, 2000)
            ),
            new WaitCommand(0.2),
            new ParallelDeadlineGroup(
                new WaitCommand(0.2), 
                new FeederShootCommandAuton(robotContainer.shooter)
            )
        );
    }
}

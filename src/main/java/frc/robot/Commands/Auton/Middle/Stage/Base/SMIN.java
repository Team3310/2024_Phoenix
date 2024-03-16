package frc.robot.Commands.Auton.Middle.Stage.Base;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;

public class SMIN extends AutonCommandBase{
    public SMIN(RobotContainer container){
        super(container);

        this.addCommands(
            FollowToIntake(Paths.getInstance().SM_IN),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().IN_SM),
                new SetLeftShooterRPM(robotContainer.shooter, 5500),
                new SetRightShooterRPM(robotContainer.shooter, 3500)
            ),
            AimAndShoot(robotContainer)
        );
    }
}

package frc.robot.Commands.Auton.Middle.Stage;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Auton.Middle.Stage.Base.StageON;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;

public class StageThreeON extends AutonCommandBase{
    public StageThreeON(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageON(robotContainer),
            FollowToIntake(Paths.getInstance().SM_IN),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().IN_SM3),
                new SetLeftShooterRPM(robotContainer.shooter, 5500),
                new SetRightShooterRPM(robotContainer.shooter, 3500)
            ),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().SM_GRAB_CLOSE),
            AimAndShoot(robotContainer)
        );
    }
}

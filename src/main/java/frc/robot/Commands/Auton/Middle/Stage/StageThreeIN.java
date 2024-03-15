package frc.robot.Commands.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;

public class StageThreeIN extends AutonCommandBase{
    public StageThreeIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageIN(robotContainer),
            FollowToIntake(Paths.getInstance().SM_ON3),
            Follow(Paths.getInstance().ON_SM3),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().SM_GRAB_CLOSE),
            AimAndShoot(robotContainer)
        );
    }
}

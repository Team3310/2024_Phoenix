package frc.robot.Commands.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Auton.Middle.Stage.Base.StageIN;

public class StageThreeIN extends AutonCommandBase{
    public StageThreeIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageIN(robotContainer),
            FollowToIntake(Paths.getInstance().SM_ON3),
            GoToShoot(robotContainer, Paths.getInstance().ON_SM3),
            FollowToIntake(Paths.getInstance().SM_GRAB_CLOSE),
            AimAndShoot(robotContainer)
        );
    }
}

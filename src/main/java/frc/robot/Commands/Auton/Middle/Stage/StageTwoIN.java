package frc.robot.Commands.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Auton.Middle.Stage.Base.StageIN;

public class StageTwoIN extends AutonCommandBase{
    public StageTwoIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageIN(robotContainer),
            FollowToIntake(Paths.getInstance().SM_ON),
            GoToShoot(robotContainer, Paths.getInstance().ON_SM)
        );
    }
}

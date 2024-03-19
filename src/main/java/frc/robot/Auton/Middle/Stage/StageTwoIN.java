package frc.robot.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Stage.Base.StageIN;

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

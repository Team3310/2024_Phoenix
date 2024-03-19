package frc.robot.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Stage.Base.StageON;

public class StageTwoON extends AutonCommandBase{
    public StageTwoON(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageON(robotContainer),
            FollowToIntake(Paths.getInstance().SM_IN),
            GoToShoot(robotContainer, Paths.getInstance().IN_SM)
        );
    }
}

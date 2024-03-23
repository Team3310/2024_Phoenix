package frc.robot.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class StageThreeINMiddle extends AutonCommandBase{
    public StageThreeINMiddle(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageTwoINMiddle(robotContainer),
            FollowToIntake(Paths.getInstance().MM_CN),
            GoToShoot(robotContainer, Paths.getInstance().CN_MM)
        );
    }
}

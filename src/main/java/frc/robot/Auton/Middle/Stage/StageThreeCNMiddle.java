package frc.robot.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class StageThreeCNMiddle extends AutonCommandBase{
    public StageThreeCNMiddle(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageTwoCNMiddle(robotContainer),
            FollowToIntake(Paths.getInstance().MM_IN),
            GoToShoot(robotContainer, Paths.getInstance().IN_MM)
        );
    }
}

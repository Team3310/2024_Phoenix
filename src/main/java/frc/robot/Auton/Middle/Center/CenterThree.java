package frc.robot.Auton.Middle.Center;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterThree extends AutonCommandBase{
    public CenterThree(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterTwo(robotContainer),
            FollowToIntake(Paths.getInstance().CN_CCN),
            GoToShoot(robotContainer, Paths.getInstance().CN_S)
        );
    }
}

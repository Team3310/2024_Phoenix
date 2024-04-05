package frc.robot.Auton.Middle.Center.base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterThreeAIN extends AutonCommandBase{
    public CenterThreeAIN(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterTwo(robotContainer),
            FollowToIntake(Paths.getInstance().CN_AIN),
            GoToShoot(robotContainer, Paths.getInstance().AIN_CS)
        );
    }
}
